/*****************************************************************************
* File Name: event-control.c
*
* Version 1.00
*
* Description:
*   This file contains the declarations of all the high-level APIs.
*
* Note:
*   N/A
*
* Owner:
*   vinhlq
*
* Related Document:
*
* Hardware Dependency:
*   N/A
*
* Code Tested With:
*
******************************************************************************
* Copyright (2019), vinhlq.
******************************************************************************
* This software is owned by vinhlq and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* (vinhlq) hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* (vinhlq) Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a (vinhlq) integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of (vinhlq).
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* (vinhlq) reserves the right to make changes without further notice to the
* materials described herein. (vinhlq) does not assume any liability arising out
* of the application or use of any product or circuit described herein. (vinhlq)
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of (vinhlq)' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies (vinhlq) against all charges. Use may be
* limited by and subject to the applicable (vinhlq) software license agreement.
*****************************************************************************/

/*******************************************************************************
* Included headers
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "event-control.h"


/*******************************************************************************
* Macro
*******************************************************************************/

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )
#define FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
#endif

#define FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE

#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
#define debugPrintln(fmt,args...)	printf(fmt "%s", ## args, "\r\n")
#else
#define debugPrintln(...)
#endif

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
	static xTimerHandle timerdeleteQueue;
#endif

#if FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
static EventGroupHandle_t eventControlEventGroup[(FREERTOS_EVENT_CONTROL_MAX_EVENT/sizeof(EventBits_t)) + 1]={NULL};
#else
static xQueueHandle eventControlQueue=NULL;
#endif
static FreertosEventControl_t *eventControls[FREERTOS_EVENT_CONTROL_MAX_EVENT];
static FreertosEventBits_t eventControlEventBits[(FREERTOS_EVENT_CONTROL_MAX_EVENT/sizeof(FreertosEventBits_t)) + 1];

#define eventControlEventGetEventGroup(eventNumber)		eventControlEventGroup[eventNumber/sizeof(EventBits_t)]
#define eventControlEventGetEventGroupBit(eventNumber)	(eventNumber%sizeof(EventBits_t))
#define eventControlEventBitsClear(eventNumber)		eventControlEventBits[eventNumber/sizeof(FreertosEventBits_t)] &= ~(1<<(eventNumber%sizeof(FreertosEventBits_t)))
#define eventControlEventBitsSet(eventNumber)		eventControlEventBits[eventNumber/sizeof(FreertosEventBits_t)] |= (1<<(eventNumber%sizeof(FreertosEventBits_t)))
#define eventControlEventNumberIsSet(eventNumber)	(eventControlEventBits[eventNumber/sizeof(FreertosEventBits_t)] & (1<<(eventNumber%sizeof(FreertosEventBits_t))))

void freertosEventControlSetInactive(FreertosEventNumber_t eventToClear)
{
	if(eventToClear >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	taskENTER_CRITICAL();
	eventControlEventBitsClear(eventToClear);
	taskEXIT_CRITICAL();
#ifdef FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
	xEventGroupClearBits( eventControlEventGetEventGroup(eventToClear), eventControlEventGetEventGroupBit(eventToClear) );
#endif
}

void freertosEventControlSetActive(FreertosEventNumber_t eventToSet)
{
	if(eventToSet >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	taskENTER_CRITICAL();
	eventControlEventBitsSet(eventToSet);
	taskEXIT_CRITICAL();
#ifdef FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
	xEventGroupSetBits( eventControlEventGetEventGroup(eventToSet), eventControlEventGetEventGroupBit(eventToSet) );
#else
	xQueueSend(eventControlQueue, (void *)&eventToSet, portMAX_DELAY);
#endif
}

void freertosEventControlSetInactiveFromISR(FreertosEventNumber_t eventToClear)
{
	if(eventToClear >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	eventControlEventBitsClear(eventToClear);
#ifdef FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
	xEventGroupClearBitsFromISR( eventControlEventGetEventGroup(eventToClear), eventControlEventGetEventGroupBit(eventToClear));
#endif
}

void freertosEventControlSetActiveFromISR(FreertosEventNumber_t eventToSet, portBASE_TYPE *pxHigherPriorityTaskWoken)
{
	if(eventToSet >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	eventControlEventBitsSet(eventToSet);
#ifdef FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
	xEventGroupSetBitsFromISR( eventControlEventGetEventGroup(eventToSet), eventControlEventGetEventGroupBit(eventToSet), pxHigherPriorityTaskWoken );
#else
	xQueueSendFromISR(eventControlQueue, (void *)&eventToSet, pxHigherPriorityTaskWoken);
#endif
}

bool freertosEventControlIsActivated(FreertosEventNumber_t eventNumber)
{
	return eventControlEventNumberIsSet(eventNumber);
}

static void vTimerSetEventCallback( xTimerHandle xTimer )
{
	FreertosEventNumber_t eventToSet;
	portBASE_TYPE xReturn;

	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT( xTimer );

	/* event is saved as the
	timer's ID */
	eventToSet = ( FreertosEventNumber_t ) pvTimerGetTimerID( xTimer );

	/* delete */
#ifdef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
	xReturn = xTimerDelete(xTimer, portMAX_DELAY);
	debugPrintln("Event control timer delete: %s: %u", xReturn == pdPASS ? "pass":"fail", (uint32_t)xReturn);
	configASSERT(xReturn == pdPASS);
#else
	xReturn = xQueueSend(timerdeleteQueue, (void *)&xTimer, 0);
	debugPrintln("Event control timer queue for delete: %s: %u", xReturn == pdPASS ? "pass":"fail", (uint32_t)xReturn);
	configASSERT(xReturn == pdPASS);
#endif

	freertosEventControlSetActive( eventToSet );
}

void freertosEventControlSetActiveDelayMS(FreertosEventNumber_t eventToSet, uint32_t timerPeriodMs)
{
	eventControlEventBitsClear(eventToSet);
	xTimerHandle capsenseTimer = xTimerCreate(	"pcTimerName",
												timerPeriodMs/portTICK_RATE_MS,
												pdFALSE,
												(void *)eventToSet,
												vTimerSetEventCallback );
	xTimerStart(capsenseTimer, 0);
}

static void freertosEventControlTask(void *param)
{
	while(1)
	{
		int i;

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
		xTimerHandle eventControlTimer;
		if (xQueueReceive(timerdeleteQueue, &eventControlTimer, 0))
		{
			xTimerDelete(capsenseTimer, portMAX_DELAY);
		}
#endif

#ifdef FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
		EventBits_t eventBits = xEventGroupWaitBits(eventControlEventGroup, eventControlBits, pdFALSE, pdTRUE, portMAX_DELAY);
		if(eventControlEventBits & eventBits)
		{
			// re-queue un-clear item
			xEventGroupSetBits( eventControlEventGroup, eventControlEventBits & eventBits );
		}
#else
		FreertosEventNumber_t eventNumber;
		if (!xQueueReceive(eventControlQueue, &eventNumber, portMAX_DELAY))
		{
			continue;
		}

		if( eventNumber >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
		{
			printf("Invalid event number: %u", eventNumber);
			continue;
		}

		if(eventControlEventNumberIsSet(eventNumber))
		{
			xQueueSend(eventControlQueue, (void *)&eventNumber, 10);
		}


		if(eventControls[eventNumber])
		{
			eventControls[eventNumber]->callback(eventControls[eventNumber]->args);
		}
#endif
	}
}

void freertosEventControlInit(const configSTACK_DEPTH_TYPE usStackDepth, UBaseType_t uxPriority)
{
	uint8_t i;

#ifdef FREERTOS_EVENT_CONTROL_USE_EVENT_GROUP
	eventControlEventGroup = xEventGroupCreate();
#else
	eventControlQueue = xQueueCreate(FREERTOS_EVENT_CONTROL_MAX_EVENT, sizeof(FreertosEventNumber_t));
	configASSERT( eventControlQueue );
#endif

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
	timerdeleteQueue = xQueueCreate(8, sizeof(xTimerHandle));
#endif

	for(i = 0; i < FREERTOS_EVENT_CONTROL_MAX_EVENT; i++)
	{
		eventControls[i] = NULL;
	}
	for(i = 0; i < ((FREERTOS_EVENT_CONTROL_MAX_EVENT/sizeof(FreertosEventBits_t)) + 1); i++)
	{
		eventControlEventBits[i] = 0;
	}

	xTaskCreate(freertosEventControlTask, "event control task", usStackDepth, NULL, uxPriority, NULL);
}

uint16_t freertosEventControlRegister(const FreertosEventControl_t *eventControl)
{
	uint8_t i;

	for(i = 0; i < FREERTOS_EVENT_CONTROL_MAX_EVENT; i++)
	{
		if(!eventControlEventNumberIsSet(i))
		{
			eventControls[i] = eventControl;
			break;
		}
	}
	return i;
}

/****************************End of File***************************************/
