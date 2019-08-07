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
#include "freertos/timers.h"

#include "event-control.h"

/*******************************************************************************
* API Constants
*******************************************************************************/

#ifndef FREERTOS_EVENT_CONTROL_TASK_STATIC_STACK_SIZE
#define FREERTOS_EVENT_CONTROL_TASK_STATIC_STACK_SIZE	(2048)
#endif

#ifndef FREERTOS_EVENT_CONTROL_QUEUE_SIZE
#define FREERTOS_EVENT_CONTROL_QUEUE_SIZE	FREERTOS_EVENT_CONTROL_MAX_EVENT
#endif

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
#ifndef FREERTOS_EVENT_CONTROL_TIMER_DELETE_QUEUE_SIZE
#define FREERTOS_EVENT_CONTROL_TIMER_DELETE_QUEUE_SIZE	(8)
#endif
#endif

/*******************************************************************************
* Macro
*******************************************************************************/
#define FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE

#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
#define debugPrintln(fmt,args...)	\
	printf(fmt "%s", ## args, "\r\n");

#define evtDebugPrintln(eventNumber, fmt,args...)	\
	do {	\
		if(eventNumber == FREERTOS_EVENT_CONTROL_MAX_EVENT || eventControlFlags[eventNumber] & FREERTOS_EVENT_CONTROL_FLAG_DEBUG_ENABLED)	{	\
			debugPrintln(fmt, ## args);	\
		}	\
	}while(0)
#else
#define debugPrintln(...)
#define evtDebugPrintln(eventNumber, fmt,args...)
#endif

#define FREERTOS_EVENT_CONTROL_BLOCKING_MAXIMUM	(100)

typedef struct
{
	FreertosEventNumber_t eventNumber;
	uint16_t delayPeriodMs;
}EventQueueItem_t;

static xTaskHandle eventControlTaskHandle = NULL;
#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
static StaticTask_t eventControlTaskBuffer;
static StackType_t eventControlTaskStack[ FREERTOS_EVENT_CONTROL_TASK_STATIC_STACK_SIZE ];

/* The variable used to hold the queue's data structure. */
static StaticQueue_t eventControlQueue;

/* The array to use as the queue's storage area.  This must be at least
uxQueueLength * uxItemSize bytes. */
static uint8_t eventControlQueueStorageArea[ FREERTOS_EVENT_CONTROL_QUEUE_SIZE * sizeof(EventQueueItem_t) ];

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
/* The variable used to hold the queue's data structure. */
static StaticQueue_t timerDeleteQueue;

/* The array to use as the queue's storage area.  This must be at least
uxQueueLength * uxItemSize bytes. */
static uint8_t timerDeleteQueueStorageArea[ FREERTOS_EVENT_CONTROL_TIMER_DELETE_QUEUE_SIZE * sizeof(xTimerHandle) ];
#endif
#endif // #if ( configSUPPORT_STATIC_ALLOCATION == 1 )

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
static xTimerHandle timerDeleteQueueHandle;
#endif

static xQueueHandle eventControlQueueHandle=NULL;
static const FreertosEventControl_t *eventControls[FREERTOS_EVENT_CONTROL_MAX_EVENT];
#define FREERTOS_EVENT_CONTROL_NUMBER_OF_REGISTER	((FREERTOS_EVENT_CONTROL_MAX_EVENT/sizeof(FreertosEventBits_t)) + ((FREERTOS_EVENT_CONTROL_MAX_EVENT%sizeof(FreertosEventBits_t)) == 0 ? 0:1))
static FreertosEventBits_t eventControlEventBits[FREERTOS_EVENT_CONTROL_NUMBER_OF_REGISTER * 2];

#define setRegister(eventNumber)	(eventNumber/sizeof(FreertosEventBits_t))
#define activeRegister(eventNumber)	(FREERTOS_EVENT_CONTROL_NUMBER_OF_REGISTER + setRegister(eventNumber))

#define FREERTOS_EVENT_CONTROL_FLAG_DEBUG_ENABLED	(1<<0)
static uint8_t eventControlFlags[FREERTOS_EVENT_CONTROL_MAX_EVENT];


#define eventControlEventBitsClear(eventNumber)	\
		eventControlEventBits[setRegister(eventNumber)] &= ~(1<<(eventNumber%sizeof(FreertosEventBits_t)));	\

#define eventControlEventBitsDeactivate(eventNumber)	\
		eventControlEventBits[activeRegister(eventNumber)] &= ~(1<<(eventNumber%sizeof(FreertosEventBits_t)));	\

#define eventControlEventBitsDeactivateAndClear(eventNumber)	\
	do {	\
		eventControlEventBitsClear(eventNumber);	\
		eventControlEventBitsDeactivate(eventNumber);	\
	}while(0)

#define eventControlEventBitsSet(eventNumber)	\
		eventControlEventBits[setRegister(eventNumber)] |= (1<<(eventNumber%sizeof(FreertosEventBits_t)));

#define eventControlEventBitsActive(eventNumber)	\
		eventControlEventBits[activeRegister(eventNumber)] |= (1<<(eventNumber%sizeof(FreertosEventBits_t)));

#define eventControlEventBitsSetAndActive(eventNumber)	\
	do {	\
		eventControlEventBitsSet(eventNumber);	\
		eventControlEventBitsActive(eventNumber);	\
	}while(0)

#define eventControlEventBitsClearAndActive(eventNumber)	\
	do {	\
		eventControlEventBitsClear(eventNumber);	\
		eventControlEventBitsActive(eventNumber);	\
	}while(0)

#define eventControlEventNumberIsSet(eventNumber)	\
		(eventControlEventBits[setRegister(eventNumber)] & (1<<(eventNumber%sizeof(FreertosEventBits_t))))

#define eventControlEventNumberIsActivated(eventNumber)	\
		(eventControlEventBits[activeRegister(eventNumber)] & (1<<(eventNumber%sizeof(FreertosEventBits_t))))

#define eventControlEventNumberIsSetAndActivated(eventNumber)	\
		(eventControlEventNumberIsSet(eventNumber) && eventControlEventNumberIsActivated(eventNumber))

#define eventControlGetEventName(eventNumber)	\
		(eventNumber < FREERTOS_EVENT_CONTROL_MAX_EVENT && eventControls[eventNumber] && eventControls[eventNumber]->name && eventControls[eventNumber]->name[0] ? eventControls[eventNumber]->name:"unknown")


/*******************************************************************************
*   Function Code
*******************************************************************************/

static void freertosQueueFullHander(void);

void freertosEventControlSetInactive(FreertosEventNumber_t eventToClear)
{
	if(eventToClear >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	taskENTER_CRITICAL();
	eventControlEventBitsDeactivateAndClear(eventToClear);
	taskEXIT_CRITICAL();
}

static inline void freertosEventControlSetReActive(FreertosEventNumber_t eventToSet)
{
	EventQueueItem_t eventQueueItem;
	portBASE_TYPE xReturn;

	while(!uxQueueSpacesAvailable(eventControlQueueHandle))
	{
		freertosQueueFullHander();
	}

	eventQueueItem.delayPeriodMs = 0;
	eventQueueItem.eventNumber = eventToSet;
	xReturn = xQueueSend(eventControlQueueHandle, (void *)&eventQueueItem, 0);
	evtDebugPrintln(	eventToSet,
						"Event control set active: event[%u]: %s, result: %s: %u",
						eventToSet, eventControlGetEventName(eventToSet),
						xReturn == pdPASS ? "pass":"fail", (uint32_t)xReturn);
}

void freertosEventControlSetActive(FreertosEventNumber_t eventToSet)
{


	if(eventToSet >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	taskENTER_CRITICAL();
	eventControlEventBitsSetAndActive(eventToSet);
	taskEXIT_CRITICAL();
	freertosEventControlSetReActive(eventToSet);
}

void freertosEventControlSetInactiveFromISR(FreertosEventNumber_t eventToClear)
{
	UBaseType_t uxSavedInterruptStatus;

	if(eventToClear >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}

	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	eventControlEventBitsDeactivateAndClear(eventToClear);
	taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
}

void freertosEventControlSetActiveFromISR(FreertosEventNumber_t eventToSet, portBASE_TYPE *pxHigherPriorityTaskWoken)
{
	UBaseType_t uxSavedInterruptStatus;
	EventQueueItem_t eventQueueItem;

	if(eventToSet >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}

	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	eventControlEventBitsSetAndActive(eventToSet);
	taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );

	eventQueueItem.delayPeriodMs = 0;
	eventQueueItem.eventNumber = eventToSet;
	xQueueSendFromISR(eventControlQueueHandle, (void *)&eventQueueItem, pxHigherPriorityTaskWoken);
}

bool freertosEventControlIsActivated(FreertosEventNumber_t eventNumber)
{
	if(eventNumber >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return false;
	}
	return eventControlEventNumberIsSetAndActivated(eventNumber);
}

static void vTimerSetEventCallback( xTimerHandle xTimer )
{
	FreertosEventNumber_t eventNumber;
	portBASE_TYPE xReturn;
#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
	const char *eventName;
#endif

	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT( xTimer );

	/* event is saved as the
	timer's ID */
	eventNumber = ( FreertosEventNumber_t ) ((uint32_t)pvTimerGetTimerID( xTimer ));

#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
	eventName = eventControlGetEventName(eventNumber);
#endif

	/* delete */
#ifdef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
	xReturn = xTimerDelete(xTimer, portMAX_DELAY);
	evtDebugPrintln(	eventNumber,
						"Event control delete timer: event[%u]: %s, result: %s: %u",
						eventNumber, eventName,
						xReturn == pdPASS ? "pass":"fail", (uint32_t)xReturn);

	// pdFAIL == FATAL
	configASSERT(xReturn == pdPASS);
#else
	xReturn = xQueueSend(timerDeleteQueueHandle, (void *)&xTimer, 0);
	evtDebugPrintln(	eventNumber,
						"Event control timer enqueue for delete: event[%u]: %s, result: %s: %u",
						eventNumber, eventName,
						xReturn == pdPASS ? "pass":"fail", (uint32_t)xReturn);

	// pdFAIL == FATAL
	configASSERT(xReturn == pdPASS);
#endif

	if(eventControlEventNumberIsActivated(eventNumber))
	{
		taskENTER_CRITICAL();
		eventControlEventBitsSet(eventNumber);
		taskEXIT_CRITICAL();
		freertosEventControlSetReActive( eventNumber );
	}
}

void freertosEventControlSetDelayMS(FreertosEventNumber_t eventToSet, uint32_t timerPeriodMs)
{
	EventQueueItem_t eventQueueItem;

	if(eventToSet >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}

	taskENTER_CRITICAL();
	eventControlEventBitsClearAndActive(eventToSet);
	taskEXIT_CRITICAL();
	eventQueueItem.eventNumber = eventToSet;
	eventQueueItem.delayPeriodMs = timerPeriodMs < 65535 ? timerPeriodMs:65535;
	xQueueSend(eventControlQueueHandle, (void *)&eventQueueItem, 0);
}

void freertosEventControlSetDelayMSFromISR(FreertosEventNumber_t eventToSet, uint32_t timerPeriodMs, portBASE_TYPE *pxHigherPriorityTaskWoken)
{
	EventQueueItem_t eventQueueItem;
	UBaseType_t uxSavedInterruptStatus;

	if(eventToSet >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}

	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	eventControlEventBitsClearAndActive(eventToSet);
	taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
	eventQueueItem.eventNumber = eventToSet;
	eventQueueItem.delayPeriodMs = timerPeriodMs < 65535 ? timerPeriodMs:65535;
	xQueueSendFromISR(eventControlQueueHandle, (void *)&eventQueueItem, pxHigherPriorityTaskWoken);
}

static void freertosQueueEventHander(EventQueueItem_t *item)
{
#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
	const char *eventName;
#endif

	if( item->eventNumber >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		evtDebugPrintln(item->eventNumber, "Invalid event number: %u", item->eventNumber);
		return;
	}

#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
	eventName = eventControlGetEventName(item->eventNumber);
#endif

	if(item->delayPeriodMs > 0 && eventControlEventNumberIsActivated(item->eventNumber))
	{
		xTimerHandle eventControlDelayTimer = xTimerCreate(	"event-control-delay",
															item->delayPeriodMs/portTICK_RATE_MS,
															pdFALSE,
															(void *)((uint32_t)item->eventNumber),
															vTimerSetEventCallback );
		evtDebugPrintln(	item->eventNumber,
							"Event control set delay: %u, event[%u]: %s, result: %s",
							 item->delayPeriodMs, item->eventNumber, eventName,
							 eventControlDelayTimer == NULL ? "fail":"ok");

		// NULL == FATAL
		configASSERT(eventControlDelayTimer);
		if(eventControlDelayTimer)
		{
			portBASE_TYPE xReturn;

			xReturn = xTimerStart(eventControlDelayTimer, FREERTOS_EVENT_CONTROL_BLOCKING_MAXIMUM);
			evtDebugPrintln(	item->eventNumber,
								"Event control start delay timer: event[%u]: %s, result: %s",
								item->eventNumber, eventName,
								xReturn == pdPASS ? "pass":"fail");
			// pdFAIL == FATAL
			configASSERT(xReturn == pdPASS);
		}
		return;
	}


	if(eventControls[item->eventNumber])
	{
		evtDebugPrintln(	item->eventNumber,
						"Event control handle: event[%u]: %s",
						item->eventNumber, eventName);
		eventControls[item->eventNumber]->callback(eventControls[item->eventNumber]->args);
	}

	// re-queue un-clear event
	if(eventControlEventNumberIsSetAndActivated(item->eventNumber))
	{
		portBASE_TYPE xReturn;
		EventQueueItem_t eventQueueItem;

		eventQueueItem.delayPeriodMs = 0;
		eventQueueItem.eventNumber = item->eventNumber;

		while(!uxQueueSpacesAvailable(eventControlQueueHandle))
		{
			freertosQueueFullHander();
		}
		xReturn = xQueueSend(eventControlQueueHandle, (void *)&eventQueueItem, FREERTOS_EVENT_CONTROL_BLOCKING_MAXIMUM);

		evtDebugPrintln(	item->eventNumber,
						"Event control re-queue: event[%u]: %s, result: %s",
						item->eventNumber, eventName,
						xReturn == pdPASS ? "pass":"fail");
	}
}

static void freertosQueueFullHander(void)
{
	EventQueueItem_t eventQueueItem;
	extern void *pxCurrentTCB;

	// not current task == FATAL
	taskENTER_CRITICAL();
	configASSERT(eventControlTaskHandle == pxCurrentTCB);
	taskEXIT_CRITICAL();

	if (!xQueueReceive(eventControlQueueHandle, &eventQueueItem, portMAX_DELAY))
	{
		return;
	}
	freertosQueueEventHander(&eventQueueItem);
}

static void freertosEventControlTask(void *param)
{
	extern void *pxCurrentTCB;

	configASSERT(eventControlTaskHandle == pxCurrentTCB);
	while(1)
	{
		EventQueueItem_t eventQueueItem;

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
		xTimerHandle eventControlTimer;
		if (xQueueReceive(timerDeleteQueueHandle, &eventControlTimer, 0))
		{
			xTimerDelete(capsenseTimer, portMAX_DELAY);
		}
#endif

		if (!xQueueReceive(eventControlQueueHandle, &eventQueueItem, portMAX_DELAY))
		{
			continue;
		}

		freertosQueueEventHander(&eventQueueItem);
	}
}

static inline void freertosEventControlEventBitsInit(void)
{
	uint8_t i;

	for(i = 0; i < FREERTOS_EVENT_CONTROL_MAX_EVENT; i++)
	{
		eventControls[i] = NULL;
		eventControlFlags[i] = 0;
	}
	for(i = 0; i < sizeof(eventControlEventBits)/sizeof(FreertosEventBits_t); i++)
	{
		eventControlEventBits[i] = 0;
	}
	debugPrintln(	"Event control: maximum of event: %u, register count: %u",
					FREERTOS_EVENT_CONTROL_MAX_EVENT,
					sizeof(eventControlEventBits)/sizeof(FreertosEventBits_t));
	configASSERT((FREERTOS_EVENT_CONTROL_NUMBER_OF_REGISTER * 2) == sizeof(eventControlEventBits)/sizeof(FreertosEventBits_t));
}

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
void freertosEventControlInitStatic(UBaseType_t uxPriority)
{
	freertosEventControlEventBitsInit();

	/* Create a queue capable of containing 10 uint64_t values. */
	eventControlQueueHandle = xQueueCreateStatic(
									FREERTOS_EVENT_CONTROL_QUEUE_SIZE,
									sizeof(EventQueueItem_t),
									eventControlQueueStorageArea,
									&eventControlQueue );

	/* pxQueueBuffer was not NULL so xQueue should not be NULL. */
	configASSERT( eventControlQueueHandle );

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
	/* Create a queue capable of containing 10 uint64_t values. */
	timerDeleteQueueHandle = xQueueCreateStatic(
									FREERTOS_EVENT_CONTROL_TIMER_DELETE_QUEUE_SIZE,
									sizeof(EventQueueItem_t),
									timerDeleteQueueStorageArea,
									&timerDeleteQueue );

	/* pxQueueBuffer was not NULL so xQueue should not be NULL. */
	configASSERT( eventControlQueueHandle );
#endif

	eventControlTaskHandle = xTaskCreateStatic(
					freertosEventControlTask,
					"event control task static",
					FREERTOS_EVENT_CONTROL_TASK_STATIC_STACK_SIZE,
					NULL,
					uxPriority,
					eventControlTaskStack,
					&eventControlTaskBuffer);
	// NULL == FATAL
	configASSERT(eventControlTaskHandle);
}
#endif

#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
void freertosEventControlInitDynamic(const configSTACK_DEPTH_TYPE usStackDepth, UBaseType_t uxPriority)
{
	portBASE_TYPE xReturn;

	freertosEventControlEventBitsInit();

	eventControlQueueHandle = xQueueCreate(FREERTOS_EVENT_CONTROL_QUEUE_SIZE, sizeof(EventQueueItem_t));
	configASSERT( eventControlQueueHandle );

#ifndef FREERTOS_EVENT_CONTROL_TIMER_SELF_DELETE
	timerDeleteQueueHandle = xQueueCreate(FREERTOS_EVENT_CONTROL_TIMER_DELETE_QUEUE_SIZE, sizeof(xTimerHandle));
	configASSERT( timerDeleteQueueHandle );
#endif

	xReturn = xTaskCreate(
					freertosEventControlTask,
					"event control task dynamic",
					usStackDepth,
					NULL,
					uxPriority,
					&eventControlTaskHandle);
	// pdFAIL == FATAL
	configASSERT(xReturn == pdPASS);
}
#endif

#ifdef FREERTOS_EVENT_CONTROL_DEBUG_ENABLED
void freertosEventControlDebugEnable(FreertosEventNumber_t eventNumber)
{
	if(eventNumber >= FREERTOS_EVENT_CONTROL_MAX_EVENT)
	{
		return;
	}
	eventControlFlags[eventNumber] |= FREERTOS_EVENT_CONTROL_FLAG_DEBUG_ENABLED;
}
#endif

uint16_t freertosEventControlRegister(const FreertosEventControl_t *eventControl)
{
	uint8_t i;

	for(i = 0; i < FREERTOS_EVENT_CONTROL_MAX_EVENT; i++)
	{
		if(!eventControls[i])
		{
			eventControls[i] = eventControl;
			break;
		}
	}
	return i;
}

/****************************End of File***************************************/
