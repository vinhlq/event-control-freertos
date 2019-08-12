/*****************************************************************************
* File Name: event-control.h
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

#if !defined(FREERTOS_EVENT_CONTROL_H)
#define FREERTOS_EVENT_CONTROL_H
    
/*******************************************************************************
* Included headers
*******************************************************************************/
#include <stdint.h>


/*******************************************************************************
* User defined Macros
*******************************************************************************/

#ifndef FREERTOS_EVENT_CONTROL_MAX_EVENT
#define FREERTOS_EVENT_CONTROL_MAX_EVENT 17
#endif

/*******************************************************************************
* Data Type Definitions
*******************************************************************************/

typedef uint32_t FreertosEventBits_t;

typedef uint16_t FreertosEventNumber_t;

typedef void (*freertosEventControlCallback_t)(void *args);

typedef struct
{
	const char *name;
	freertosEventControlCallback_t callback;
	void * const args;
}FreertosEventControl_t;

/*******************************************************************************
* Structure Definitions
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
void freertosEventControlSetInactive(FreertosEventNumber_t eventToClear);

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
void freertosEventControlSetActive(FreertosEventNumber_t eventToSet);

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
void freertosEventControlSetInactiveFromISR(FreertosEventNumber_t eventToClear);

/** @brief freertosEventControlSetActiveFromISR
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param eventToSet.
 */
void freertosEventControlSetActiveFromISR(FreertosEventNumber_t eventToSet, portBASE_TYPE *pxHigherPriorityTaskWoken);

/** @brief freertosEventControlSetDelayMS
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param eventToSet.
 */
void freertosEventControlSetDelayMS(FreertosEventNumber_t eventToSet, uint32_t timerPeriodMs);

/** @brief freertosEventControlSetDelayMSFromISR
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param eventToSet.
 */
void freertosEventControlSetDelayMSFromISR(FreertosEventNumber_t eventToSet, uint32_t timerPeriodMs, portBASE_TYPE *pxHigherPriorityTaskWoken);

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
bool freertosEventControlIsActivated(FreertosEventNumber_t eventNumber);

/** @brief freertosEventControlDebugEnable
 *
 * This enable event debug print.
 *
 * @param eventNumber.
 */
void freertosEventControlDebugEnable(FreertosEventNumber_t eventNumber);

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
void freertosEventControlInitDynamic(const configSTACK_DEPTH_TYPE usStackDepth, UBaseType_t uxPriority);

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
void freertosEventControlInitStatic(UBaseType_t uxPriority);

#if ( configSUPPORT_STATIC_ALLOCATION == 1 )
#define freertosEventControlInit freertosEventControlInitStatic
#elif ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
#define freertosEventControlInit freertosEventControlInitDynamic
#endif

/** @brief eventControlSetInactive
 *
 * This callback is called when a eventControlSetInactive.
 *
 * @param uxBitsToClear.
 */
uint16_t freertosEventControlRegister(const FreertosEventControl_t *eventControl);

#endif

/****************************End of File***************************************/
