#include "FreeRTOS.h"
#include <stdio.h>
#include <task.h>

void vApplicationGetPassiveIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                           StackType_t **ppxIdleTaskStackBuffer,
                                           uint32_t *pulIdleTaskStackSize,
                                           BaseType_t xCoreID )
{
    static StaticTask_t xPassiveIdleTaskTCB;
    static StackType_t uxPassiveIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Avoid compiler warning for unused parameter */
    ( void ) xCoreID;

    *ppxIdleTaskTCBBuffer = &xPassiveIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxPassiveIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationPassiveIdleHook( void )
{
    __asm volatile ("wfe");
}

void vApplicationMallocFailedHook( void )
{
    while(1);
}