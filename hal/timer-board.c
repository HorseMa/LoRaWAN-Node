/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: MCU RTC timer and low power modes management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <math.h>
#include "board.h"
#include "timer-board.h"

/*!
 * Hardware Time base in us
 */
#define HW_TIMER_TIME_BASE                              100 //us 

/*!
 * Hardware Timer tick counter
 */
volatile TimerTime_t TimerTickCounter = 1;

/*!
 * Saved value of the Tick counter at the start of the next event
 */
static TimerTime_t TimerTickCounterContext = 0;

/*!
 * Value trigging the IRQ
 */
volatile TimerTime_t TimeoutCntValue = 0;

/*!
 * Increment the Hardware Timer tick counter
 */
void TimerIncrementTickCounter( void );

/*!
 * Counter used for the Delay operations
 */
volatile uint32_t TimerDelayCounter = 0;

/*!
 * Return the value of the counter used for a Delay
 */
uint32_t TimerHwGetDelayValue( void );

/*!
 * Increment the value of TimerDelayCounter
 */
void TimerIncrementDelayCounter( void );


void TimerHwInit( void )
{

    /* Select HSE as system clock source */
    CLK_SYSCLKSourceSwitchCmd(ENABLE);
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);
    /*High speed external clock prescaler: 1*/
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_2);

    while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSE)
    {}
    
    /* TIM2 init */
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2,ENABLE);


    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_Prescaler_1,TIM2_CounterMode_Up,16000000 / 1 / 10000);

    TIM2_SetAutoreload(16000000 / 1 / 10000);
    TIM2_ITConfig(TIM2_IT_Update,ENABLE);
    /* TIM2 counter enable */
    TIM2_Cmd(ENABLE);
    TimeoutCntValue = 0;
        
    /* TIM3 clock enable */ 
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM3,ENABLE);


    TIM3_DeInit();
    TIM3_TimeBaseInit(TIM3_Prescaler_1,TIM3_CounterMode_Up,16000000 / 1 / 10000);

    TIM3_SetAutoreload(16000000 / 1 / 10000);
    TIM3_ITConfig(TIM3_IT_Update,ENABLE);
    /* TIM2 counter enable */
    TIM3_Cmd(ENABLE);
}

void TimerHwDeInit( void )
{
    /* Deinitialize the timer */
    TIM2_DeInit();
}

uint32_t TimerHwGetMinimumTimeout( void )
{
    return( ceil( 2 * HW_TIMER_TIME_BASE ) );
}

void TimerHwStart( uint32_t val )
{
    TimerTickCounterContext = TimerHwGetTimerValue( );

    if( val <= HW_TIMER_TIME_BASE + 1 )
    {
        TimeoutCntValue = TimerTickCounterContext + 1;
    }
    else
    {
        TimeoutCntValue = TimerTickCounterContext + ( ( val - 1 ) / HW_TIMER_TIME_BASE );
    }
}

void TimerHwStop( void )
{
    TIM2_ITConfig(TIM2_IT_CC1,DISABLE);
    TIM2_Cmd(DISABLE);
}

void TimerHwDelayMs( uint32_t delay )
{
    uint32_t delayValue = 0;

    delayValue = delay;

    TimerDelayCounter = 0;

    TIM3_ITConfig(TIM3_IT_Update,ENABLE);
    TIM3_Cmd(ENABLE);

    while( TimerHwGetDelayValue( ) < delayValue )
    {
    }

    TIM3_ITConfig(TIM3_IT_Update,DISABLE);
    TIM3_Cmd(DISABLE);
}

TimerTime_t TimerHwGetElapsedTime( void )
{
     return( ( ( TimerHwGetTimerValue( ) - TimerTickCounterContext ) + 1 )  * HW_TIMER_TIME_BASE );
}

TimerTime_t TimerHwGetTimerValue( void )
{
    TimerTime_t val = 0;

    __disable_irq( );

    val = TimerTickCounter;

    __enable_irq( );

    return( val );
}

TimerTime_t TimerHwGetTime( void )
{

    return TimerHwGetTimerValue( ) * HW_TIMER_TIME_BASE;
}

uint32_t TimerHwGetDelayValue( void )
{
    uint32_t val = 0;

    __disable_irq( );

    val = TimerDelayCounter;

    __enable_irq( );

    return( val );
}

void TimerIncrementTickCounter( void )
{
    __disable_irq( );

    TimerTickCounter++;

    __enable_irq( );
}

void TimerIncrementDelayCounter( void )
{
    __disable_irq( );

    TimerDelayCounter++;

    __enable_irq( );
}

/*!
 * Timer IRQ handler
 */
void TIM2_IRQHandler( void )
{
    if( TIM2_GetFlagStatus(TIM2_FLAG_Update) != RESET )
    {
        TIM2_ClearFlag(TIM2_FLAG_Update);
        TimerIncrementTickCounter( );
        
    
        if( TimerTickCounter == TimeoutCntValue )
        {
            TimerIrqHandler( );
        }
    }
}

/*!
 * Timer IRQ handler
 */
void TIM3_IRQHandler( void )
{
    if( TIM3_GetFlagStatus(TIM3_FLAG_Update) != RESET )
    {
        TIM3_ClearFlag(TIM3_FLAG_Update);
        TimerIncrementDelayCounter( );
        
    }
}

void TimerHwEnterLowPowerStopMode( void )
{
#ifndef USE_DEBUGGER
    //__WFI( );
    //__wait_for_interrupt();
#endif
}
