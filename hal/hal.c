/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lmic.h"
#include "hw.h"

// -----------------------------------------------------------------------------
// I/O

#ifdef CFG_sx1276mb1_board

#define NSS_PORT           1 // NSS: PB6, sx1276
#define NSS_PIN            6  // sx1276: PB6

#define TX_PORT            2 // TX:  PC1
#define TX_PIN             1

#define RST_PORT           0 // RST: PA0
#define RST_PIN            0

#define DIO0_PORT          0 // DIO0: PA10, sx1276   (line 1 irq handler)
#define DIO0_PIN           10
#define DIO1_PORT          1 // DIO1: PB3, sx1276  (line 10-15 irq handler)
#define DIO1_PIN           3
#define DIO2_PORT          1 // DIO2: PB5, sx1276  (line 10-15 irq handler)
#define DIO2_PIN           5

static const u1_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN  };
static const u1_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

#elif CFG_wimod_board

// output lines
#define NSS_PORT           1 // NSS: PB0, sx1272
#define NSS_PIN            0

#define TX_PORT            0 // TX:  PA4
#define TX_PIN             4
#define RX_PORT            2 // RX:  PC13
#define RX_PIN            13
#define RST_PORT           0 // RST: PA2
#define RST_PIN            2

// input lines
#define DIO0_PORT          1 // DIO0: PB1   (line 1 irq handler)
#define DIO0_PIN           1
#define DIO1_PORT          1 // DIO1: PB10  (line 10-15 irq handler)
#define DIO1_PIN          10
#define DIO2_PORT          1 // DIO2: PB11  (line 10-15 irq handler)
#define DIO2_PIN          11

static const u1_t outputpins[] = { NSS_PORT, NSS_PIN, TX_PORT, TX_PIN, RX_PORT, RX_PIN };
static const u1_t inputpins[]  = { DIO0_PORT, DIO0_PIN, DIO1_PORT, DIO1_PIN, DIO2_PORT, DIO2_PIN };

#else
#error Missing CFG_sx1276mb1_board/CFG_wimod_board!
#endif

// HAL state
static struct {
    int irqlevel;
    u4_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);
    GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_In_PU_IT);    // DIO0
    EXTI_SetPinSensitivity(EXTI_Pin_6,EXTI_Trigger_Falling);
    GPIO_Init(GPIOC, GPIO_Pin_5, GPIO_Mode_In_PU_IT);    // DIO1
    EXTI_SetPinSensitivity(EXTI_Pin_5,EXTI_Trigger_Falling);
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_PU_IT);    // DIO2
    EXTI_SetPinSensitivity(EXTI_Pin_4,EXTI_Trigger_Falling);
    GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_High_Fast);    // UART_TX
    GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_FL_No_IT);    // UART_RX
    GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_In_PU_IT);    // DIO3
    EXTI_SetPinSensitivity(EXTI_Pin_1,EXTI_Trigger_Falling);
    GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_In_PU_IT);    // DIO4
    EXTI_SetPinSensitivity(EXTI_Pin_0,EXTI_Trigger_Falling);
    //GPIO_Init(GPIOD, GPIO_Pin_4, GPIO_Mode_In_PU_IT);    // DIO5
    //EXTI_SetPinSensitivity(EXTI_Pin_4,EXTI_Trigger_Falling);
    
    GPIO_Init(GPIOB, GPIO_Pin_3, GPIO_Mode_In_FL_No_IT);    // PB3
    GPIO_Init(GPIOB, GPIO_Pin_2, GPIO_Mode_Out_PP_High_Fast);    // LED_RX
    GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_Out_PP_High_Fast);    // LED_TX
    GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT);    // CH1
    GPIO_Init(GPIOD, GPIO_Pin_3, GPIO_Mode_In_FL_No_IT);    // CH2
    GPIO_Init(GPIOD, GPIO_Pin_2, GPIO_Mode_In_FL_No_IT);    // CH3
    GPIO_Init(GPIOD, GPIO_Pin_1, GPIO_Mode_In_FL_No_IT);    // CH4
    GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT);    // CH5
    GPIO_Init(GPIOA, GPIO_Pin_5, GPIO_Mode_In_FL_No_IT);    // PA5
    GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Fast);    // SX1278_RST

    // clock enable for GPIO ports A,B,C

    // configure output lines and set to low state

    // configure input lines and register IRQ
}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val) {
    //GPIO_WriteBit(GPIOA, 1, val);
#if 0
    ASSERT(val == 1 || val == 0);
#ifndef CFG_sx1276mb1_board
    hw_set_pin(GPIOx(RX_PORT), RX_PIN, ~val);
#endif
    hw_set_pin(GPIOx(TX_PORT), TX_PIN, val);
#endif
}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val) {
    GPIO_WriteBit(GPIOB, GPIO_Pin_4, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, val);
}

extern void radio_irq_handler(u1_t dio);

// generic EXTI IRQ handler for all channels
void EXTI_IRQHandler (u1_t irq) {
    
    //radio_irq_handler(irq);
    //EXTI_ClearITPendingBit(EXTI_IT_Pin0);
    //EXTI_ClearITPendingBit(EXTI_IT_Pin1);
    //EXTI_ClearITPendingBit(EXTI_IT_Pin2);
    //EXTI_ClearITPendingBit(EXTI_IT_Pin3);
    //EXTI_ClearITPendingBit(EXTI_IT_Pin4);
    return;
#if 0
    // DIO 0
    if((EXTI->PR & (1<<DIO0_PIN)) != 0) { // pending
        EXTI->PR = (1<<DIO0_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(0);
    }
    // DIO 1
    if((EXTI->PR & (1<<DIO1_PIN)) != 0) { // pending
        EXTI->PR = (1<<DIO1_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(1);
    }
    // DIO 2
    if((EXTI->PR & (1<<DIO2_PIN)) != 0) { // pending
        EXTI->PR = (1<<DIO2_PIN); // clear irq
        // invoke radio handler (on IRQ!)
        radio_irq_handler(2);
    }
       
#ifdef CFG_EXTI_IRQ_HANDLER
    // invoke user-defined interrupt handler
    {
        extern void CFG_EXTI_IRQ_HANDLER(void);
        CFG_EXTI_IRQ_HANDLER();
    }
#endif // CFG_EXTI_IRQ_HANDLER
#endif
}

#if CFG_lmic_clib
#if 0
void EXTI0_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI1_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI2_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI3_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI4_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI9_5_IRQHandler () {
    EXTI_IRQHandler();
}

void EXTI15_10_IRQHandler () {
    EXTI_IRQHandler();
}
#endif
#endif // CFG_lmic_clib

// -----------------------------------------------------------------------------
// SPI

// for sx1272 and 1276

#define SCK_PORT   0 // SCK:  PB5
#define SCK_PIN    5
#define MISO_PORT  0 // MISO: PB7
#define MISO_PIN   7
#define MOSI_PORT  0 // MOSI: PB6
#define MOSI_PIN   6

#define GPIO_AF_SPI1        0x05

static void hal_spi_init () {
    CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
    GPIO_Init(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Fast); // SPI_CS output high
    GPIO_Init(GPIOB, GPIO_Pin_5, GPIO_Mode_Out_PP_High_Fast);  // SPI_SCLK output
    GPIO_Init(GPIOB, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast); // SPI_MOSI output
    GPIO_Init(GPIOB, GPIO_Pin_7, GPIO_Mode_In_FL_No_IT); // SPI_MISO input
    SPI_Init(SPI1, SPI_FirstBit_MSB,
              SPI_BaudRatePrescaler_8,
              SPI_Mode_Master, SPI_CPOL_Low,
              SPI_CPHA_1Edge, SPI_Direction_2Lines_FullDuplex,
              SPI_NSS_Soft, 7);
        
    SPI_Cmd(SPI1,ENABLE);
    
#if 0
    // enable clock for SPI interface 1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // use alternate function SPI1 (SCK, MISO, MOSI)
    hw_cfg_pin(GPIOx(SCK_PORT),  SCK_PIN,  GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
    hw_cfg_pin(GPIOx(MISO_PORT), MISO_PIN, GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
    hw_cfg_pin(GPIOx(MOSI_PORT), MOSI_PIN, GPIOCFG_MODE_ALT | GPIOCFG_OSPEED_40MHz | GPIO_AF_SPI1 | GPIOCFG_OTYPE_PUPD | GPIOCFG_PUPD_PDN);
    
    // configure and activate the SPI (master, internal slave select, software slave mgmt)
    // (use default mode: 8-bit, 2-wire, no crc, MSBF, PCLK/2, CPOL0, CPHA0)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_SPE;
#endif
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    /* Loop while DR register in not emplty */
    while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);

    /* Send byte through the SPI peripheral */
    SPI_SendData(SPI1, out);

    /* Wait to receive a byte */
    while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET);

    /* Return the byte read from the SPI bus */
    return SPI_ReceiveData(SPI1);
}

#ifdef CFG_lmic_clib

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
    /* Select HSE as system clock source */
    CLK_SYSCLKSourceSwitchCmd(ENABLE);
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);
    /*High speed external clock prescaler: 1*/
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_2);

    while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSE)
    {}
    
    /* TIM2 init */
#if 0
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM2,ENABLE);


    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_Prescaler_8,TIM2_CounterMode_Up,61);

    TIM2_SetAutoreload(61);
    TIM2_ITConfig(TIM2_IT_Update,ENABLE);
    /* TIM2 counter enable */
    TIM2_Cmd(ENABLE);
#endif
#if 0
#ifndef CFG_clock_HSE
    PWR->CR |= PWR_CR_DBP; // disable write protect
    RCC->CSR |= RCC_CSR_LSEON; // switch on low-speed oscillator @32.768kHz
    while( (RCC->CSR & RCC_CSR_LSERDY) == 0 ); // wait for it...
#endif
    
    RCC->APB2ENR   |= RCC_APB2ENR_TIM9EN;     // enable clock to TIM9 peripheral 
    RCC->APB2LPENR |= RCC_APB2LPENR_TIM9LPEN; // enable clock to TIM9 peripheral also in low power mode
    RCC->APB2RSTR  |= RCC_APB2RSTR_TIM9RST;   // reset TIM9 interface
    RCC->APB2RSTR  &= ~RCC_APB2RSTR_TIM9RST;  // reset TIM9 interface

#if CFG_clock_HSE
    TIM9->PSC  = (640 - 1); // HSE_CLOCK_HWTIMER_PSC-1);  XXX: define HSE_CLOCK_HWTIMER_PSC somewhere
#else
    TIM9->SMCR = TIM_SMCR_ECE; // external clock enable (source clock mode 2) with no prescaler and no filter
#endif
    
    NVIC->IP[TIM9_IRQn] = 0x70; // interrupt priority
    NVIC->ISER[TIM9_IRQn>>5] = 1<<(TIM9_IRQn&0x1F);  // set enable IRQ

    // enable update (overflow) interrupt
    TIM9->DIER |= TIM_DIER_UIE;
    
    // Enable timer counting
    TIM9->CR1 = TIM_CR1_CEN;
#endif
}

u4_t hal_ticks () {
#if 1
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    //u2_t cnt = TIM9->CNT;
    u2_t cnt = TIM2_GetCounter();
    //if( (TIM9->SR & TIM_SR_UIF) ) {
    if( TIM2_GetFlagStatus(TIM2_FLAG_Update) == SET) {
        // Overflow before we read CNT?
        // Include overflow in evaluation but
        // leave update of state to ISR once interrupts enabled again
        //cnt = TIM9->CNT;
        cnt = TIM2_GetCounter();
        t++;
    }
    hal_enableIRQs();
    return (t<<16)|cnt;
#endif
    //return 0;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    u2_t dt;
#if 0
    TIM2_ClearITPendingBit(TIM2_IT_CC2);
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        //TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        TIM2_ITConfig(TIM2_IT_CC2,DISABLE);
        return 1;
    } else { // rewind timer (fully or to exact time))
        //TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
        TIM2_SetCompare2(TIM2_GetCounter() + dt);
        //TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
        TIM2_ITConfig(TIM2_IT_CC2,ENABLE);
        //TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        TIM2_CCxCmd(TIM2_Channel_2,ENABLE);
        return 0;
    }
#endif
#if 0
    u2_t dt;
    TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
        TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
        TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
        TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        return 0;
    }
#endif
}
  
void TIM9_IRQHandler () {
#if 0
    //if(TIM9->SR & TIM_SR_UIF) { // overflow
  if(TIM2_GetFlagStatus(TIM2_FLAG_Update) == SET){
        HAL.ticks++;
    }
    //if((TIM9->SR & TIM_SR_CC2IF) && (TIM9->DIER & TIM_DIER_CC2IE)) { // expired
  if((TIM2_GetFlagStatus(TIM2_FLAG_CC2) == SET)){// && (TIM2_GetITStatus(TIM2_IT_CC2) == SET)){
        // do nothing, only wake up cpu
    }
    //TIM9->SR = 0; // clear IRQ flags
    TIM2_ClearFlag(TIM2_FLAG_Update);
    TIM2_ClearFlag(TIM2_FLAG_CC2);
    //TIM2_ClearITPendingBit(TIM2_IT_Update);
    //TIM2_ClearITPendingBit(TIM2_IT_CC2);
#endif
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
    __disable_interrupt();
    HAL.irqlevel++;
}

void hal_enableIRQs () {
    if(--HAL.irqlevel == 0) {
        __enable_interrupt();
    }
}

void hal_sleep () {
    // suspend execution until IRQ, regardless of the CPSR I-bit
    __wait_for_interrupt;
}

// -----------------------------------------------------------------------------

void hal_init () {
    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    hal_enableIRQs();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

#endif // CFG_lmic_clib
