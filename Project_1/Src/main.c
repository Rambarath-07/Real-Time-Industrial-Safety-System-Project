#include "stm32f405xx.h"
#include <stdint.h>
#include <stdio.h>
#include "lcd.h"
#include "cmn.h"

#define GPIO_PIN_2   (1 << 2)   // DHT11 on PA2
#define PC9_PIN      (1 << 9)   // Buzzer
#define PC10_PIN     (1 << 10)  // Overheat Alert LED

uint32_t SystemCoreClock = 72000000;

typedef struct {
    uint8_t Temperature;
    uint8_t Humidity;
} DHT11_Data;

DHT11_Data dht;

volatile uint32_t vibration_count = 0;
volatile uint8_t beep_5_done = 0;
volatile uint8_t beep_10_done = 0;
volatile uint8_t temperature_alert = 0;
volatile uint8_t humidity_alert = 0;

// ---------------- DWT Delay ----------------
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// ---------------- DHT11 ----------------
void DHT11_Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t pos = __builtin_ctz(GPIO_Pin);
    GPIOx->MODER &= ~(3 << (pos * 2));
    GPIOx->MODER |= (1 << (pos * 2));
    GPIOx->PUPDR &= ~(3 << (pos * 2));
}

void DHT11_Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t pos = __builtin_ctz(GPIO_Pin);
    GPIOx->MODER &= ~(3 << (pos * 2));
    GPIOx->PUPDR &= ~(3 << (pos * 2));
    GPIOx->PUPDR |= (1 << (pos * 2)); // Pull-up
}

void DHT11_Start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    DHT11_Set_Pin_Output(GPIOx, GPIO_Pin);
    GPIOx->ODR &= ~GPIO_Pin;
    delay_ms(20);
    GPIOx->ODR |= GPIO_Pin;
    delay_us(30);
    DHT11_Set_Pin_Input(GPIOx, GPIO_Pin);
}

uint8_t DHT11_Check_Response(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    delay_us(40);
    if (!(GPIOx->IDR & GPIO_Pin)) {
        delay_us(80);
        if (GPIOx->IDR & GPIO_Pin) {
            delay_us(80);
            return 1;
        }
    }
    return 0;
}

uint8_t DHT11_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        while (!(GPIOx->IDR & GPIO_Pin));
        delay_us(40);
        if (GPIOx->IDR & GPIO_Pin)
            data |= (1 << (7 - i));
        while (GPIOx->IDR & GPIO_Pin);
    }
    return data;
}

uint8_t DHT11_GetData(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, DHT11_Data *data) {
    uint8_t Rh1, Rh2, Temp1, Temp2, checksum;
    DHT11_Start(GPIOx, GPIO_Pin);
    if (DHT11_Check_Response(GPIOx, GPIO_Pin)) {
        Rh1 = DHT11_Read(GPIOx, GPIO_Pin);
        Rh2 = DHT11_Read(GPIOx, GPIO_Pin);
        Temp1 = DHT11_Read(GPIOx, GPIO_Pin);
        Temp2 = DHT11_Read(GPIOx, GPIO_Pin);
        checksum = DHT11_Read(GPIOx, GPIO_Pin);
        if (checksum == (Rh1 + Rh2 + Temp1 + Temp2)) {
            data->Humidity = Rh1;
            data->Temperature = Temp1;
            return 1;
        }
    }
    return 0;
}

// ---------------- Clock ----------------
void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->PLLCFGR = (8 << 0) | (72 << 6) | RCC_PLLCFGR_PLLSRC_HSI;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

// ---------------- GPIO ----------------
void GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // PIR Sensor PA9
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->PUPDR &= ~(3 << (9 * 2));
    GPIOA->PUPDR |= (2 << (9 * 2));

    // Motion LED PA12
    GPIOA->MODER &= ~(3 << (12 * 2));
    GPIOA->MODER |= (1 << (12 * 2));
    GPIOA->ODR |= (1 << 12);

    // DHT11 PA2 (already managed dynamically)
    GPIOA->ODR |= GPIO_PIN_2;

    // Vibration PA3
    GPIOA->MODER &= ~(3 << (3 * 2));
    GPIOA->PUPDR &= ~(3 << (3 * 2));
    GPIOA->PUPDR |= (2 << (3 * 2));

    // Buzzer PC9
    GPIOC->MODER &= ~(3 << (9 * 2));
    GPIOC->MODER |= (1 << (9 * 2));

    // Alert LED PC10
    GPIOC->MODER &= ~(3 << (10 * 2));
    GPIOC->MODER |= (1 << (10 * 2));
}

// ---------------- EXTI for Vibration ----------------
void EXTI3_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~(0xF << 12);
    SYSCFG->EXTICR[0] |= (0 << 12);
    EXTI->IMR |= (1 << 3);
    EXTI->RTSR |= (1 << 3);
    NVIC_EnableIRQ(EXTI3_IRQn);
}

void EXTI3_IRQHandler(void) {
    if (EXTI->PR & (1 << 3)) {
        vibration_count++;
        delay_ms(50);

        lprint(0xC0, "V:     ");
        lprint(0xC0, "V:");
        lprint_num(0xC3, vibration_count);

        if (vibration_count >= 5 && !beep_5_done) {
            beep_5_done = 1;
            GPIOC->BSRR = GPIO_BSRR_BS9;
            delay_ms(500);
            GPIOC->BSRR = GPIO_BSRR_BR9;
        }

        if (vibration_count >= 10 && !beep_10_done) {
            beep_10_done = 1;
            GPIOC->BSRR = GPIO_BSRR_BS9;
            delay_ms(1000);
            GPIOC->BSRR = GPIO_BSRR_BR9;
        }

        if (vibration_count >= 15) {
            vibration_count = 0;
            beep_5_done = 0;
            beep_10_done = 0;

            lprint(0xC0, "V:     ");
            lprint(0xC0, "V:");
            lprint_num(0xC3, vibration_count);
        }

        EXTI->PR = (1 << 3);
    }
}

// ---------------- Timer3 for PIR ----------------
void TIM3_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 7200 - 1;
    TIM3->ARR = 5000 - 1;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void) {
    static uint8_t motion_timer = 0;
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;

        if (GPIOA->IDR & (1 << 9)) {
            motion_timer = 4;
            GPIOA->ODR &= ~(1 << 12);
            lprint(0xC6, "MOTION:YES");
        } else {
            if (motion_timer > 0) motion_timer--;
            else {
                GPIOA->ODR |= (1 << 12);
                lprint(0xC6, "MOTION: NO ");
            }
        }
    }
}

// ---------------- Timer4 for DHT11 ----------------
void TIM4_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 7200 - 1;
    TIM4->ARR = 20000 - 1;
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;

        if (DHT11_GetData(GPIOA, GPIO_PIN_2, &dht)) {
            temperature_alert = dht.Temperature;
            humidity_alert = dht.Humidity;

            lprint(0x80, "T:");
            lprint_num(0x83, temperature_alert);
            lprint(0x86, "H:");
            lprint_num(0x89, humidity_alert);

            if (temperature_alert > 24 && humidity_alert > 80) {
                GPIOC->ODR |= PC10_PIN;
                delay_ms(5000);
                GPIOC->ODR &= ~PC10_PIN;
            }
        }
    }
}

// ---------------- Main ----------------
int main(void) {
    SystemClock_Config();
    DWT_Init();
    GPIO_Config();
    LcdInit();
    EXTI3_Config();
    TIM3_Config();
    TIM4_Config();

    while (1);
}
