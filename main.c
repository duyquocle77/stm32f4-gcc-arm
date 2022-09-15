#include <stdint.h>

uint32_t sys_cnt = 0;

typedef enum {
	OFF,
	ON
} Led_state_t;

void SysTick_Init();
void SysTick_Handler();
void Delay_ms(uint32_t time_ms);
void SystemInit();
void LED_Init();
void LED_Control(Led_state_t state);

int main() {
	SysTick_Init();
	SystemInit();
	LED_Init();
	
	while (1) {
		LED_Control(ON);
		Delay_ms(1000);
		LED_Control(OFF);
		Delay_ms(1000);
	}

	return 0;
}

void SysTick_Init() {
	volatile uint32_t* const SYST_CSR = (uint32_t*)(0xe000e010 + 0x00);
	volatile uint32_t* const SYST_RVR = (uint32_t*)(0xe000e010 + 0x04);

	*SYST_RVR = 16000 - 1;
	*SYST_CSR |= (1 << 0) | (1 << 1) | (1 << 2);
}

void SysTick_Handler() {
	sys_cnt++;
}

void Delay_ms(uint32_t time_ms) {
	sys_cnt = 0;
	while (sys_cnt < time_ms);
}

void SystemInit() {
	volatile uint32_t* const RCC_CR        = (uint32_t*)(0x40023800 + 0x00);
	volatile uint32_t* const RCC_PLLCFGR   = (uint32_t*)(0x40023800 + 0x04);
	volatile uint32_t *const RCC_CFGR	   = (uint32_t *)(0x40023800 + 0x08);
	*RCC_CR |= (1 << 16) | (1 << 24);
	while(((*RCC_CR >> 17) & 1) == 0);
	while(((*RCC_CR >> 25) & 1) == 0);
	*RCC_PLLCFGR |= (1 << 22);
	*RCC_PLLCFGR &= ~(0b111111 << 0);
	*RCC_PLLCFGR |= (8 << 0);
	*RCC_PLLCFGR &= ~(0b111111111 << 6);
	*RCC_PLLCFGR |= (200 << 6);
	*RCC_PLLCFGR &= ~(0b11 << 16);
	*RCC_CFGR &= ~(0b11 << 0);
	*RCC_CFGR |= (0b10 << 0);
	*RCC_CFGR &= ~(0b111 << 10);
	*RCC_CFGR |= (0b100 << 10);
}

void LED_Init() {
	volatile uint32_t* const RCC_AHB1ENR   = (uint32_t*)(0x40023800 + 0x30);
	*RCC_AHB1ENR |= (1 << 3);

	volatile uint32_t * const GPIOD_MODER = (uint32_t*)(0x40020c00 + 0x00);
	*GPIOD_MODER &= ~(0b11111111 << 24);
	*GPIOD_MODER |= (0b01010101 << 24);
}

void LED_Control(Led_state_t state) {
	uint32_t volatile* const GPIOD_ODR = (uint32_t*)(0x40020c00 + 0x14);
	if (state == ON) {
		*GPIOD_ODR |= (1 << 12);
	}
	else {
		*GPIOD_ODR &= ~(1 << 12);
	}
}
