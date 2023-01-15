#include <stdint.h>

#include "clock.h"


#define AHB1_RCC_BASE		0x40023800UL
#define RCC_CR_OFFSET		0x0
#define RCC_CFGR_OFFSET		0x08
#define RCC_AHB1ENR_OFFSET	0x30
#define RCC_APB1ENR_OFFSET	0x40
#define RCC_APB2ENR_OFFSET	0x44
#define RCC_AHB1ENR_GPIOBEN	(1 << 1)
#define RCC_AHB1ENR_CCDATARAMEN	(1 << 20)
#define RCC_CR_ADDR		(AHB1_RCC_BASE + RCC_CR_OFFSET)
#define RCC_CFGR_ADDR		(AHB1_RCC_BASE + RCC_CFGR_OFFSET)
#define RCC_AHB1ENR_ADDR	(AHB1_RCC_BASE + RCC_AHB1ENR_OFFSET)
#define RCC_APB1ENR_ADDR	(AHB1_RCC_BASE + RCC_APB1ENR_OFFSET)
#define RCC_APB2ENR_ADDR	(AHB1_RCC_BASE + RCC_APB2ENR_OFFSET)
#define RCC_APB2ENR_TIM1EN	1

#define FLASHBASE 0x40023C00UL
#define FLASH_ACR_DCEN (1UL << 10)
#define FLASH_ACR_ICEN (1UL << 9)
#define FLASH_ACR_PRFTEN (1UL << 8)
#define FLASH_ACR_LATENCY_5 5
#define FLASH_ACR *((volatile uint32_t *const)(FLASHBASE))

#define RCC_PLLCFGR *((volatile uint32_t *const)(AHB1_RCC_BASE + 0x04UL))
#define RCC_PLLCFGR_PLLSRC_HSE (1UL << 22)
#define RCC_PLLCFGR_PLLN_168 (168UL << 6)
#define RCC_PLLCFGR_PLLQ_7 (7UL << 24)
#define RCC_PLLCFGR_PLLM_4 (4UL << 0)
#define RCC_PLLCFGR_PLLP_4 (0UL << 16)

#define RCC_CFGR_PPRE1_4 (5UL << 10)
#define RCC_CFGR_PPRE2_2 (4UL << 13)

void clock_init(void)
{
	volatile uint32_t *const RCC_CR = (volatile uint32_t *const)(RCC_CR_ADDR);
	volatile uint32_t *const RCC_CFGR = (volatile uint32_t *const)(RCC_CFGR_ADDR);
	volatile uint32_t *const RCC_AHB1ENR = (volatile uint32_t *const)(RCC_AHB1ENR_ADDR);
	volatile uint32_t *const RCC_APB1ENR = (volatile uint32_t *const)(RCC_APB1ENR_ADDR);
	volatile uint32_t *const RCC_APB2ENR = (volatile uint32_t *const)(RCC_APB2ENR_ADDR);
	const uint32_t RCC_AHB1ENR_DMA1EN = 1UL << 21;
	const uint32_t RCC_AHB1ENR_DMA2EN = 1UL << 22;

	FLASH_ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5;

	*RCC_APB2ENR |= 1UL << 14; // System configuration controller clock enable
	__sync_synchronize();
	volatile uint32_t tmpreg = *RCC_APB2ENR;

	*RCC_APB1ENR |= 1UL << 28; // Power interface clock enable
	__sync_synchronize();
	tmpreg = *RCC_APB1ENR;

	*RCC_CR |= 1UL << 16; // HSE oscillator ON
	while ((*RCC_CR & (1UL << 17)) == 0) { } // HSE oscillator ready

	*RCC_CFGR |= RCC_CFGR_PPRE1_4 | RCC_CFGR_PPRE2_2;

	RCC_PLLCFGR &= ~0xF0000000uL;
	RCC_PLLCFGR |= RCC_PLLCFGR_PLLQ_7;
	RCC_PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
	RCC_PLLCFGR &= ~0x7FC0uL;
	RCC_PLLCFGR |= RCC_PLLCFGR_PLLN_168;
	RCC_PLLCFGR &= ~0x3FuL;
	RCC_PLLCFGR |= RCC_PLLCFGR_PLLM_4;
	__sync_synchronize();
	
	*RCC_CR |= 1UL << 24; // Main PLL (PLL) enable
	while ((*RCC_CR & (1UL << 25)) == 0) { } // Main PLL (PLL) clock ready flag

	*RCC_CFGR |= 2UL; // PLL selected as system clock
	while ((*RCC_CFGR & (3UL << 2)) != (2UL << 2)) { } // PLL used as the system clock

	/* Enable AHB1 */
	*RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	__sync_synchronize(); /* This GCC built-in function issues a full memory barrier */

	/* wait at least one cycle */
	tmpreg = *RCC_AHB1ENR;
	(void)tmpreg;

	*RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;
	tmpreg = *RCC_APB2ENR;
}

static volatile uint32_t s_ticks = 0;
void clock_tick(void)
{
	s_ticks++;
}

void clock_delay_ms(uint32_t ms)
{
	if (ms == UINT32_MAX) {
		for ( ; ; ) {
			/* wait forever */
		}
	}
	ms++; // wait at least requested delay
	const uint32_t prev_ticks = s_ticks;
	while ((s_ticks - prev_ticks) < ms) {
		/* wait ms */
	}
}

static const uint32_t cpu_freq = 168000000UL;
uint32_t clock_sysreload_get(void)
{
	const uint32_t systick_freq = 1000UL;
	return cpu_freq / systick_freq - 1;
}

void clock_delay_us(uint32_t us)
{
	const uint32_t STK_VAL_addr = 0xE000E018UL;
	volatile uint32_t *const STK_VAL = (volatile uint32_t *const)STK_VAL_addr;
	const uint32_t cnt_start = *STK_VAL;
	__sync_synchronize();

	const uint32_t cnt_per_us = cpu_freq / 1000000;
	const uint32_t max_us = clock_sysreload_get() / cnt_per_us;
	if (us >= max_us) {
		for ( ; ; ) {
			/* wait forever */
		}
	}

	const uint32_t delay_cnt = us * cnt_per_us;
	if (delay_cnt < cnt_start) {
		const uint32_t cnt_stop = cnt_start - delay_cnt;
		for ( ; ; ) {
			const uint32_t cnt_curr = *STK_VAL;
			if ((cnt_curr < cnt_stop) || (cnt_curr > cnt_start)) {
				break;
			}
		}
	} else {
		const uint32_t cnt_stop = clock_sysreload_get() - delay_cnt + cnt_start;
		for ( ; ; ) {
			const uint32_t cnt_curr = *STK_VAL;
			if ((cnt_start < cnt_curr) && (cnt_curr < cnt_stop)) {
				break;
			}
		}
	}
}
