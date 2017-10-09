//ghetto-chrono on STM32F103C8T6
//using one 16-bit timer to achieve 32-bit input capture

#include "gpio.h"					//we use F_CPU + gpio functions
#include "delay.h"					//we use software delays
#include "coretick.h"				//we use dwt
//#include "systick.h"				//we use systick timer

//hardware configuration
#define LED_PORT				GPIOB
#define LED_DDR					GPIOB
#define LED_START				(1<<12)
#define LED_STOP				(0<<11)	//0->not used

#define CHRONO_PORT				GPIOB
#define CHRONO_DDR				GPIOB
#define CHRONO					(1<<0)		//PB0 = TIM3_CH1

#define CHRONO_PS				1		//tmr3 prescaler
#define CHRONO_DISTANCE			1234		//chrono sensor distance, x10mm (1234=123.4mm)
#define CHRONO_TRIGGER			RISING		//input capture on rising / falling edge
#define CHRONO_DP							//define CHRONO_DP if you want to show decimal point on digit 3/4.
//#define FAST_MATH							//using faster math so the code runs at 1Mhz
//end hardware configuration

//global defines
#define RISING					0
#define FALLING					1

//end hardware configuration

//global defines
//led indicators - active high
#define LED_ON(LEDs)			IO_SET(LED_PORT, LEDs)
#define LED_OFF(LEDs)			IO_CLR(LED_PORT, LEDs)

//global variables
uint32_t time0, time1;
volatile uint32_t chrono_ticks=0;			//32-bit ticks elapsed between start / end
volatile char chrono_available=0;			//data availability flag. 1=new data available, 0=no new data available
volatile uint32_t ticks=0;					//32-bit ticks. LSW is 16-bit timer capture

//tim3 irq
void TIM3_IRQHandler(void) {
	static uint32_t chrono_start, chrono_end;
	static char chrono_ch=0;				//chrono input channel. 0=>put ICR1 into start, 1->put ICR1 input end

	//timer overflow
	if (TIM3->SR & (1<<0)) {
		TIM3->SR &=~(1<<0);					//clear the flag
		ticks += 0x10000ul;					//16-bit timer
	}

	//input capture
	if (TIM3->SR & (1<<1)) {
		//clear the flag
		TIM3->SR &=~(1<<1);						//0->clear the flag, 1->set the flag
		if ((chrono_ch++ & 0x01)==0) {			//ch = 0 right now -> ICR1 to start
			chrono_start = ticks | TIM3->CCR1;			//save ICR1 to chrono_start
			LED_OFF(LED_START);					//turn off the start led
		} else {								//ch = 1 right now -> ICR1 to end
			chrono_end = ticks | TIM3->CCR1;			//save ICR1 to end
			chrono_ticks = chrono_end - chrono_start;	//calculate ticks elapsed
			chrono_available = 1;				//1->new data available
			LED_OFF(LED_STOP); 					//turn off the stop led
		}
	}
}

//initialize the chrono
//using tim3_ch1 input capture as master
void chrono_init(void) {
	ticks = chrono_ticks = 0;		//reset the ticks
	chrono_available = 0;			//0->no new data, 1->new data

	//configure input pin as input
	IO_IN(CHRONO_DDR, CHRONO);
	//should consider pull-up in final implementation

	//configure led as output
	LED_OFF(LED_START | LED_STOP);	//led start/stop are off
	IO_OUT(LED_DDR, LED_START | LED_STOP);	//as output

	//configure TIM3 as free-running timer with prescaler at 1x
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;	//1->enable clock
	//TIM3->CR1 &=~(1<<0);			//0->turn off the timer, 1->turn on the timer
	TIM3->CR1 = 	(0<<8) |		//0b00->sampled at 1:1
					(0<<7) |		//0->apr not buffer, 1->apr buffered
					(0x00<<5) |		//0->edge aligned. 1/2/3->center aligned mode 1/2/3
					(0<<4) |		//0->upcounter, 1->downcounter
					(0<<3) |		//0->contineous mode, 1->one-pulse mode
					(0<<2) |		//UEV update request source. default 0
					(0<<1) |		//0->UEV enabled, 1->UEV disabled
					(0<<0) |		//0->timer disabled, 1->timer enabled
					0x00;
	TIM3->CR2 = 	(0<<7) |		//0->TI1 connected to ch1; 1->TI1 connected to ch1/ch2/ch3 (xor operation)
					(0x00<<4) |		//0->master mode reset;
					(0<<3) |		//0->capture compare DMA disabled, 1->enabled
					0x00;
	TIM3->SMCR = 0;					//reset value for slave mode register
	TIM3->CNT = 0;					//reset the counter
	TIM3->ARR = 0xffff;				//autoreloading register
	TIM3->PSC = CHRONO_PS-1;		//set prescaler = 0 -> 1:1, ...
	//TIM3->EGR = 0;				//1->generate an event
	//start the tim3
	TIM3->CR1 |= (1<<0);			//0->turn off the timer, 1->turn on the timer

	//configure the input capture
	TIM3->CCER &=~(1<<0);			//0->capture disabled, 1->capture enabled
	TIM3->CCR1 = 0x0000;			//reset capture 1
	TIM3->CCMR1 = (TIM3->CCMR1 &~(0x0f<<4)) | (0x00 & (0x0f<<4));		//0->input capture filter 1:1
	TIM3->CCMR1 = (TIM3->CCMR1 &~(0x03<<2)) | (0x00 & (0x03<<3));		//0->input capture prescaler = 1:1
	TIM3->CCMR1 = (TIM3->CCMR1 &~(0x01<<0)) | (0x01 & (0x01<<0));		//0->ch1 as output; 1->ic1 mapped to TI1, 2->ic1 mapped to TI2, 3->ic1 mapped to TRC
#if CHRONO_TRIGGER == RISING		//rising edge
	TIM3->CCER &=~(1<<1);			//0->capture on rising edge, 1->on falling edge
#else								//falling edge
	TIM3->CCER |= (1<<1);			//0->capture on rising edge, 1->on falling edge
#endif
	TIM3->CCER |= (1<<0);			//0->capture disabled, 1->capture enabled

	TIM3->SR &=~((1<<1) | (1<<0));			//0->clear input capture / compare interrupt ch1 + overflow interrupt
	TIM3->DIER |= (1<<1) | (1<<0);			//1->enable input capture / compare interrupt ch1, 0->disable + overflow interrupt
	//configure input capture on TIM3_CH1/TI1

	//set up the interrupt
	NVIC_EnableIRQ(TIM3_IRQn);		//enable irq
	//priorities not set -> default values used.

}

//conversion routines
//converting ticks to us
uint32_t ticks2usx10(uint32_t ticks) {
	ticks *= TIM3->PSC + 1; 			//correct for prescalers
	return ticks * 10 / (F_CPU / 1000000ul);		//return us
}

//convert ticks to mpsx10 using floating point math
//for demo only, not used -> too slow / bulky
uint32_t ticks2mpsx10_fp(uint32_t ticks) {
	//return (float) CHRONO_DISTANCE * 0.001 / ((float) ticks2us(ticks) * 1e-6);
	//return (float) CHRONO_DISTANCE * 1000.0 * 10.0 / (float) ticks2usx10(ticks);
}

//convert ticks to meters per second x 10 (mpsx10) using integer math
uint32_t ticks2mpsx10(uint32_t ticks) {
	uint32_t tmp = (uint32_t) CHRONO_DISTANCE * 1000ul * (F_CPU / 1000000ul) / ticks;
	tmp /= TIM3->PSC + 1;
	return tmp;
}

//convert ticks to ft per second x 10 (fpsx10) using integer math
uint32_t ticks2fpsx10(uint32_t ticks) {
	return ticks2mpsx10(ticks) * 32804ul/10000;			//1meter = 3.28084
}


int main(void) {
	int i=0, cnt=10, tmp=0;
#if defined(FAST_MATH)
	int tmp1;									//temp for fast_math calculation
#endif

	mcu_init();									//reset the  mcu
	coretick_init();							//initialize core_ticks

	//remap PB4 to GPIO
	//enable afio, and then set SWJ_CFG to 0b001 to enable GPIO on PB4
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	AFIO->MAPR = (AFIO->MAPR &~(0x01<<24)) | (0x01 << 24);	//SWJ_CFG = 0b001 -> PB4 to GPIO

	//led_init();									//reset the led module
	chrono_init();								//reset the chrono
	//systick_init();								//reset systick - for benchmarking
	ei();										//enable global interrupts
	while (1) {
		//display cnt
		//chrono_available=1;
		TIM3->EGR |= (1<<1);					//trigger the first capture
		//delay(10);								//waste sometime
		//waste sufficient amount of time
		time0=coreticks();						//time stamp
		while (coreticks() - time0 < (1ul<<17)) continue;	//waste some time
		time0=coreticks() - time0;				//calculate elapsed time -> make sure it is more than 0xffff ticks
		TIM3->EGR |= (1<<1);					//trigger the 2nd capture
		if (chrono_available) {					//new data is available
			chrono_available = 0;				//0->no new data available
			//chrono_ticks=1234ul;				//for debugginh only
			tmp = chrono_ticks;					//disable chrono_ticks
			//tmp = ticks2usx10(chrono_ticks);
			//tmp = ticks2mpsx10(chrono_ticks);
			//tmp = ticks2fpsx10(chrono_ticks);
			//bound tmp to 9999
			tmp = (tmp > 9999)?9999:tmp;
			//tmp = tmp % 10000;					//bound it to [0..9999]
			//display tmp - forming lRAM (display buffer)
			//time0=systicks();
			//led_display();							//display lRAM
			//time1=systicks() - time0;
			//NOP();
		}
	};
}
