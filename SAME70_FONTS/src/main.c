#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Bot?o Placa
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX  11
#define BUT_IDX_MASK (1 << BUT_IDX)
//butao 1 oled
#define EBUT1_PIO PIOD //start EXT 9 PD28
#define EBUT1_PIO_ID 16
#define EBUT1_PIO_IDX 28
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)
//butao 2 oled
#define EBUT2_PIO PIOA //pause  Ext 4 PA19 PA = 10
#define EBUT2_PIO_ID 10
#define EBUT2_PIO_IDX 19
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)
//butao 3 oled
#define EBUT3_PIO PIOC //sei la EXT 3 PC31
#define EBUT3_PIO_ID 12 // piod ID
#define EBUT3_PIO_IDX 31
#define EBUT3_PIO_IDX_MASK (1u << EBUT3_PIO_IDX)

volatile Bool but_flag;
volatile Bool but_p_freq;
volatile Bool but_m_freq;
volatile Bool but_stop = false;

#define YEAR        2018
#define MOUNTH      3
#define DAY         19 
#define WEEK        12
#define HOUR        0
#define MINUTE      0
#define SECOND      0

uint32_t hour, minuto, seg;
volatile uint8_t flag_led0 = 1;
int v, d, bot, count;
bot = 0;
count = 0;


void but_flag_callback(void){
	but_flag = true;
}
void but_p_freq_callback(void){
	but_p_freq = true;
}
void but_m_freq_callback(void){
	but_m_freq = true;
}
void but_stop_callback(void){	
	but_stop = true;
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	count += 4;

	if (flag_led0 == 1) {
				flag_led0 = 0;
				rtc_get_time(RTC, &hour, &minuto, &seg);
				rtc_set_time_alarm(RTC, 1, hour, 1, minuto, 1, seg+4);
				//tc_stop(TC0, 1);
				//rtc_disable_interrupt(RTC,  RTC_IER_ALREN);
				v = (2 * 3.14 * bot/4)*0.325;
				d = (v * count)*3.6
				bot = 0;
				
				char hnum5[5];
				itoa(v, hnum5, 10);
				gfx_mono_draw_string(hnum5,65,16, &sysfont);
				
				char hnum7[5];
				itoa(d, hnum7, 10);
				gfx_mono_draw_string(hnum7,68,16, &sysfont);
	}else{
			flag_led0 = 1;
			rtc_get_time(RTC, &hour, &minuto, &seg);
			rtc_set_time_alarm(RTC, 1, hour, 1, minuto, 1, seg+4);
			//tc_start(TC0, 1);
			//rtc_disable_interrupt(RTC,  RTC_IER_ALREN);	
			v = (2 * 3.14 * bot/4)*0.325;
			d = (v * count)*3.6
			bot = 0;
			char hnum6[5];
			itoa(v, hnum6, 10);
			gfx_mono_draw_string(hnum6,65,16, &sysfont);
			
			char hnum7[5];
			itoa(d, hnum7, 10);
			gfx_mono_draw_string(hnum7,68,16, &sysfont);		
		}
	}
	
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}


// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{
	board_init();
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();

	// Configura led da placa
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	// configura botoes do oled
	pmc_enable_periph_clk(EBUT1_PIO_ID);
	pmc_enable_periph_clk(EBUT2_PIO_ID);
	pmc_enable_periph_clk(EBUT3_PIO_ID);
	// configura botoes do oled como input
	pio_set_input(EBUT1_PIO,EBUT1_PIO_IDX_MASK,PIO_DEFAULT);
	pio_pull_up(EBUT1_PIO,EBUT1_PIO_IDX_MASK,PIO_PULLUP);
	pio_set_input(EBUT2_PIO,EBUT2_PIO_IDX_MASK,PIO_DEFAULT);
	pio_pull_up(EBUT2_PIO,EBUT2_PIO_IDX_MASK,PIO_PULLUP);
	pio_set_input(EBUT3_PIO,EBUT3_PIO_IDX_MASK,PIO_DEFAULT);
	pio_pull_up(EBUT3_PIO,EBUT3_PIO_IDX_MASK,PIO_PULLUP);
	// Inicializa clock do perif?rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	// Configura PIO para lidar com o pino do bot?o como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	// Configura interrup??o no pino referente ao botao e associa
	// fun??o de callback caso uma interrup??o for gerada
	// a fun??o de callback ? a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_flag_callback);
	pio_handler_set(EBUT1_PIO,
	EBUT1_PIO_ID,
	EBUT1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_p_freq_callback);
	pio_handler_set(EBUT2_PIO,
	EBUT2_PIO_ID,
	EBUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_m_freq_callback);
	pio_handler_set(EBUT3_PIO,
	EBUT3_PIO_ID,
	EBUT3_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_stop_callback);
	// Ativa interrup??o
	pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
	pio_enable_interrupt(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
	pio_enable_interrupt(EBUT3_PIO, EBUT3_PIO_IDX_MASK);

	// Ativa interrup??o
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(EBUT1_PIO_ID);
	NVIC_SetPriority(EBUT1_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(EBUT2_PIO_ID);
	NVIC_SetPriority(EBUT2_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(EBUT3_PIO_ID);
	NVIC_SetPriority(EBUT3_PIO_ID, 4); // Prioridade 4
}

void pisca_led(int hz){
	
	for (int i=0;i<hz;i++){
		pio_clear(LED_PIO, LED_IDX_MASK);
		delay_ms(200);
		pio_set(LED_PIO, LED_IDX_MASK);
		delay_ms(200);
	}
	but_flag = false;
	
	
}

int display_freq(int hz, int minu, int hora){
	gfx_mono_draw_string("  ",0,16, &sysfont);

	char hnum[5];
	itoa(hora, hnum, 10);
	gfx_mono_draw_string(hnum,0,16, &sysfont);
	
	//gfx_mono_draw_string("  ",30,16, &sysfont);
	char hnum1[5];
	itoa(minu, hnum1, 10);
	gfx_mono_draw_string(hnum1,30,16, &sysfont);

	gfx_mono_draw_string("  ",60,16, &sysfont);
	
	char hnum2[5];
	itoa(hz, hnum2, 10);
	gfx_mono_draw_string(hnum2, 60,16, &sysfont);
}


void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);



}



int main (void)
{
	io_init();
	delay_init();
	
	// Inicializa clock
	sysclk_init();
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

	// configura botao com interrupcao
	pio_set(LED_PIO, LED_IDX_MASK);
	
	RTC_init();
	
	rtc_get_time(RTC, &hour, &minuto, &seg);
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, hour, 1, minuto, 1, seg+4);
	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
	
	gfx_mono_ssd1306_init();
	
	gfx_mono_draw_string(":",50,16, &sysfont);
	gfx_mono_draw_string(":",20,16, &sysfont);
	
	/* Insert application code here, after the board has been initialized. */
	
	while(1) {
		rtc_get_time(RTC, &hour, &minuto, &seg);
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		//delay_ms(100);
		display_freq(seg, minuto,hour);
		
		
		if(but_stop){
			rtc_set_time(RTC, hour, minuto+1, seg);		
			but_stop = false;
		}
		else if(but_p_freq){	
			bot += 1;
			but_p_freq=false;
		}
		else if(but_m_freq){
			rtc_set_time(RTC, hour+1, minuto, seg);
			but_m_freq=false;
		}		
	}
}
