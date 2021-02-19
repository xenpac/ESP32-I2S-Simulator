/*
I2S Simulator.

For testing the 8Bit parallel input capture of the i2s interface step by step (clock by clock) and view the received datastage in between.
Learn how the I2S and DMA process is functioning.

There must be no camera connected to the esp32 modul.
All input signals are simulated by writing values to the PortPins output register, which in turn are used as input signals.


It needs an esp32 modul connected via serial monitor for interactive control.
A main menue lists available features.
*/











#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_eth.h"
#include "tcpip_adapter.h"
#include "driver/ledc.h"


#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"


// pin for pclk
#define CLOCKPIN 22

extern int Dcnt;
void print_status(void);

//extern
int i2sInit(void);
int i2s_set_rxsize(int size);
void I2S_Start(void);
void I2S_Stop(void);
uint32_t get_intstat(void);
void set_samplemode( int mode);

//protos:
void setbyte(uint8_t b);
uint8_t getbyte(void);
void setbit(gpio_num_t pin, int val);
void pclk(gpio_num_t pin, int cnt);
void mainmenue(void);
int GetPinVal(gpio_num_t pin);
int IsOutput(gpio_num_t pin);
esp_err_t start_clock(gpio_num_t pin);
esp_err_t timer_conf(int ledc_timer, int xclk_freq_hz);

//serial io
void strcopy( char *s, char *d, int len);
void putcc(char c);
char getcc(void);
void getss(char *buf);
void putss(const char *ps);

//global data:
int ClockCnt;
int SampleMode;


void app_main(void)
{
    mainmenue();
}

// returns 1 if output, else 0
int IsOutput(gpio_num_t pin)
{
    int state;
    if (pin < 32)
    {
        state = GPIO_REG_READ(GPIO_ENABLE_REG) & BIT(pin);
    }
    else
    {
        pin -= 31;
        state = GPIO_REG_READ(GPIO_ENABLE1_REG) & BIT(pin);
    }
    return state;
}




// returns current value of in or out pin as 1 or 0
int GetPinVal(gpio_num_t pin)
{
    int state;

    if (IsOutput(pin))
    {
        //pin is output - read the GPIO_OUT_REG register
        if (pin < 32)
        {
            state = (GPIO_REG_READ(GPIO_OUT_REG)  >> pin) & 1U;
        }
        else
        {
            pin -= 31;
            state = (GPIO_REG_READ(GPIO_OUT1_REG)  >> pin) & 1U;
        }
    }
    else
    {
        //pin is input - read the GPIO_IN_REG register
        if (pin < 32)
        {
            state = (GPIO_REG_READ(GPIO_IN_REG)  >> pin) & 1U;
        }
        else
        {
            pin -= 31;
            state = (GPIO_REG_READ(GPIO_IN1_REG)  >> pin) & 1U;
        }
    }
    return state;
}

void setpin_level(gpio_num_t pin, uint32_t level)
{
    if (level) {
        if (pin < 32) {
            GPIO.out_w1ts = (1 << pin);
        } else {
            GPIO.out1_w1ts.data = (1 << (pin - 32));
        }
    } else {
        if (pin < 32) {
            GPIO.out_w1tc = (1 << pin);
        } else {
            GPIO.out1_w1tc.data = (1 << (pin - 32));
        }
    }

}

void putcc(char c)
{
    fputc(c, stdout);
}

char getcc(void)
{
    char c;

    while (1)
    {
        vTaskDelay(10/portTICK_PERIOD_MS); //min 10 for watchdog not to trigger!
        c = fgetc(stdin);
        if (c!=0xFF) break;
    }
    putcc(c);
    return c;
}

void getss(char *buf)
{
    char *ps, c;
    ps=buf;
    while (1)
    {
        c=getcc();
        if (( c == 0x0d)||(c==0x0a)) // CR LF
            break;
        *ps++=c;
    }
    *ps=0;
    putcc(0x0d);
    putcc(0x0a);
}

void putss(const char *ps)
{
    while (*ps)
    {
        putcc(*ps++);
    }
}

// string copy where len is the total fieldlength including 0.
void strcopy( char *s, char *d, int len)
{
    len--;
    while (len--&&*s) *d++=*s++;
    *d=0;
}


const char *menue="\n***Menue***\n"
                  "l-list state\n"
                  "c-clock1\n"
                  "s-set SampleMode\n"
                  "i-set inport Data\n"
                  ;
const char *prompt="\nI2Ssim>";


void mainmenue(void)
{
	uint32_t l;
    char sbuf[256];
    uint8_t c,pval;

	SampleMode=1;
	i2sInit();
	i2s_set_rxsize(16);
	set_samplemode(SampleMode);
	setbyte(0x01);
	pval=getbyte();
	gpio_pullup_en(CLOCKPIN); // set pclk to initial high level at start

    putss(menue);
    putss(prompt);

    while(1) // menue loop
    {
        c = getcc();
        switch(c)
        {
        case 'l': //list
            sprintf(sbuf,"\n+++ClockCnt:%d  IntCnt:%d  DataPort:0x%02x SampleMode:%d\n",ClockCnt,Dcnt,getbyte(),SampleMode);
            putss(sbuf);
			print_status();
            break;

        case 'c': //
			pclk(CLOCKPIN,1);
			ClockCnt++;
			pval++; // inc val per clock
			setbyte(pval);
            break;

        case 's': 
		ClockCnt=0;
		Dcnt=0;
		setbyte(0x01);
		pval=getbyte();
		putss("\n+Reset done..\n");
		putss("Enter SampleMode[0-3]");
		getss(sbuf);
		if (sscanf(sbuf,"%u",&l) == 1)
		{
			if (l<4)
			{
			SampleMode=l;
			set_samplemode(SampleMode);
			}
			else putss("wrong value!\n");
		}
		else putss("invalid!\n");
            break;
			
		case 'i':
		c=getbyte();
		sprintf(sbuf,"InPort is:0x%02x\n",c);
		putss(sbuf),
		putss("Enter new value hex:0x");
		getss(sbuf);
		if (sscanf(sbuf,"%02x",&l) == 1)
		{
			c=l;
			sprintf(sbuf,"\nNew value: 0x%02x\n",c);
			putss(sbuf);
			setbyte(c);
			pval=c;
		}
		break;

        default:
            putss(menue);
        }
        putss(prompt);


    }

}


void setbit(gpio_num_t pin, int val)
{
	if (val)
	{
			gpio_pullup_en(pin); //set high
			gpio_pulldown_dis(pin);
	}
	else
	{
			gpio_pulldown_en(pin); // set low
			gpio_pullup_dis(pin);
	}
			vTaskDelay(1/portTICK_PERIOD_MS); 

}

// the 8bit databus as input. 34 to 39 are input only with no pullup capability!
#define D7      35
#define D6      34
#define D5      39
#define D4      36
#define D3      21
#define D2      19
#define D1      18
#define D0      5
gpio_num_t PinTb[]= {D0, D1, D2, D3, D4, D5, D6, D7};

void setbyte(uint8_t b)
{
	int i,val;
	for (i=0;i<8;i++)
	{
		val=b&0x01;
		setbit(PinTb[i],val);
		b>>=1;
	}
}

uint8_t getbyte(void)
{
	int i,ret;
	uint8_t b = 0;
	for (i=7;i>=0;i--)
	{
		b<<=1;
		 ret=gpio_get_level(PinTb[i]);
		if (ret) b|=0x01;
	}
	return b;
}

void pclk(gpio_num_t pin, int cnt)
{
	int i;
			for (i=0;i<cnt;i++) // do pclks
		{
			setbit(pin, 0);
			setbit(pin, 1);
		}

}

esp_err_t timer_conf(int ledc_timer, int xclk_freq_hz)
{
    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = 2;
    timer_conf.freq_hz = xclk_freq_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
#if ESP_IDF_VERSION_MAJOR >= 4
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
#endif
    timer_conf.timer_num = (ledc_timer_t)ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK)
    {
        printf("ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
    }
    return err;
}

esp_err_t start_clock(gpio_num_t pin)
{
    periph_module_enable(PERIPH_LEDC_MODULE);

    esp_err_t err = timer_conf(0, 1000);
    if (err != ESP_OK)
    {
        printf("ledc_timer_config failed, rc=%x", err);
        return err;
    }

    ledc_channel_config_t ch_conf;
    ch_conf.gpio_num = pin; // output pin
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.channel = 0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = 0;
    ch_conf.duty = 2;
    ch_conf.hpoint = 0;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK)
    {
        printf("ledc_channel_config failed, rc=%x", err);
        return err;
    }
    return ESP_OK;
}
