/*

This is a driver for the esp32 or esp32-s processors  "I2S" interface doing 8-Bit parallel IO.
 It can be used as:
 - camera input capture
 - general 8Bit input

 
 Interface has:
- 8 Data pins
- 1 Data strobe pin. (PCLK , WS, or similar)
- optinally VSYNC, HREF for camera can be used to gate the input data. special pin numbers for constant level are: 0x30 = 0; 0x38 = 1;

IO is done using a fixed data length which can be configured.

The input operation can work continuously where data is supplied into multible buffers marked ready for an application to read.
Buffers taken by the application for processing are marked "blocked" until application "releases" that buffer.
Buffers will overrun on a least used bases, so camera frames in the buffers are "recent".

The DMA controller is used to transfer Data to/from the I2S module without CPU interaction.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "rom/lldesc.h"


static const char *TAG = "I2S";

//+++ data structures:

typedef enum
{
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     *  Mode-0: fifo receives: 0x01 0x00 0x01 0x00 0x02 0x00 0x02 0x00     0x03 0x00 0x03 0x00 0x04 0x00 0x04 0x00
     *  Mode-1: fifo receives: 0x02 0x00 0x01 0x00 0x04 0x00 0x03 0x00
     *  Mode-2: fifo receives: 0x00 0x00 0x01 0x00 0x00 0x00 0x01 0x00     0x00 0x00 0x02 0x00 0x00 0x00 0x02 0x00 .....
     *  Mode-3: fifo receives: 0x00 0x00 0x01 0x00 0x00 0x00 0x02 0x00     0x00 0x00 0x03 0x00 0x00 0x00 0x04 0x00 
     */
    SM_0A0B_0B0C = 0,
   SM_0A0B_0C0D = 1,

    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

 


//+++protos:
void I2S_Stop();
void I2S_Start();
int i2sInit(void);
int i2s_set_rxsize(int size);

//#define CAM_PIN_SIOD    26
//#define CAM_PIN_SIOC    27
//vsynv = 25
//pclk = 22
//href = 23

//+++global data:

// +++PIN definitions for the parallel interface:
// use 0x30 for constant LOW, or 0x38 for constant HIGH Levels, as PIN-number, if there is no related real IO-signal.
//esp32-cam PIN Map for OV2640:
#define D7      35
#define D6      34
#define D5      39
#define D4      36
#define D3      21
#define D2      19
#define D1      18
#define D0      5
#define VSYNC   25  
#define HREF    23  
#define PCLK    22 // this is the data strobe signal, rising edge (falling edge also possible to config)

uint8_t PinTab[]= {VSYNC, HREF, PCLK, D0, D1, D2, D3, D4, D5, D6, D7};

// +++DMA buffer stuff:
/*
The DMA buffer descriptor is a hardware specific structure used by the esp32 DMA controller.
   DMA Desc struct, aka lldesc_t
  
  -------------------------------------------------------------- 
  | own | EoF | sub_sof | 5'b0   | length [11:0] | size [11:0] |
  --------------------------------------------------------------
  |            buf_ptr [31:0]                                  |
  --------------------------------------------------------------
  |            next_desc_ptr [31:0]                            |
  --------------------------------------------------------------
 
typedef struct lldesc_s 
{
    volatile uint32_t size  :12,   // total size in Bytes of the pointed-to DataBuffer. max 4096 bytes. 12bit=4095, count sequence 0 to n-1
    length:12, // (IO-DataLen).TX=set by software = valid NuMBytes in DataBufer. RX= set by hardware= NumBytes received in DataBuffer. max 4096 bytes
    offset: 5, // h/w reserved 5bit field, s/w use it as offset in buffer.part of reserved Bitfield[6]. ?? not used
    sosf  : 1, // start of sub-frame. part of reserved Bitfield[6]. ?? not used.the reserved Bitfield[6] can be used as RW register by software.
    eof   : 1, // if set, indicates last used descriptor. set by hardware in RX. set by software in TX.
    owner : 1; // hw or sw . should be 1 for DMA. cleared by hardware at end.use PERI_IN_LOOP_TEST bit to disable clearing.
	
    volatile uint8_t *buf;       // pointer to memory address of related DataBuffer 
    union{
        volatile uint32_t empty;
        STAILQ_ENTRY(lldesc_s) qe;  // IRAM pointer to next descriptor, or NULL if last descriptor in the chain.
    };
} lldesc_t; // DMA buffer descriptor struct, as defined in esp-idf/components/esp32/include/rom/lldesc.h

The DMA controller will start transfers using the first descriptor which is given to him by the I2S module.
When the related buffer is full, it will automaticly load the next descriptor from the linked list until it finds the eof-bit set or
the transfer count is exhausted.
DMA transfers are triggered by the I2S-Fifo logic which is always the source/destination to/from memory(IRAM prefered!). 
The minimum transfersize is 4Bytes or 1 DWORD(32bit).
*/

DMA_ATTR lldesc_t DMAdesc[4]; // we are using max 4 dma descriptors
DMA_ATTR uint8_t DMAbuf[4][4096]; // and max 4 dmabuffers


// data buffer stuff

intr_handle_t i2sInterruptHandle = 0;

int Dcnt;  // debug counter
uint8_t* pDataBuf = 0; // pointer to the currently used Databuffer
int SampleCount = 0; // transfersize/(databytes per DWORD) . current samplecount is: max:4092/4 or if less: transfersize/4
int BytesPerSample;



// Reset DMA and I2S
void i2sConfReset()
{
// reset DMA controller	, AHB interface and AHB cmdFifo
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags; // set reset bits
    I2S0.lc_conf.val &= ~lc_conf_reset_flags; // clear reset bits

// perform fifo and receiver reset, also transmitter
    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back); // wait for reset done
}



// this triggers when requested number of bytes have been received
void IRAM_ATTR i2sInterrupt(void *arg) // arg is just dummy for that stupid intr_handler_t . parameter to an interrupt.
{
    I2S0.int_clr.val = I2S0.int_raw.val; // clear all int flags??
Dcnt++;
}

void IRAM_ATTR vsync_isr(void *arg)
{
Dcnt++;
}

void I2S_Stop()
{
    I2S0.conf.rx_start = 0;
    esp_intr_disable(i2sInterruptHandle);
    i2sConfReset();
    I2S0.int_clr.val = 0x1ffff; // clear all intflags



}

void I2S_Start()
{
	i2sConfReset();
	
    I2S0.rx_eof_num = SampleCount; // the datalength/4 to be received
    I2S0.in_link.addr = (uint32_t) &DMAdesc[0]; //the address of the first DMAdescriptor to use
    I2S0.in_link.start = 1; // start the inlink descriptor
    I2S0.int_clr.val = I2S0.int_raw.val; // clear possible intflags
	//interrupts:
    I2S0.int_ena.val = 0; // disable all ints
    I2S0.int_ena.in_done = 1; //enable.Triggered when current inlink descriptor is handled. ????????????????????????????
    esp_intr_enable(i2sInterruptHandle);
    I2S0.conf.rx_start = 1; // start receiving data. this bit starts/stops the receiver!!!
}



void set_samplemode( int mode)
{
	volatile uint8_t *pb;
	int i;
	
	switch (mode)
	{
		case 0:
		BytesPerSample = 4;  // how many bytes are used up in DMAbuffer for one Sample/clocked input Byte.
		break;
		case 1:
		BytesPerSample = 2; // but fetches only after two clocks, so even amount of samples!! because DMA only transfer 32bit - so 2*16bits=2 clocks
		break;
		case 2:
		BytesPerSample = 8; // here two DMA transfers are done per input clock!
		break;
		case 3:
		BytesPerSample = 4;  // same as 0, one DMA transfer per input clock
		break;
		default:
		return;
	}

	I2S_Stop();
	i2sConfReset();
    I2S0.fifo_conf.rx_fifo_mod = mode; //SampleMode
	Dcnt=0;

	pb=DMAdesc[0].buf;
	for (i=0;i<16;i++) *pb++ = 0;
	DMAdesc[0].owner=1;
	DMAdesc[0].eof=0;
	DMAdesc[0].length=0;

	

	I2S_Start();	
}



/* Main Init. Init the I2S interface with given portpins set in the defines up top of file.
This shall be called only once after reset!!

uses global PinTab table and defines of portpins.
  Format: VSYNC, HREF, PCLK, D0, D1, D2, D3, D4, D5, D6, D7  for 8Bit interface
entry: samplemode 0 to 3
exit:
- 0 = fail
- 1 = OK, I2S is operational but not running
*/
int i2sInit(void)
{
	int i;
	
	I2S_Stop(); // just in case
	
    gpio_config_t conf =
    {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    }; // init the port config struct to use for all pins

    for (i = 0; i < sizeof(PinTab); ++i)
    {
        conf.pin_bit_mask = 1LL << PinTab[i]; // set all pins to input, no pullup/down/interrupt
        gpio_config(&conf);
    }

    // Route Pins to I2S peripheral using GPIO matrix, last parameter is invert
    gpio_matrix_in(D0,    I2S0I_DATA_IN0_IDX, 0);
    gpio_matrix_in(D1,    I2S0I_DATA_IN1_IDX, 0);
    gpio_matrix_in(D2,    I2S0I_DATA_IN2_IDX, 0);
    gpio_matrix_in(D3,    I2S0I_DATA_IN3_IDX, 0);
    gpio_matrix_in(D4,    I2S0I_DATA_IN4_IDX, 0);
    gpio_matrix_in(D5,    I2S0I_DATA_IN5_IDX, 0);
    gpio_matrix_in(D6,    I2S0I_DATA_IN6_IDX, 0);
    gpio_matrix_in(D7,    I2S0I_DATA_IN7_IDX, 0);
	
    gpio_matrix_in(0x30,  I2S0I_DATA_IN8_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN9_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN10_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN11_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN12_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN13_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN14_IDX, 0);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN15_IDX, 0);

    gpio_matrix_in(0x30, I2S0I_V_SYNC_IDX, 1);  //+++++++debug!! VSYNC   INVERT!!!!!!!!!!!!!!
    gpio_matrix_in(0x38,  I2S0I_H_SYNC_IDX, 0);  //0x30 sends 0, 0x38 sends 1
    gpio_matrix_in(0x38,  I2S0I_H_ENABLE_IDX, 0); //+++++++debug!! HREF
    gpio_matrix_in(PCLK,  I2S0I_WS_IN_IDX, 0); 
	
	//PortPins are configured now

    // Power on the I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);

    // Reset I2S and DMA controller
    i2sConfReset(); //??
    // Enable slave receiver mode (WS is input clock/strobe)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel camera mode
    I2S0.conf2.lcd_en = 1;
    // enable camera-mode. Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1; // Fractional clock divider denominator value
    I2S0.clkm_conf.clkm_div_b = 0; // Fractional clock divider numerator value
    I2S0.clkm_conf.clkm_div_num = 2; // Integral I2S clock divider value
    // enable I2S DMA mode
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    //two bytes per dword packing CHECK THIS****************************
    I2S0.fifo_conf.rx_fifo_mod = 1; //SampleMode
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
	//rx channel mode
    I2S0.conf_chan.rx_chan_mod = 1;  
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 8;  // bus width **************** was 0
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;
	
    I2S0.int_clr.val = 0x1ffff; // clear all intflags


    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,&i2sInterrupt, NULL, &i2sInterruptHandle);

// install vsync int and leave it running
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(VSYNC, &vsync_isr, NULL);
	gpio_set_intr_type(VSYNC, GPIO_INTR_NEGEDGE); //enable int on negedge. GPIO_INTR_DISABLE

    return 1;
}

 
/* initializes the i2s dma buffer size.
entry:
- bytes = requested number of input bytes, must be divby4.
exit:
- 0 = Ok
- 1 = not div by 4 
- 2 = too many bytes requested, max: 4*4096 = 16384
*/
int i2s_set_rxsize(int size)
{
	int i,n,bytes;
	
	bytes = size;
	if (bytes%4) 
	{
		ESP_LOGE(TAG, "size not div by 4!");
		return 1;
	}
	if (bytes > 16368) 
	{
		ESP_LOGE(TAG, "size > 16368!");
		return 2;
	}

	memset(DMAdesc, 0, sizeof(DMAdesc)); //preclear everything
	SampleCount=0;
	n=0;
	while(bytes>4092)
	{
		n++;
		bytes -= 4092;
		SampleCount += 4092/4;
	}
	// n now contains the needed amount of fullsize dma descriptors
	for (i=0;i<n;i++)
	{
		DMAdesc[i].owner=1;
		DMAdesc[i].size = 4092;
		DMAdesc[i].length = 4092;
		DMAdesc[i].buf = (uint8_t*)&DMAbuf[i];
		DMAdesc[i].qe.stqe_next=&DMAdesc[i+1];
	}
	// set the last or only one
		DMAdesc[i].owner=1; // DMA can use it
		DMAdesc[i].size = bytes;  // this important=destination buffersize
		DMAdesc[i].length = 0; // set by hardware at EOT=total bytes transfereed, not important
		DMAdesc[i].buf = (uint8_t*)&DMAbuf[i];  //important!
		DMAdesc[i].qe.stqe_next=&DMAdesc[0]; // set the last next-pointer to the first to form a ring
		SampleCount += bytes/4;
    I2S0.rx_eof_num = size; // set the datasize to be received at which an interrupt will occur
    I2S0.in_link.addr = (uint32_t)&DMAdesc[0]; // set address of first descriptor
	ESP_LOGI(TAG, "set_size: descrCnt:%d, size:%d",i,size);	
return 0;		
}

uint32_t get_intstat(void)
{
	return I2S0.int_raw.val;
}

void print_status(void)
{
	volatile uint8_t *pb;
	int i;

	printf("DMAsize:%u inlen:%u eof:%u owner:%u  -  Dmabuffer-BytesPerSample:%d\nDMA-Buffer[0-15]:\n",DMAdesc[0].size,DMAdesc[0].length,DMAdesc[0].eof,DMAdesc[0].owner,BytesPerSample);
	pb=DMAdesc[0].buf;
	for (i=0;i<16;i++)
		printf("0x%02x ",*pb++);
}
