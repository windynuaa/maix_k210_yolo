#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "fpioa.h"
#include "lcd.h"
#include "board_config.h"
#include "plic.h"
#include "sysctl.h"
#include "uarths.h"
#include <bsp.h>
#include <sysctl.h>
#include "ov2460.h"
#include "utils.h"
#include "kpu.h"
#include "region_layer.h"
#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "incbin.h"

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL
#define DCX_GPIONUM             (2)
#define RST_GPIONUM             (0)

#define CLASS_NUMBER 20

#define LOAD_KMODEL_FROM_FLASH 0


INCBIN(model, "yolo.kmodel");


kpu_model_context_t task;
static region_layer_t detect_rl;

volatile uint8_t g_ai_done_flag;
static uint16_t lcd_gram[320 * 240] __attribute__((aligned(32)));



#define ANCHOR_NUM 5

float g_anchor[ANCHOR_NUM * 2] = {1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52};


static void io_mux_init(void)
{
    /* Init DVP IO map and function settings */
    fpioa_set_function(42, FUNC_CMOS_RST);
    fpioa_set_function(44, FUNC_CMOS_PWDN);
    fpioa_set_function(46, FUNC_CMOS_XCLK);
    fpioa_set_function(43, FUNC_CMOS_VSYNC);
    fpioa_set_function(45, FUNC_CMOS_HREF);
    fpioa_set_function(47, FUNC_CMOS_PCLK);
    fpioa_set_function(41, FUNC_SCCB_SCLK);
    fpioa_set_function(40, FUNC_SCCB_SDA);

    /* Init SPI IO map and function settings */
    fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
    fpioa_set_function(36, FUNC_SPI0_SS3);
    fpioa_set_function(39, FUNC_SPI0_SCLK);
    fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);

    sysctl_set_spi0_dvp_data(1);
}

static void io_set_power(void)
{
    /* Set dvp and spi pin to 1.8V */
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
}



#if (CLASS_NUMBER > 1)
typedef struct
{
    char *str;
    uint16_t color;
    uint16_t height;
    uint16_t width;
    uint32_t *ptr;
} class_lable_t;

class_lable_t class_lable[CLASS_NUMBER] =
{
    {"aeroplane", GREEN},
    {"bicycle", GREEN},
    {"bird", GREEN},
    {"boat", GREEN},
    {"bottle", 0xF81F},
    {"bus", GREEN},
    {"car", GREEN},
    {"cat", GREEN},
    {"chair", 0xFD20},
    {"cow", GREEN},
    {"diningtable", GREEN},
    {"dog", GREEN},
    {"horse", GREEN},
    {"motorbike", GREEN},
    {"person", 0xF800},//face 14
    {"pottedplant", GREEN},
    {"sheep", GREEN},
    {"sofa", GREEN},
    {"train", GREEN},
    {"tvmonitor", 0xF9B6}
};

static uint32_t lable_string_draw_ram[115 * 16 * 8 / 2];
#endif

static void lable_init(void)
{
#if (CLASS_NUMBER > 1)
    uint8_t index;

    class_lable[0].height = 16;
    class_lable[0].width = 8 * strlen(class_lable[0].str);
    class_lable[0].ptr = lable_string_draw_ram;
    lcd_ram_draw_string(class_lable[0].str, class_lable[0].ptr, BLACK, class_lable[0].color);
    for (index = 1; index < CLASS_NUMBER; index++) {
        class_lable[index].height = 16;
        class_lable[index].width = 8 * strlen(class_lable[index].str);
        class_lable[index].ptr = class_lable[index - 1].ptr + class_lable[index - 1].height * class_lable[index - 1].width / 2;
        lcd_ram_draw_string(class_lable[index].str, class_lable[index].ptr, BLACK, class_lable[index].color);
    }
#endif
}




uint16_t *framebuffer; //display buffer
	
static int ai_done(void *ctx)
{
    g_ai_done_flag = 1;
    return 0;
}

static volatile int hook_1;
static volatile int x1,x2,y1,y2;//bbox 
/*
	K210 has two cores, so we use core0 for AI computation and core1 for display
	known bug:
		Sometimes CORE1 may crash.
	
*/
int core1_function(void *ctx) // for display while core0 for AI 
{
    while(1)
    {
		Sipeed_OV2640_sensor_snapshot();
		lcd_draw_picture(0, 0, 320, 240, framebuffer);
		if(hook_1>0)
			lcd_draw_rectangle(x1, y1, x2, y2, 4, RED);
    }
}

static void drawboxes(uint32_t xx1, uint32_t yy1, uint32_t xx2, uint32_t yy2, uint32_t class, float prob)
{
	
	
	if(prob>0)//have bbox
	{	
		hook_1=300;//add a delay or the bbox will not be show 
		x1=xx1>=320?319:xx1;
		x2=xx2>=320?319:xx2;
		y1=yy1>=240?239:yy1;
		y2=yy2>=240?239:yy2;
	}
	if(hook_1--<-255)//no bbox
	{
		hook_1=0;
		x1=0;
		x2=0;
		y1=0;
		y2=0;
	}
}


int main(void)
{
    int ov2640_addr;
    int a;
    uint8_t *aibuffer;
    
    float *output;
    size_t output_size;
    /* Set CPU and dvp clk */
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
    uarths_init();

    io_mux_init();
    io_set_power();
    plic_init();

    lable_init();

    /* LCD init */
    printf("LCD init\n");
    //lcd_init();
    lcd_init(0, 3, 6, 7, 20000000, 37,  38, 3);
    /////////////////
    dvpInit(12000000);
    ov2640_addr=cambus_scan();
    Sipeed_OV2640_begin(ov2640_addr, 24000000);
    Sipeed_OV2640_run(1);
    aibuffer = get_k210_aiBuffer();
    framebuffer = get_k210_dataBuffer();
    ////////////////

    lcd_set_direction(DIR_YX_RLDU);

    /* enable global interrupt */
    sysctl_enable_irq();

    /* system start */
    printf("system start\n");

    /* init kpu */
    if (kpu_load_kmodel(&task, model_data) != 0)
    {
        printf("\nmodel init error\n");
        while (1);
    }
    printf("kmodel init success \n");
    detect_rl.anchor_number = ANCHOR_NUM;
    detect_rl.anchor = g_anchor;
    detect_rl.threshold = 0.5;
    detect_rl.nms_value = 0.2;
    region_layer_init(&detect_rl, 10, 7, 125, 320, 240);
    Sipeed_OV2640_sensor_snapshot();
    //prepare
    g_ai_done_flag = 0;
    Sipeed_OV2640_sensor_snapshot();
    kpu_run_kmodel(&task, aibuffer, DMAC_CHANNEL5, ai_done, NULL);
    while(!g_ai_done_flag);
    kpu_get_output(&task, 0, &detect_rl.input, &output_size);
    register_core1(core1_function, NULL);
    
    //
    while(1)
    {
        g_ai_done_flag = 0;
        /* start to calculate */
        kpu_run_kmodel(&task, aibuffer, DMAC_CHANNEL5, ai_done, NULL);
		Sipeed_OV2640_sensor_snapshot();
		region_layer_draw_boxes(&detect_rl, drawboxes);
		region_layer_run(&detect_rl, NULL);
        while(!g_ai_done_flag);
        kpu_get_output(&task, 0, &output, &output_size);
        detect_rl.input = output;

    }

    
}

