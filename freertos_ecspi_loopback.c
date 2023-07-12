/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_ecspi.h"
#include "fsl_ecspi_freertos.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "lvgl-src/lvgl.h"
#include "lv_drivers-src/display/ILI9341.h"
#include "fsl_gpio.h"      
//#include "lvgl-src/demos/lv_demos.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ECSPI_TRANSFER_SIZE     64
#define ECSPI_TRANSFER_BAUDRATE 12000000U
#define ECSPI_MASTER_BASEADDR   ECSPI1
#define ECSPI_MASTER_CLK_FREQ                                                                 \
    (CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / \
     (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1)))
#define ECSPI_MASTER_TRANSFER_CHANNEL kECSPI_Channel0
#define EXAMPLE_ECSPI_MASTER_IRQN     ECSPI1_IRQn

#define GPIO_PAD        GPIO1
#define LCD_CMD_DATA    0U
#define LCD_RESET       1U
// #define SPI_CS_LOW      GPIO_PinWrite(GPIO5, 9U, 0)
// #define SPI_CS_HIGH     GPIO_PinWrite(GPIO5, 9U, 1)



/* Task priorities. */
#define LVGL_DEMO_PRIORITY          (configMAX_PRIORITIES - 3)
#define LV_TASK_HANDLER_PRIORITY    (configMAX_PRIORITIES - 4)
#define INIT_TASK_PRIORITY          (configMAX_PRIORITIES - 2)
#define SPI_TRANSFER_TASK_PRIORITY  (configMAX_PRIORITIES - 1)

#define MAX_LOG_LENGTH 1
#define MAX_QUEUE_ITEMS 128




/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ecspi_task(void *pvParameters);
static void draw_lvgl_ui(void *pvParameters);
static void init_task(void *pvParameters);
//static void lv_100ask_task_handler(void *pvParameters);
//static void draw_lvgl_ui(void *pvParameters);
static void spi_init(void);                                                              
static void lv_task_hander_task(void *pvParameters);


uint32_t masterTxData[ECSPI_TRANSFER_SIZE] = {0};
static ecspi_master_config_t masterConfig;
static ecspi_transfer_t masterXfer;
static ecspi_rtos_handle_t master_rtos_handle;

/* static lv_disp_draw_buf_t disp_buf1;
static lv_disp_drv_t disp_drv;
static lv_color_t buf1_1[ILI9341_HOR_RES * 10];
static lv_color_t buf1_2[ILI9341_HOR_RES * 10]; */

gpio_pin_config_t lcd_cmd_data_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
gpio_pin_config_t lcd_reset_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
// gpio_pin_config_t lcd_cs_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

//static SemaphoreHandle_t xSemaphore = NULL;

extern void lv_diplay_cmd_data(uint8_t val);
extern void lv_diplay_reset(uint8_t val);
extern void spi_transaction_one_byte(uint8_t data);
extern void spi_transaction_array(uint8_t *data, uint16_t len);


static QueueHandle_t spi_queue = NULL;

volatile uint32_t ulIdleCycleCount = 0UL;
TaskHandle_t xUITaskHandle = NULL;
TaskHandle_t xLVTaskHandle = NULL;


/*******************************************************************************
 * Code
 ******************************************************************************/




void lv_diplay_cmd_data(uint8_t val)
{
    GPIO_PinWrite(GPIO_PAD, LCD_CMD_DATA, val);
    //vTaskDelay( 1 / portTICK_PERIOD_MS );
}


void lv_diplay_reset(uint8_t val)
{
    GPIO_PinWrite(GPIO_PAD, LCD_RESET, val);
}

void LVGL_DELAY_MS(uint8_t ms)
{
     vTaskDelay( ms / portTICK_PERIOD_MS );

}

#if 0
void spi_transaction_one_byte(uint8_t data)
{
/*     BaseType_t xStatus;
    int32_t lValueToSend;

    spi_transfer.buffer[0] = data;
    spi_transfer.length = 1;
    lValueToSend = 1L;

    xStatus = xQueueSendToBack(spi_transfer_queue, &lValueToSend, 0);
    if( xStatus != pdPASS )
    {

        PRINTF( "Could not send byte to the queue.\r\n" );
    }

    PRINTF("spi_transaction_one_byte was called \r\n"); */

    status_t status;

    masterTxData[0] = (uint32_t) data;
    masterXfer.dataSize = 1;
    PRINTF("%x ", masterTxData[0]);
    PRINTF("\r\n");


    // SPI_CS_LOW;
    status = ECSPI_RTOS_Transfer_Blocking(&master_rtos_handle, &masterXfer);
    // SPI_CS_HIGH;
    if (status != kStatus_Success)
    {
        PRINTF("ECSPI transfer completed with error. \r\n\r\n");
    }

}
#endif



void spi_transaction_one_byte(uint8_t data)
{
    BaseType_t xStatus;
    uint32_t data_to_queue;

    data_to_queue = (uint32_t)data;

    xStatus = xQueueSend(spi_queue, &data_to_queue, portMAX_DELAY);
    if( xStatus != pdPASS )
    {
        PRINTF( "Could not send to the queue.\r\n" );
    }

}


#if 0
void spi_transaction_array(uint8_t *data, uint16_t len)
{
/*     BaseType_t xStatus;
    int32_t lValueToSend;

    memcpy(spi_transfer.buffer, data, len);
    spi_transfer.length = len;

    lValueToSend = 1L;
    xStatus = xQueueSendToBack(spi_transfer_queue, &lValueToSend, 0);
    if( xStatus != pdPASS )
    {

        PRINTF( "Could not send array to the queue.\r\n" );
    }

    PRINTF("spi_transaction_array was called \r\n"); */


    status_t status;
    uint32_t i, j, k;

    
    if(len <= 64 )
    {

        for(i = 0; i < len; i++)
        {
            masterTxData[i] = *(data+i);
            PRINTF("%x ", masterTxData[i]);
        }
        PRINTF("\r\n");

        masterXfer.dataSize = len;
        // SPI_CS_LOW;
        status = ECSPI_RTOS_Transfer_Blocking(&master_rtos_handle, &masterXfer);
        if (status != kStatus_Success)
        {
            PRINTF("ECSPI transfer completed with error.\r\n");
        }
        //PRINTF("SPI TRANSFER: %d bytes\r\n",len);

    }
    else
    {
        j = len/64;
        k = 0;
        while (j > 0)
        {
            for(i = 0; i < 64; i++)
            {
                masterTxData[i] = *(data+(i+(k*64)));
                // if(0 == i%2)
                // {
                //     PRINTF("%04x ", ((masterTxData[i+1]<<8) | masterTxData[i]));
                // }
                // if(i == 31)
                // {
                //     PRINTF("\r\n");
                // }
            }
            // PRINTF("\r\n");
            masterXfer.dataSize = 64;
            // SPI_CS_LOW;
            status = ECSPI_RTOS_Transfer_Blocking(&master_rtos_handle, &masterXfer);
            if (status != kStatus_Success)
            {
                PRINTF("ECSPI transfer completed with error. \r\n\r\n");
            }
            k++;
            j--;
            // PRINTF("SPI TRANSFER: 64 bytes, %d / %d, total %d\r\n", k, len/64, len);

        }

        if (len%64)
        {
            for(i = 0; i < len%64; i++)
            {
                masterTxData[i] =  *(data+(i+(k*64)));
                // if(0 == i%2)
                // {
                //     PRINTF("%04x ", ((masterTxData[i+1]<<8) | masterTxData[i]));
                // }
                // if(i == 31)
                // {
                //     PRINTF("\r\n");
                // }
            }
            // PRINTF("\r\n");
            masterXfer.dataSize = len%64;
            // PRINTF("spi_transaction_array reset of 64 was called, %d, k=%d \r\n", len%64, k);
            status = ECSPI_RTOS_Transfer_Blocking(&master_rtos_handle, &masterXfer);
            if (status != kStatus_Success)
            {
                PRINTF("ECSPI transfer completed with error. \r\n\r\n");
            }
            //PRINTF("SPI TRANSFER: last %d bytes\r\n", len%64);

        }
    }

    // SPI_CS_HIGH;

}
#endif



void spi_transaction_array(uint8_t *data, uint16_t len)
{
    BaseType_t xStatus;
    uint32_t i;
    uint32_t data_to_queue;

    for(i = 0;i < len; i++)
    {
        data_to_queue = *(data+i);
        xStatus = xQueueSend(spi_queue, &data_to_queue, portMAX_DELAY);
        if( xStatus != pdPASS )
        {
            PRINTF( "Could not send to the queue from array call.\r\n" );
        }

    }


}



static void hal_init(void)
{
    static lv_disp_draw_buf_t draw_buf_dsc_2;
    static lv_color_t buf_2_1[ILI9341_HOR_RES * 10];                        /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[ILI9341_HOR_RES * 10];                        /*An other buffer for 10 rows*/
    lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, ILI9341_HOR_RES * 10);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = ILI9341_HOR_RES;
    disp_drv.ver_res = ILI9341_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = ili9341_flush;


    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_2;

    /*Required for Example 3)*/
    //disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

    lv_theme_t * th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
    lv_disp_set_theme(disp, th);

    lv_group_t * g = lv_group_create();
    lv_group_set_default(g);
  
  
/*   lv_disp_draw_buf_init(&disp_buf1, buf1_1, buf1_2, ILI9341_HOR_RES * 10);

 
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &disp_buf1;
  disp_drv.flush_cb = ili9341_flush;
  disp_drv.hor_res = ILI9341_HOR_RES;
  disp_drv.ver_res = ILI9341_VER_RES;
  disp_drv.antialiasing = 1;

  lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

  lv_theme_t * th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
  lv_disp_set_theme(disp, th);

  lv_group_t * g = lv_group_create();
  lv_group_set_default(g); */




}



/*!
 * @brief tick hook is executed every tick.
 */
void vApplicationTickHook(void)
{
    static uint32_t ulCount = 0;

    /* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
    1 in FreeRTOSConfig.h. 
    configTICK_RATE_HZ is 1000
    */
    ulCount++;
    if (ulCount >= 2UL)
    {
        lv_tick_inc(2);   //calling every 2 milliseconds.
        ulCount = 0UL;

    }

}

static void set_temp(void * bar, int32_t temp)
{
    lv_bar_set_value(bar, temp, LV_ANIM_ON);
}


void lv_example_get_started_1(void)
{
     
    // /*Change the active screen's background color*/
    // lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    // /*Create a white label, set its text and align it to the center*/
    // lv_obj_t * label = lv_label_create(lv_scr_act());
    // lv_label_set_text(label, "LVGL on Verdin iMX8MP M7");
    // lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
    // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);



    static lv_style_t style_indic;

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);

    lv_obj_t * bar = lv_bar_create(lv_scr_act());
    lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR);
    lv_obj_set_size(bar, 20, 200);
    lv_obj_center(bar);
    lv_bar_set_range(bar, -20, 40);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_temp);
    lv_anim_set_time(&a, 3000);
    lv_anim_set_playback_time(&a, 3000);
    lv_anim_set_var(&a, bar);
    lv_anim_set_values(&a, -20, 40);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);


}



void lv_task_hander_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 5 );
    bool flag = false;
	
	xLastWakeTime = xTaskGetTickCount();  
	
	for(;;)
	{		
        lv_task_handler();
        if(!flag)
        {
            PRINTF("lv_task_hander is called for the first time.\r\n");
            flag = true;
        }
            

		vTaskDelayUntil( &xLastWakeTime,xPeriod );
		
	}
	vTaskDelete(NULL);

}






void init_task(void *pvParameters)
{
    vTaskSuspend(xUITaskHandle);  //suspend ui task untill init task finisded.
    vTaskSuspend(xLVTaskHandle);
    
    spi_init();
    ili9341_init();
    lv_init();
    hal_init();
    PRINTF("Init finised. resume xUI and XLV tasks\r\n");
    vTaskResume(xUITaskHandle);
    vTaskResume(xLVTaskHandle);
    //vTaskDelete( NULL );
    vTaskSuspend(NULL);

}


void draw_lvgl_ui(void *pvParameters)
{
    lv_example_get_started_1();
    PRINTF("UI task finish.\r\n");
    //vTaskDelete( NULL );
    vTaskSuspend(NULL);
}




/***/
void vApplicationIdleHook( void )
{


    ulIdleCycleCount++;
    if( ulIdleCycleCount == 20000000UL)
    {
        PRINTF("idle task heart beat.\r\n");
        ulIdleCycleCount = 0UL;
    }
        
    
}

/* void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                               char * pcTaskName )
{
    PRINTF("OOPS, STACK OVERFLOW\r\n");
} */


void spi_init(void)
{
    status_t status;
    
    ECSPI_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps   = ECSPI_TRANSFER_BAUDRATE;
    // masterConfig.burstLength = 8UL;
    // masterConfig.channelConfig.polarity = 0U;
    // masterConfig.channelConfig.phase = 0U;
    // masterConfig.channelConfig.chipSlectActiveState = kECSPI_ChipSelectActiveStateLow;

    // status = ECSPI_RTOS_Init_Blocking(&master_rtos_handle, ECSPI_MASTER_BASEADDR, &masterConfig, ECSPI_MASTER_CLK_FREQ);

    // if (status != kStatus_Success)
    // {
    //     PRINTF("ECSPI meets error during initialization. \r\n");
    // }


    status = ECSPI_RTOS_Init(&master_rtos_handle, ECSPI_MASTER_BASEADDR, &masterConfig, ECSPI_MASTER_CLK_FREQ);

    if (status != kStatus_Success)
    {
        PRINTF("ECSPI meets error during initialization. \r\n");
        vTaskSuspend(NULL);
    }


    masterXfer.txData   = masterTxData;
    masterXfer.rxData   = NULL;
    //masterXfer.dataSize = ECSPI_TRANSFER_SIZE;
    masterXfer.channel  = ECSPI_MASTER_TRANSFER_CHANNEL;


}


/* void my_log_cb(lv_log_level_t level, const char * buf)
{
  PRINTF("LV LOG: %s\r\n", buf);
} */




/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    /* M7 has its local cache and enabled by default,
     * need to set smart subsystems (0x28000000 ~ 0x3FFFFFFF)
     * non-cacheable before accessing this address region */


    BOARD_InitMemory();

    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();


    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI1 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */
    /* Set IRQ priority for freertos_ecspi */
    NVIC_SetPriority(EXAMPLE_ECSPI_MASTER_IRQN, 2);

    PRINTF("\r\n***Verdin iMX8MP LVGL Demo***\r\n");

    GPIO_PinInit(GPIO_PAD, LCD_CMD_DATA, &lcd_cmd_data_pin);
    GPIO_PinInit(GPIO_PAD, LCD_RESET, &lcd_reset_pin);
    // GPIO_PinInit(GPIO5, 9U, &lcd_cs_pin);

    GPIO_PinWrite(GPIO_PAD, LCD_CMD_DATA, 1);
    GPIO_PinWrite(GPIO_PAD, LCD_RESET, 1);

    spi_queue = xQueueCreate(MAX_QUEUE_ITEMS, MAX_LOG_LENGTH);



    if (xTaskCreate(draw_lvgl_ui, "lvgl_demo_task", 2000, NULL, LVGL_DEMO_PRIORITY, &xUITaskHandle) !=
        pdPASS)
    {
        PRINTF("LVGL DEMO Task creation failed!.\r\n");
        while (1)
            ;
    }
    else
    {
        PRINTF("lvgl ui task created\r\n");
    }

    if (xTaskCreate(lv_task_hander_task, "lvg_task_hander_task", 2000, NULL, LV_TASK_HANDLER_PRIORITY, &xLVTaskHandle) !=
        pdPASS)
    {
        PRINTF("LV TASK HANDER Task creation failed!.\r\n");
        while (1)
            ;
    }
    else
    {
        PRINTF("lv task hander task created\r\n");
    }


    if (xTaskCreate(init_task, "init_task", 2000, NULL, INIT_TASK_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("INIT Task creation failed!.\r\n");
        while (1)
            ;
    }
    else
    {
        PRINTF("init task created\r\n");
    }

    if (xTaskCreate(ecspi_task, "ecspi_task", 2000, NULL, SPI_TRANSFER_TASK_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("ecspi Task creation failed!.\r\n");
        while (1)
            ;
    }
    else
    {
        PRINTF("ecspi task created\r\n");
    }


    vTaskStartScheduler();

    for(;;)
	{		
	}
    


}



/*!
 * @brief Task responsible for ecspi.
 */
static void ecspi_task(void *pvParameters)
{
    uint8_t data = 255;
    uint32_t count = 0;
    uint32_t left_count = 0;
    status_t status;


    while(1)
    {
        while(xQueueReceive(spi_queue, &data, portMAX_DELAY) == pdPASS)
        {
            masterTxData[count] = data;
            count++;
            //PRINTF("read queue-> %d: 0x%x\r\n",count, data);

            left_count = uxQueueMessagesWaiting( spi_queue );
            while(left_count)
            {
                xQueueReceive(spi_queue, &data, 0);
                masterTxData[count] = data;
                left_count--;
                count++;
                //PRINTF("read queue (left)-> %d: %d\r\n",count, data);
            }

            if(count)
            {
                break;
            }

        }

        masterXfer.dataSize = count;
        status = ECSPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);
        if (status != kStatus_Success)
        {
            PRINTF("ECSPI transfer completed with error. \r\n\r\n");
            vTaskSuspend(NULL);
        }
        //PRINTF("%d bytes sent.\r\n", count);
        count = 0;

    }

    /* Deinit the ECSPI. */
    ECSPI_RTOS_Deinit(&master_rtos_handle);
    vTaskSuspend(NULL);
}

