/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   RTX5 任务实现
  ******************************************************************************
  */

#include "app_tasks.h"
#include "main.h"
#include "spi.h"
#include "modbus_slave.h"
#include "spi3test.h"
#include "vofa.h"
#include <math.h>
#include "FMCREG.h"

uint16_t *p_FMC_Read=REG_FMC;
uint16_t *p_FMC_Latched=REG_FMC_UART1;

/* ========================================================================
 *                    全局句柄
 * ======================================================================== */
osThreadId_t    tid_mb_task;
osThreadId_t    tid_spi3_task;
osThreadId_t    tid_app_task;

osSemaphoreId_t sem_mb_frame;
osSemaphoreId_t sem_spi3_cs_up;
osMutexId_t     mutex_regmap;

/* ========================================================================
 *                    任务主体
 * ======================================================================== */

/* ---------- Modbus UART 后台任务 ---------- */
extern void Modbus_Poll(void);

static __NO_RETURN void mb_task_fn(void *arg)
{
    (void)arg;
    for (;;)
    {
        /* 等 TIM6 判帧结束发信号 */
        osStatus_t wait_status = osSemaphoreAcquire(sem_mb_frame, osWaitForever);

		if (wait_status == osOK)
		{
			/* 处理一帧（Modbus_Poll 内部会自己决定解析/应答/丢弃） */
			osMutexAcquire(mutex_regmap, osWaitForever);
			Modbus_Poll();
			osMutexRelease(mutex_regmap);
		}
		else if(wait_status == osErrorTimeout)
		{
			osMutexAcquire(mutex_regmap, osWaitForever);
			
			p_FMC_Read[ADC_CH0]=FMC_READ(ADC_CH0);
			p_FMC_Read[ADC_CH1]=FMC_READ(ADC_CH1);
			p_FMC_Read[ADC_CH2]=FMC_READ(ADC_CH2);
			p_FMC_Read[ADC_CH3]=FMC_READ(ADC_CH3);
			p_FMC_Read[ADC_CH4]=FMC_READ(ADC_CH4);
			p_FMC_Read[ADC_CH5]=FMC_READ(ADC_CH5);
			p_FMC_Read[ADC_CH6]=FMC_READ(ADC_CH6);
			p_FMC_Read[ADC_CH7]=FMC_READ(ADC_CH7);
			p_FMC_Read[ADC_CH8]=FMC_READ(ADC_CH8);
			p_FMC_Read[ADC_CH9]=FMC_READ(ADC_CH9);
			p_FMC_Read[ADC_CH10]=FMC_READ(ADC_CH10);
			p_FMC_Read[ADC_CH11]=FMC_READ(ADC_CH11);
			p_FMC_Read[ADC_CH12]=FMC_READ(ADC_CH12);
			p_FMC_Read[ADC_CH13]=FMC_READ(ADC_CH13);
			p_FMC_Read[ADC_CH14]=FMC_READ(ADC_CH14);
			p_FMC_Read[ADC_CH15]=FMC_READ(ADC_CH15);
			uint16_t *p_FMC_Temp=p_FMC_Read;
			p_FMC_Read=p_FMC_Latched;
			p_FMC_Latched=p_FMC_Temp;
			
			osMutexRelease(mutex_regmap);
		}
    }
}

/* ---------- SPI3 后台任务 ----------
 * EXTI15_10 里的 SPI3 处理逻辑搬到这里，ISR 只释放信号量
 */
extern SPI_HandleTypeDef hspi3;
extern uint8_t  spi3_re_flag;
extern uint8_t  spi3_tx_flag;

static __NO_RETURN void spi3_task_fn(void *arg)
{
    (void)arg;
    for (;;)
    {
        osSemaphoreAcquire(sem_spi3_cs_up, osWaitForever);

        osMutexAcquire(mutex_regmap, osWaitForever);
        if (spi3_re_flag == 1)
        {
            uint16_t current_ndtr = DMA1_Stream0->NDTR;
            HAL_SPI_Abort(&hspi3);
            spi3_re_flag = 0;
            uint16_t rx_len = 256 - current_ndtr;
            if (rx_len >= 8) Parse_Modbus_Frame(rx_len);
        }
        else if (spi3_tx_flag == 1)
        {
            spi3_tx_flag = 0;
            SPI3_Receive();
        }
        osMutexRelease(mutex_regmap);
    }
}

/* ---------- 应用层：VOFA+ JustFloat 演示 ----------
 * 跑满 2 Mbps 线速：每帧 16 字节 = 80μs 空中 + ~20μs DMA/ISR 开销
 * 实际采样率 ≈ 10 kHz（3 通道时），由 DMA 完成速度决定。
 *
 * 时间基使用采样索引 n / SAMPLE_RATE_HZ，保证不管实际发送多快波形频率都是对的。
 */
#define VOFA_SAMPLE_RATE_HZ   10000.0f
#define VOFA_2PI              (2.0f * 3.14159265358979f)

#define VOFA_CHANNELS		16
/* 缩放系数：根据你的 ADC 分辨率和电路分压比调整 */
#define ADC_TO_PHYSICAL  (3.3f / 4096.0f)
#define ADC_TO_PHYSICAL1	(1.0f/2048.0f)

static __NO_RETURN void app_task_fn(void *arg)
{
//    (void)arg;
//    uint32_t n = 0;
//    for (;;)
//    {
//        float t = (float)n / VOFA_SAMPLE_RATE_HZ;    /* 秒 */
//        float ch[3];
//        ch[0] = sinf(VOFA_2PI * 10.0f  * t);         /* ch1: 10 Hz  */
//        ch[1] = sinf(VOFA_2PI * 100.0f * t);         /* ch2: 100 Hz */
//        ch[2] = sinf(VOFA_2PI * 500.0f * t);         /* ch3: 500 Hz */

//        /* 阻塞版：DMA 发完立刻推下一帧，跑满 USART1 @ 2 Mbps 线速 */
//        Vofa_SendFloats_Wait(ch, 3);

//        n++;
//    }
	
	(void)arg;
    float vofa_data[VOFA_CHANNELS]; 

    for (;;)
    {
        // 1. 极速读取 FPGA 的 ADC 数据
       
//			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
			volatile uint16_t raw_adc[VOFA_CHANNELS];
			FMC_WRITE(FPGA_INIT, 0x000A);
			
			raw_adc[0]  = FMC_READ(ADC_CH0);
			raw_adc[1]  = FMC_READ(ADC_CH1);
			raw_adc[2]  = FMC_READ(ADC_CH2);
			raw_adc[3]  = FMC_READ(ADC_CH3);
			raw_adc[4]  = FMC_READ(ADC_CH4);
			raw_adc[5]  = FMC_READ(ADC_CH5);
			raw_adc[6]  = FMC_READ(ADC_CH6);
			raw_adc[7]  = FMC_READ(ADC_CH7);
			raw_adc[8]  = FMC_READ(ADC_CH8);
			raw_adc[9]  = FMC_READ(ADC_CH9);
			raw_adc[10] = FMC_READ(ADC_CH10);
			raw_adc[11] = FMC_READ(ADC_CH11);
			raw_adc[12] = FMC_READ(ADC_CH12);
			raw_adc[13] = FMC_READ(ADC_CH13);
			raw_adc[14] = FMC_READ(ADC_CH14);
			raw_adc[15] = FMC_READ(ADC_CH15);

			
			
		   /* 2. 数据类型转换：int16 -> float */
			for (int i = 0; i < VOFA_CHANNELS; i++)
			{
				if (i < 4)
				{
					// 提取干净的 12 位原始数据
					int16_t signed_val = raw_adc[i] & 0x0FFF; 
					// 检查第 12 位（0x0800）是否为 1
					if (signed_val & 0x0800) 
					{
						// 如果是负数（比如 0x0800），把高 4 位全部补 1
						// 0x0800 就会变成 0xF800，C 语言就会将其正确识别为 -2048
						signed_val |= 0xF000; 
					}
					// 此时 signed_val 的范围完美映射为 -2048 到 +2047
					vofa_data[i] = (float)signed_val * ADC_TO_PHYSICAL1;
				}
				else
				{
					// 第 5 个通道及之后：处理 12 位无符号数 (0-4095)
					uint16_t unsigned_val = raw_adc[i] & 0x0FFF; // 屏蔽高 4 位
					
					vofa_data[i] = (float)unsigned_val * ADC_TO_PHYSICAL;
				}
			}
			
			vofa_data[8]=vofa_data[2]-vofa_data[3];
			
			/* 3. 阻塞式发送给上位机 (等待 340us 物理传输时间) */
			Vofa_SendFloats_Wait(vofa_data, VOFA_CHANNELS);
		
//			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
//        /* 此处不加 osDelay，让它按串口极限速度跑 */
//		osDelay(500);
    }
	
}

/* ========================================================================
 *                    任务属性
 * ======================================================================== */

static const osThreadAttr_t mb_task_attr = {
    .name       = "mb_task",
    .stack_size = 1024U,
    .priority   = osPriorityNormal,
};

static const osThreadAttr_t spi3_task_attr = {
    .name       = "spi3_task",
    .stack_size = 1024U,
    .priority   = osPriorityAboveNormal,
};

static const osThreadAttr_t app_task_attr = {
    .name       = "app_task",
    .stack_size = 1024U,
    .priority   = osPriorityLow,
};

/* ========================================================================
 *                    初始化入口
 * ======================================================================== */
void App_TasksInit(void)
{
    /* 信号量：二值，初始为 0，上来就阻塞等待 */
    sem_mb_frame   = osSemaphoreNew(1U, 0U, NULL);
    sem_spi3_cs_up = osSemaphoreNew(1U, 0U, NULL);

    /* 保护寄存器表（UART 和 SPI3 都会读写） */
    mutex_regmap = osMutexNew(NULL);

    /* VOFA+ 初始化（内部会创建 TX 完成信号量，需要内核已初始化） */
    Vofa_Init();

    /* 创建任务 */
    tid_mb_task   = osThreadNew(mb_task_fn,   NULL, &mb_task_attr);
    tid_spi3_task = osThreadNew(spi3_task_fn, NULL, &spi3_task_attr);
    tid_app_task  = osThreadNew(app_task_fn,  NULL, &app_task_attr);
}
