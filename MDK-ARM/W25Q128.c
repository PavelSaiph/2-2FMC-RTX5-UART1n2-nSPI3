#include "main.h"
#include "stm32h7xx.h"                  // Device header
#include "stm32h7xx_hal_gpio.h"
#include "spi.h"

#include "W25Q128.h"

/*功能：读取设备ID
变量：pData，指针类型变量，内存需要是3个字节
分别存放Manufacturer ID、MemType ID和Capacity ID
*/
void W25Q128_Read_ID(uint8_t* pData)
{
	uint8_t cmd[]={Read_JEDEC_ID};
	
	SPI2_Start();
	if(HAL_SPI_Transmit(&hspi2,cmd,1,W25Q128_TIMEOUT_VALUE)==HAL_OK)
	{
		HAL_SPI_Receive(&hspi2,pData,3,W25Q128_TIMEOUT_VALUE);
	}
	SPI2_Stop();
}

/*
功能：软复位
*/
void W25Q128_Reset(void)
{
	uint8_t cmd[2] = {Enable_Reset,Reset_Device};
	
	SPI2_Start();
	HAL_SPI_Transmit(&hspi2,cmd,1,W25Q128_TIMEOUT_VALUE);
	SPI2_Stop();
	SPI2_Start();
	HAL_SPI_Transmit(&hspi2,cmd+1,1,W25Q128_TIMEOUT_VALUE);
	SPI2_Stop();
}

/*
功能：读取状态寄存器1的值并返回
描述：返回的数据位8位，最低位是busy标志
*/
uint8_t W25Q128_ReadSR1(void)   
{  
    uint8_t cmd = Read_Status_Reg1; 
    uint8_t status;  
  
    SPI2_Start();  
    HAL_SPI_Transmit(&hspi2, &cmd, 1, W25Q128_TIMEOUT_VALUE); // 发送指令 05h 
    HAL_SPI_Receive(&hspi2, &status, 1, W25Q128_TIMEOUT_VALUE); // 接收返回的状态值
    SPI2_Stop();  
    
    return status;  
}  

/*
功能：等待SPI的BUSY状态消失
*/
void W25Q128_Wait_Busy(void)   
{   
    // 不断读取，直到最低位 (BUSY) 变为 0
    while ((W25Q128_ReadSR1() & 0x01) == 0x01);   
}

/*
功能：标准SPI读取
变量：@pData：存放Size个字节的空间的指针变量
		@ReadAddr：想要读取的24位的地址,高位默认补0
		@Size：读取多少个字节的数据
*/
void W25Q128_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
	uint8_t cmd[4];
	
	cmd[0]=Read_Data;
	cmd[1]=(uint8_t)(ReadAddr >> 16);
	cmd[2]=(uint8_t)(ReadAddr >> 8);
	cmd[3]=(uint8_t) ReadAddr;
	
	SPI2_Start();
	HAL_SPI_Transmit(&hspi2,cmd,4,W25Q128_TIMEOUT_VALUE);
	HAL_SPI_Receive(&hspi2, pData, Size, W25Q128_TIMEOUT_VALUE);
	SPI2_Stop();
}

/*
功能：写使能
描述：修改数据前必须先进行写使能
*/
void W25Q128_WriteEnable(void)
{
	uint8_t cmd[]={Write_Enable};
	SPI2_Start();
	HAL_SPI_Transmit(&hspi2,cmd,1,W25Q128_TIMEOUT_VALUE);
	SPI2_Stop();
}

/*
功能：页编辑（写入）
描述：已经包含写使能，可以实现页内的写入，但不能跨页写
参数：	@pData:存放需要写入的数据的指针变量
		@WriteAddr：需要写入寄存器的首地址（24位）
		@Size：需要写入的数据的字节总数，1-256
*/
void W25Q128_PageProgram(uint8_t* pData, uint32_t WriteAddr,uint32_t Size)
{
	W25Q128_WriteEnable();
	
	uint8_t cmd[4];
	cmd[0]=Page_Program;
	cmd[1] = (uint8_t)(WriteAddr>>16);
	cmd[2] = (uint8_t)(WriteAddr>>8);
	cmd[3] = (uint8_t)(WriteAddr);
	
	SPI2_Start();
	HAL_SPI_Transmit(&hspi2,cmd,4,W25Q128_TIMEOUT_VALUE);
	HAL_SPI_Transmit(&hspi2,pData,Size,W25Q128_TIMEOUT_VALUE);
	SPI2_Stop();
	
	W25Q128_Wait_Busy();
}

/*
功能：扇区擦除（4kB）
参数：	@Addr：24位地址
*/
void W25Q128_SectorErase(uint32_t Addr)
{
	W25Q128_WriteEnable();
	
	uint8_t cmd[4];
	cmd[0]=Sector_Erase;
	cmd[1]=(uint8_t)(Addr>>16);
	cmd[2]=(uint8_t)(Addr>>8);
	cmd[3]=(uint8_t)(Addr);
	
	SPI2_Start();
	HAL_SPI_Transmit(&hspi2,cmd,4,W25Q128_TIMEOUT_VALUE);
	SPI2_Stop();
	
	W25Q128_Wait_Busy();
}

/*
功能：整片擦除
*/
void W25Q128_ChipErase(void)
{
    W25Q128_WriteEnable();
    
    uint8_t cmd = Chip_Erase;
    SPI2_Start();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, W25Q128_TIMEOUT_VALUE);
    SPI2_Stop();
    
    W25Q128_Wait_Busy(); // 注意：全片擦除可能需要几十秒！
}
