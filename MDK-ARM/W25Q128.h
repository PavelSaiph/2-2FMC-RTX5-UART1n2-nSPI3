#ifndef __W25Q128_H__
#define __W25Q128_H__


#define SPI2_Start()	HAL_GPIO_WritePin(SPI2_CS_n_GPIO_Port,SPI2_CS_n_Pin,GPIO_PIN_RESET)
#define SPI2_Stop()		HAL_GPIO_WritePin(SPI2_CS_n_GPIO_Port,SPI2_CS_n_Pin,GPIO_PIN_SET)

#define W25Q128_TIMEOUT_VALUE 1000

#define Read_JEDEC_ID	0x9F
#define Enable_Reset	0x66
#define Reset_Device	0x99
#define Read_Data		0x03
#define Write_Enable	0x06
#define Page_Program	0x02
#define Sector_Erase	0x20
#define Read_Status_Reg1  0x05
#define Chip_Erase 		0xC7

/*功能：读取设备ID
变量：pData，指针类型变量，内存需要是3个字节
分别存放Manufacturer ID、MemType ID和Capacity ID
*/
void W25Q128_Read_ID(uint8_t* pData);

/*
功能：软复位
*/
void W25Q128_Reset(void);


/*
功能：读取状态寄存器1的值并返回
描述：返回的数据位8位，最低位是busy标志
*/
uint8_t W25Q128_ReadSR1(void);

/*
功能：等待SPI的BUSY状态消失
*/
void W25Q128_Wait_Busy(void);   
/*
功能：标准SPI读取
变量：@pData：存放Size个字节的空间的指针变量
		@ReadAddr：想要读取的24位的地址,高位默认补0
		@Size：读取多少个字节的数据
*/
void W25Q128_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);

/*
功能：写使能
描述：修改数据前必须先进行写使能
*/
void W25Q128_WriteEnable(void);

/*
功能：页编辑（写入）
描述：已经包含写使能，可以实现页内的写入，但不能跨页写
参数：	@pData:存放需要写入的数据的指针变量
		@WriteAddr：需要写入寄存器的首地址（24位）
		@Size：需要写入的数据的字节总数，1-256
*/
void W25Q128_PageProgram(uint8_t* pData, uint32_t WriteAddr,uint32_t Size);

/*
功能：扇区擦除（4kB）
参数：	@Addr：24位地址
*/
void W25Q128_SectorErase(uint32_t Addr);

/*
功能：整片擦除
*/
void W25Q128_ChipErase(void);

#endif 
