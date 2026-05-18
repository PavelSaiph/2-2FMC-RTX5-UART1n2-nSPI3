#include "main.h"
#include "stm32h7xx.h"                  // Device header
#include "spi.h"

#include "spi3test.h"

uint16_t Modbus_CRC16(uint8_t *pucFrame, uint16_t usLen);

#define SPI_RX_MAX_SIZE 256
uint8_t spi_rx_buf[SPI_RX_MAX_SIZE]; // 接收
uint8_t spi_tx_buf[256];             // 发送
uint16_t RegAddr=0xFFFF;
uint16_t RegNum=0x0000;
uint16_t RegWriteVal=0x0000;

#define MAX_REG_ADDR 10000
extern uint16_t Modbus_RegMap[MAX_REG_ADDR];
uint8_t spi3_re_flag=0;
uint8_t spi3_tx_flag=0;

void SPI3_Receive(void)
{
	spi3_re_flag=1;
	__HAL_SPI_CLEAR_OVRFLAG(&hspi3);
	HAL_SPI_Receive_DMA(&hspi3, spi_rx_buf, SPI_RX_MAX_SIZE);
}

void Parse_Modbus_Frame(uint8_t rx_len)
{
	uint8_t rx_crc_low  = spi_rx_buf[rx_len - 2];
	uint8_t rx_crc_high = spi_rx_buf[rx_len - 1];
	uint16_t received_crc = (rx_crc_high << 8) | rx_crc_low; // 拼成 16 位整数
	
	uint16_t calculated_crc = Modbus_CRC16(spi_rx_buf, rx_len - 2);
		
	if(calculated_crc != received_crc)
	{
		spi3_tx_flag=1;
		return;
	}
	else
	{
		if(spi_rx_buf[0]!=0x01)
		{
			spi3_tx_flag=1;
			return;
		}
		else
		{
			uint8_t cmd=spi_rx_buf[1];
			switch(cmd)
			{
				// ==========================================
				// 03 功能码：读多个寄存器
				// ==========================================
				case 0x03:
				{
					RegAddr=(spi_rx_buf[2]<<8) | (spi_rx_buf[3]);
					RegNum= (spi_rx_buf[4]<<8) | (spi_rx_buf[5]);
					
					if(RegNum==0 || RegNum>125)
					{
						break;
					}
					if((RegAddr + RegNum) > MAX_REG_ADDR)
					{
						break; 
					}
					spi_tx_buf[0]=0x01;
					spi_tx_buf[1]=0x03;
					spi_tx_buf[2]=2*RegNum;
					
					uint16_t tx_index = 3;
					for(uint16_t i=0; i<RegNum; i++)
					{
						uint16_t reg_value = Modbus_RegMap[RegAddr + i];
						
						spi_tx_buf[tx_index++] = (reg_value >> 8) & 0xFF; 
						spi_tx_buf[tx_index++] = reg_value & 0xFF;
					}
					uint16_t crc_val = Modbus_CRC16(spi_tx_buf, tx_index);
					spi_tx_buf[tx_index++] = crc_val & 0xFF;
					spi_tx_buf[tx_index++] = (crc_val >> 8) & 0xFF;
					
					hspi3.Instance->CR1 &= ~SPI_CR1_SPE;
					hspi3.Instance->CR2 = tx_index;
					SPI3->CFG2=0;
					hspi3.Instance->CR1 |= SPI_CR1_SPE;
					
					HAL_SPI_Transmit_DMA(&hspi3, spi_tx_buf, tx_index);
					spi3_tx_flag=1;
					break;
				}
				// ==========================================
				// 06 功能码：写单个寄存器
				// ==========================================
				case 0x06:
				{
					RegAddr=(spi_rx_buf[2]<<8) | (spi_rx_buf[3]);
					RegWriteVal=(spi_rx_buf[4]<<8) | (spi_rx_buf[5]);
					
					if(RegAddr >= MAX_REG_ADDR) break;
					
					Modbus_RegMap[RegAddr]=RegWriteVal;
					
					for(uint16_t i=0;i<8;i++)
					{
						spi_tx_buf[i]=spi_rx_buf[i];
					}	
					
					hspi3.Instance->CR1 &= ~SPI_CR1_SPE;
					hspi3.Instance->CR2 = 8;
					SPI3->CFG2=0;
					hspi3.Instance->CR1 |= SPI_CR1_SPE;

					HAL_SPI_Transmit_DMA(&hspi3, spi_tx_buf, 8);
					spi3_tx_flag=1;
//					HAL_SPI_Transmit(&hspi3,spi_tx_buf,8,10000);

					break;
				}
				// ==========================================
				// 10 (十进制16) 功能码：写多个寄存器
				// ==========================================
				case 0x10:
				{
					RegAddr=(spi_rx_buf[2]<<8) | (spi_rx_buf[3]);
					RegNum= (spi_rx_buf[4]<<8) | (spi_rx_buf[5]);
					
					if(RegNum == 0 || RegNum > 123) break;
					if((RegAddr + RegNum) > MAX_REG_ADDR) break;
					
					for(uint16_t i=0;i<RegNum;i++)
					{
						Modbus_RegMap[RegAddr+i]=spi_rx_buf[7+i*2]<<8 | spi_rx_buf[8+i*2];
					}
					spi_tx_buf[0]=0x01;
					spi_tx_buf[1]=0x10;
					spi_tx_buf[2]=spi_rx_buf[2];
					spi_tx_buf[3]=spi_rx_buf[3];
					spi_tx_buf[4]=spi_rx_buf[4];
					spi_tx_buf[5]=spi_rx_buf[5];
					uint16_t crc_val = Modbus_CRC16(spi_tx_buf, 6);
					spi_tx_buf[6]=crc_val & 0xFF;
					spi_tx_buf[7]=(crc_val >> 8) & 0xFF;
					
					hspi3.Instance->CR1 &= ~SPI_CR1_SPE;
					hspi3.Instance->CR2 = 8;
					SPI3->CFG2=0;
					hspi3.Instance->CR1 |= SPI_CR1_SPE;

					HAL_SPI_Transmit_DMA(&hspi3, spi_tx_buf, 8);
					spi3_tx_flag=1;
					break;
				}
			}
		}
	}
	
}

uint16_t Modbus_CRC16(uint8_t *pucFrame, uint16_t usLen)
{
	uint16_t crc = 0xFFFF; // 1. 预置 16 位寄存器为全 1
    uint8_t i = 0;

    while (usLen--) // 遍历每一个字节
    {
        crc ^= *pucFrame++; // 2. 把当前字节与 16 位 CRC 寄存器的低 8 位进行异或

        for (i = 0; i < 8; i++) // 3. 对该字节的 8 个 bit 进行处理
        {
            if (crc & 0x0001) // 4. 如果最低位是 1
            {
                crc >>= 1;    // 寄存器右移 1 位
                crc ^= 0xA001; // 与 Modbus 多项式 0xA001 进行异或
            }
            else // 如果最低位是 0
            {
                crc >>= 1;    // 只右移 1 位，不异或
            }
        }
    }
    
    return crc; // 返回最终计算结果
}
