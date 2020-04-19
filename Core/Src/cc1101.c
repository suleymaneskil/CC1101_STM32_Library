/*
 * cc1101.c
 *
 *  Created on: Mar 11, 2020
 *      Author: suleyman.eskil but the library has Mr. Ilynx
 *      https://www.freelancer.com/u/ilynx?ref_project_id=24212020
 */

#include"cc1101.h"
#include "dw_stm32_delay.h"

SPI_HandleTypeDef* hal_spi;
UART_HandleTypeDef* hal_uart;

uint16_t CS_Pin;
GPIO_TypeDef* CS_GPIO_Port;

#define WRITE_BURST             0x40
#define READ_SINGLE             0x80
#define READ_BURST              0xC0


#define BYTES_IN_RXFIFO         0x7F
#define LQI                     1
#define CRC_OK                  0x80


#define PKTSTATUS_CCA           0x10
#define PKTSTATUS_CS            0x40


#define RANDOM_OFFSET           67
#define RANDOM_MULTIPLIER       109
#define RSSI_VALID_DELAY_US     1300

//static UINT8 rnd_seed = 0;

HAL_StatusTypeDef __spi_write(uint8_t *addr, uint8_t *pData, uint16_t size){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //set Chip Select to Low
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)); //CS pini LOW yaptığımızd MISO pini adres yazılmadan önce low da beklemeli
	status = HAL_SPI_Transmit(hal_spi, addr, 1, 0xFFFF);
	if(status==HAL_OK && pData!=NULL)
		status = HAL_SPI_Transmit(hal_spi, pData, size, 0xFFFF);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //set Chip Select to High
	return status;

}

HAL_StatusTypeDef __spi_read(uint8_t *addr, uint8_t *pData, uint16_t size){

	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); //set Chip Select to Low

	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)); //CS pini LOW yaptığımızd MISO pini adres yazılmadan önce low da beklemeli
	//HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)

	status = HAL_SPI_Transmit(hal_spi, addr, 1, 0xFFFF);
	status = HAL_SPI_Receive(hal_spi, pData, size, 0xFFFF);

//	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)); //CS pini LOW yaptığımızd MISO pini adres yazılmadan önce low da beklemeli

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); //set Chip Select to High

	return status;

}

void TI_write_reg(UINT8 addr, UINT8 value)
{
	__spi_write(&addr, &value, 1);
}

void TI_write_burst_reg(BYTE addr, BYTE* buffer, BYTE count)
{
	addr = (addr | WRITE_BURST);
	__spi_write(&addr, buffer, count);
}

void TI_strobe(BYTE strobe)
{
	__spi_write(&strobe, 0, 0);
}


BYTE TI_read_reg(BYTE addr)
{
	uint8_t data;
	addr= (addr | READ_SINGLE);
	__spi_read(&addr, &data, 1);
	return data;
}

BYTE TI_read_status(BYTE addr)
{
	uint8_t data;
	addr= (addr | READ_BURST);
	__spi_read(&addr, &data, 1);
	return data;
}

void TI_read_burst_reg(BYTE addr, BYTE* buffer, BYTE count)
{
	addr= (addr | READ_BURST);
	__spi_read(&addr, buffer, count);
}

BOOL TI_receive_packet(BYTE* rxBuffer, UINT8 *length)
{
	BYTE status[2];
	UINT8 packet_len;
	// This status register is safe to read since it will not be updated after
	// the packet has been received (See the CC1100 and 2500 Errata Note)
	if (TI_read_status(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)
	{
		// Read length byte
		packet_len = TI_read_reg(CCxxx0_RXFIFO);

		// Read data from RX FIFO and store in rxBuffer
		if (packet_len <= *length)
		{
			TI_read_burst_reg(CCxxx0_RXFIFO, rxBuffer, packet_len);
			*length = packet_len;

			// Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
			TI_read_burst_reg(CCxxx0_RXFIFO, status, 2);
			//while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			//while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
			// MSB of LQI is the CRC_OK bit
			return(status[LQI] & CRC_OK);
		}
		else
		{
			*length = packet_len;

			// Make sure that the radio is in IDLE state before flushing the FIFO
			// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
			TI_strobe(CCxxx0_SIDLE);

			// Flush RX FIFO
			TI_strobe(CCxxx0_SFRX);
			return(FALSE);
		}
	}
	else return(FALSE);
}

void init_serial(UART_HandleTypeDef* huart){

	hal_uart = huart;
}


void TI_send_packet(BYTE* txBuffer, UINT8 size)
{
	BYTE status;

  	TI_strobe(CCxxx0_SIDLE); //ïåðåâîäèì ìîäåì â IDLE

    TI_write_reg(CCxxx0_TXFIFO, size);

	status = TI_read_status(CCxxx0_TXBYTES);

    TI_write_burst_reg(CCxxx0_TXFIFO, txBuffer, size);

	status = TI_read_status(CCxxx0_TXBYTES);

    TI_strobe(CCxxx0_STX);
}


//For 433MHz, +10dBm
//it is also high
BYTE paTable[] = {0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0};


void TI_write_settings()
{

	// Address Config = No address check
	// Base Frequency = 432.999817
	// CRC Autoflush = false
	// CRC Enable = true
	// Carrier Frequency = 432.999817
	// Channel Number = 0
	// Channel Spacing = 199.951172
	// Data Format = Normal mode
	// Data Rate = 1.19948
	// Deviation = 25.390625
	// Device Address = 0
	// Manchester Enable = false
	// Modulated = true
	// Modulation Format = GFSK
	// PA Ramping = false
	// Packet Length = 20
	// Packet Length Mode = Variable packet length mode. Packet length configured by the first byte after sync word
	// Preamble Count = 4
	// RX Filter BW = 101.562500
	// Sync Word Qualifier Mode = 16/16 + carrier-sense above threshold
	// TX Power = 0
	// Whitening = false
	//
	// Rf settings for CC1101
	//

	//i checked in smartRF studio 7 of Mr. ilynx's code // the setting is yours
	TI_write_reg(CCxxx0_IOCFG2,0x29);  //GDO2 Output Pin Configuration
	TI_write_reg(CCxxx0_IOCFG1,0x2E);  //GDO1 Output Pin Configuration
	TI_write_reg(CCxxx0_IOCFG0,0x06);  //GDO0 Output Pin Configuration
	TI_write_reg(CCxxx0_FIFOTHR,0x47); //RX FIFO and TX FIFO Thresholds
	TI_write_reg(CCxxx0_SYNC1,0xD3);   //Sync Word, High Byte
	TI_write_reg(CCxxx0_SYNC0,0x91);   //Sync Word, Low Byte
	TI_write_reg(CCxxx0_PKTLEN,0xFF);  //Packet Length
	TI_write_reg(CCxxx0_PKTCTRL1,0x04);//Packet Automation Control
	TI_write_reg(CCxxx0_PKTCTRL0,0x05);//Packet Automation Control
	TI_write_reg(CCxxx0_ADDR,0x00);    //Device Address
	TI_write_reg(CCxxx0_CHANNR,0x00);  //Channel Number
	TI_write_reg(CCxxx0_FSCTRL1,0x08); //Frequency Synthesizer Control
	TI_write_reg(CCxxx0_FSCTRL0,0x00); //Frequency Synthesizer Control
	TI_write_reg(CCxxx0_FREQ2,0x10);   //Frequency Control Word, High Byte
	TI_write_reg(CCxxx0_FREQ1,0xB4);   //Frequency Control Word, Middle Byte
	TI_write_reg(CCxxx0_FREQ0,0x2E);   //Frequency Control Word, Low Byte
	TI_write_reg(CCxxx0_MDMCFG4,0xCA); //Modem Configuration
	TI_write_reg(CCxxx0_MDMCFG3,0x83); //Modem Configuration
	TI_write_reg(CCxxx0_MDMCFG2,0x93); //Modem Configuration
	TI_write_reg(CCxxx0_MDMCFG1,0x22); //Modem Configuration
	TI_write_reg(CCxxx0_MDMCFG0,0xF8); //Modem Configuration
	TI_write_reg(CCxxx0_DEVIATN,0x34); //Modem Deviation Setting
	TI_write_reg(CCxxx0_MCSM2,0x07);   //Main Radio Control State Machine Configuration
	TI_write_reg(CCxxx0_MCSM1,0x30);   //Main Radio Control State Machine Configuration
	TI_write_reg(CCxxx0_MCSM0,0x18);   //Main Radio Control State Machine Configuration
	TI_write_reg(CCxxx0_FOCCFG,0x16);  //Frequency Offset Compensation Configuration
	TI_write_reg(CCxxx0_BSCFG,0x6C);   //Bit Synchronization Configuration
	TI_write_reg(CCxxx0_AGCCTRL2,0x43);//AGC Control
	TI_write_reg(CCxxx0_AGCCTRL1,0x40);//AGC Control
	TI_write_reg(CCxxx0_AGCCTRL0,0x91);//AGC Control
	TI_write_reg(CCxxx0_WOREVT1,0x87); //High Byte Event0 Timeout
	TI_write_reg(CCxxx0_WOREVT0,0x6B); //Low Byte Event0 Timeout
	TI_write_reg(CCxxx0_WORCTRL,0xF8); //Wake On Radio Control
	TI_write_reg(CCxxx0_FREND1,0x56);  //Front End RX Configuration
	TI_write_reg(CCxxx0_FREND0,0x10);  //Front End TX Configuration
	TI_write_reg(CCxxx0_FSCAL3,0xE9);  //Frequency Synthesizer Calibration
	TI_write_reg(CCxxx0_FSCAL2,0x2A);  //Frequency Synthesizer Calibration
	TI_write_reg(CCxxx0_FSCAL1,0x00);  //Frequency Synthesizer Calibration
	TI_write_reg(CCxxx0_FSCAL0,0x1F);  //Frequency Synthesizer Calibration
	TI_write_reg(CCxxx0_RCCTRL1,0x41); //RC Oscillator Configuration
	TI_write_reg(CCxxx0_RCCTRL0,0x00); //RC Oscillator Configuration
	TI_write_reg(CCxxx0_FSTEST,0x59);  //Frequency Synthesizer Calibration Control
	TI_write_reg(CCxxx0_PTEST,0x7F);   //Production Test
	TI_write_reg(CCxxx0_AGCTEST,0x3F); //AGC Test
	TI_write_reg(CCxxx0_TEST2,0x81);   //Various Test Settings
	TI_write_reg(CCxxx0_TEST1,0x35);   //Various Test Settings
	TI_write_reg(CCxxx0_TEST0,0x09);   //Various Test Settings

	/*TI_write_reg(CCxxx0_FSCTRL1,  settings->FSCTRL1);
    TI_write_reg(CCxxx0_FSCTRL0,  settings->FSCTRL0);
    TI_write_reg(CCxxx0_FREQ2,    settings->FREQ2);
    TI_write_reg(CCxxx0_FREQ1,    settings->FREQ1);
    TI_write_reg(CCxxx0_FREQ0,    settings->FREQ0);
    TI_write_reg(CCxxx0_MDMCFG4,  settings->MDMCFG4);
    TI_write_reg(CCxxx0_MDMCFG3,  settings->MDMCFG3);
    TI_write_reg(CCxxx0_MDMCFG2,  settings->MDMCFG2);
    TI_write_reg(CCxxx0_MDMCFG1,  settings->MDMCFG1);
    TI_write_reg(CCxxx0_MDMCFG0,  settings->MDMCFG0);
    TI_write_reg(CCxxx0_CHANNR,   settings->CHANNR);
    TI_write_reg(CCxxx0_DEVIATN,  settings->DEVIATN);
    TI_write_reg(CCxxx0_FREND1,   settings->FREND1);
    TI_write_reg(CCxxx0_FREND0,   settings->FREND0);
    TI_write_reg(CCxxx0_MCSM0 ,   settings->MCSM0 );
    TI_write_reg(CCxxx0_FOCCFG,   settings->FOCCFG);
    TI_write_reg(CCxxx0_BSCFG,    settings->BSCFG);
    TI_write_reg(CCxxx0_AGCCTRL2, settings->AGCCTRL2);
  	TI_write_reg(CCxxx0_AGCCTRL1, settings->AGCCTRL1);
    TI_write_reg(CCxxx0_AGCCTRL0, settings->AGCCTRL0);
    TI_write_reg(CCxxx0_FSCAL3,   settings->FSCAL3);
    TI_write_reg(CCxxx0_FSCAL2,   settings->FSCAL2);
  	TI_write_reg(CCxxx0_FSCAL1,   settings->FSCAL1);
    TI_write_reg(CCxxx0_FSCAL0,   settings->FSCAL0);
    TI_write_reg(CCxxx0_FSTEST,   settings->FSTEST);
    TI_write_reg(CCxxx0_TEST2,    settings->TEST2);
    TI_write_reg(CCxxx0_TEST1,    settings->TEST1);
    TI_write_reg(CCxxx0_TEST0,    settings->TEST0);
    TI_write_reg(CCxxx0_FIFOTHR,  settings->FIFOTHR);
    TI_write_reg(CCxxx0_IOCFG2,   settings->IOCFG2);
    TI_write_reg(CCxxx0_IOCFG0,   settings->IOCFG0);
    TI_write_reg(CCxxx0_PKTCTRL1, settings->PKTCTRL1);
    TI_write_reg(CCxxx0_PKTCTRL0, settings->PKTCTRL0);
    TI_write_reg(CCxxx0_ADDR,     settings->ADDR);
    TI_write_reg(CCxxx0_PKTLEN,   settings->PKTLEN);*/
}

void Power_up_reset()
{
	//Güç geldikten sonra CC1101 i Macro resetlemek için

	DWT_Delay_Init();
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	DWT_Delay_us(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	DWT_Delay_us(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	DWT_Delay_us(41);

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)); //CS pini LOW yaptığımızd MISO pini adres yazılmadan önce low da beklemeli
	TI_strobe(CCxxx0_SRES);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}


void TI_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	//UINT8 i;
	//UINT16 delay;
	BYTE status;
	hal_spi = hspi;
	CS_GPIO_Port = cs_port;
	CS_Pin = cs_pin;


	for(int i=0; i<10; i++){
	status = TI_read_status(CCxxx0_VERSION);
		  if(status!=0x14)
		  {
		  }
	}
	TI_strobe(CCxxx0_SFRX); //î÷èùàåì RX FIFO
	TI_strobe(CCxxx0_SFTX); //î÷èùàåì TX FIFO
	TI_write_settings();
	TI_write_burst_reg(CCxxx0_PATABLE, paTable, 8);//is it true

	TI_write_reg(CCxxx0_FIFOTHR, 0x07);

	TI_strobe(CCxxx0_SIDLE); //ïåðåâîäèì ìîäåì â IDLE
	TI_strobe(CCxxx0_SFRX); //î÷èùàåì RX FIFO
	TI_strobe(CCxxx0_SFTX); //î÷èùàåì TX FIFO

	//TI_strobe(CCxxx0_SRX);

	/*delay = RSSI_VALID_DELAY_US;

	do
	{
		status = TI_read_status(CCxxx0_PKTSTATUS) & (PKTSTATUS_CCA | PKTSTATUS_CS);

		if (status)
		{
			break;
		}

		ACC = 16;

		while(--ACC);

		delay -= 64;
	} while(delay > 0);

	for(i = 0; i < 16; i++)
	{
	  rnd_seed = (rnd_seed << 1) | (TI_read_status(CCxxx0_RSSI) & 0x01);
	}

	rnd_seed |= 0x0080;*/

	TI_strobe(CCxxx0_SIDLE);
}
