/*
 * CC1101.H
 *
 *  Created on: Mar 11, 2020
 *      Author: suleyman.eskil
 */

#ifndef INC_CC1101_H_
#define INC_CC1101_H_
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef __STM32L4xx_HAL_H
#include "stm32l4xx_hal.h"
#endif

//-------------------------------------------------------------------------------------------------------
typedef unsigned char       BOOL;

// Data
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

// Unsigned numbers
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned long       UINT32;

// Signed numbers
typedef signed char         INT8;
typedef signed short        INT16;
typedef signed long         INT32;

//-------------------------------------------------------------------------------------------------------
// Common values
#ifndef FALSE
    #define FALSE 0
#endif

#ifndef TRUE
    #define TRUE 1
#endif

//------------------------------------------------------------------------------------------------------
// CC2500/CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1        0x04        // Sync word, high byte
#define CCxxx0_SYNC0        0x05        // Sync word, low byte
#define CCxxx0_PKTLEN       0x06        // Packet length
#define CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define CCxxx0_ADDR         0x09        // Device address
#define CCxxx0_CHANNR       0x0A        // Channel number
#define CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define CCxxx0_MDMCFG4      0x10        // Modem configuration
#define CCxxx0_MDMCFG3      0x11        // Modem configuration
#define CCxxx0_MDMCFG2      0x12        // Modem configuration
#define CCxxx0_MDMCFG1      0x13        // Modem configuration
#define CCxxx0_MDMCFG0      0x14        // Modem configuration
#define CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define CCxxx0_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2     0x1B        // AGC control
#define CCxxx0_AGCCTRL1     0x1C        // AGC control
#define CCxxx0_AGCCTRL0     0x1D        // AGC control
#define CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define CCxxx0_FREND1       0x21        // Front end RX configuration
#define CCxxx0_FREND0       0x22        // Front end TX configuration
#define CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define CCxxx0_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCxxx0_PTEST        0x2A        // Production test
#define CCxxx0_AGCTEST      0x2B        // AGC test
#define CCxxx0_TEST2        0x2C        // Various test settings
#define CCxxx0_TEST1        0x2D        // Various test settings
#define CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define CCxxx0_SRES             0x30    // Reset chip.
#define CCxxx0_SFSTXON          0x31    // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF            0x32    // Turn off crystal oscillator.
#define CCxxx0_SCAL             0x33    // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CCxxx0_SRX              0x34    // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX              0x35    // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CCxxx0_SIDLE            0x36    // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CCxxx0_SAFC             0x37    // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR             0x38    // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD             0x39    // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX             0x3A    // Flush the RX FIFO buffer.
#define CCxxx0_SFTX             0x3B    // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST          0x3C    // Reset real time clock.
#define CCxxx0_SNOP             0x3D    // No operation. May be used to pad strobe commands to two
                                        // bytes for simpler software.
// Status registers
#define CCxxx0_PARTNUM          0x30    // Part number
#define CCxxx0_VERSION          0x31    // Current version number
#define CCxxx0_FREQEST          0x32    // Frequency offset estimate
#define CCxxx0_LQI              0x33    // Demodulator estimate for link quality
#define CCxxx0_RSSI             0x34    // Received signal strength indication
#define CCxxx0_MARCSTATE        0x35    // Control state machine state
#define CCxxx0_WORTIME1         0x36    // High byte of WOR timer
#define CCxxx0_WORTIME0         0x37    // Low byte of WOR timer
#define CCxxx0_PKTSTATUS        0x38    // Current GDOx status and packet status
#define CCxxx0_VCO_VC_DAC       0x39    // Current setting from PLL cal module
#define CCxxx0_TXBYTES          0x3A    // Underflow and # of bytes in TXFIFO
#define CCxxx0_RXBYTES          0x3B    // Overflow and # of bytes in RXFIFO
#define CCxxx0_RCCTRL1_STATUS   0x3C
#define CCxxx0_RCCTRL0_STATUS   0x3D
#define TI_CCxxx0_NUM_RXBYTES   0x7F    // Mask "# of bytes" field in _RXBYTES

#define CCxxx0_PATABLE          0x3E
#define CCxxx0_TXFIFO           0x3F
#define CCxxx0_RXFIFO           0x3F

// Masks for appended status bytes
#define TI_CCxxx0_LQI_RX       0x01     // Position of LQI byte
#define TI_CCxxx0_CRC_OK       0x80     // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
#define TI_CCxxx0_WRITE_BURST  0x40
#define TI_CCxxx0_READ_SINGLE  0x80
#define TI_CCxxx0_READ_BURST   0xC0




//-------------------------------------------------------------------------------------------------------
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
//i didnt use because i used with TI_write_settings() in cc1101.c
/*typedef struct S_RF_SETTINGS{
    BYTE FSCTRL1;   // Frequency synthesizer control.
    BYTE FSCTRL0;   // Frequency synthesizer control.
    BYTE FREQ2;     // Frequency control word, high byte.
    BYTE FREQ1;     // Frequency control word, middle byte.
    BYTE FREQ0;     // Frequency control word, low byte.
    BYTE MDMCFG4;   // Modem configuration.
    BYTE MDMCFG3;   // Modem configuration.
    BYTE MDMCFG2;   // Modem configuration.
    BYTE MDMCFG1;   // Modem configuration.
    BYTE MDMCFG0;   // Modem configuration.
    BYTE CHANNR;    // Channel number.
    BYTE DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    BYTE FREND1;    // Front end RX configuration.
    BYTE FREND0;    // Front end RX configuration.
    BYTE MCSM0;     // Main Radio Control State Machine configuration.
    BYTE FOCCFG;    // Frequency Offset Compensation Configuration.
    BYTE BSCFG;     // Bit synchronization Configuration.
    BYTE AGCCTRL2;  // AGC control.
  	BYTE AGCCTRL1;  // AGC control.
    BYTE AGCCTRL0;  // AGC control.
    BYTE FSCAL3;    // Frequency synthesizer calibration.
    BYTE FSCAL2;    // Frequency synthesizer calibration.
    BYTE FSCAL1;    // Frequency synthesizer calibration.
    BYTE FSCAL0;    // Frequency synthesizer calibration.
    BYTE FSTEST;    // Frequency synthesizer calibration control
    BYTE TEST2;     // Various test settings.
    BYTE TEST1;     // Various test settings.
    BYTE TEST0;     // Various test settings.
    BYTE FIFOTHR;   // RXFIFO and TXFIFO thresholds.
    BYTE IOCFG2;    // GDO2 output pin configuration
    BYTE IOCFG0;    // GDO0 output pin configuration
    BYTE PKTCTRL1;  // Packet automation control.
    BYTE PKTCTRL0;  // Packet automation control.
    BYTE ADDR;      // Device address.
    BYTE PKTLEN;    // Packet length.

} RF_SETTINGS;

RF_SETTINGS TISettings = {
     0x08,   // FSCTRL1   Frequency synthesizer control.
     0x00,   // FSCTRL0   Frequency synthesizer control.
     0x10,   // FREQ2     Frequency control word, high byte.
     0xB4,   // FREQ1     Frequency control word, middle byte.
     0x2E,   // FREQ0     Frequency control word, low byte.
     0xCA,   // MDMCFG4   Modem configuration.
     0x83,   // MDMCFG3   Modem configuration.
     0x93,   // MDMCFG2   Modem configuration.
     0x22,   // MDMCFG1   Modem configuration.
     0xF8,   // MDMCFG0   Modem configuration.
     0x00,   // CHANNR    Channel number.
     0x34,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
     0x56,   // FREND1    Front end RX configuration.
     0x10,   // FREND0    Front end TX configuration.
     0x18,   // MCSM0     Main Radio Control State Machine configuration.
     0x16,   // FOCCFG    Frequency Offset Compensation Configuration.
     0x6C,   // BSCFG     Bit synchronization Configuration.
     0x43,   // AGCCTRL2  AGC control.
     0x40,   // AGCCTRL1  AGC control.
     0x91,   // AGCCTRL0  AGC control.
     0xE9,   // FSCAL3    Frequency synthesizer calibration.
     0x2A,   // FSCAL2    Frequency synthesizer calibration.
     0x00,   // FSCAL1    Frequency synthesizer calibration.
     0x1F,   // FSCAL0    Frequency synthesizer calibration.
     0x59,   // FSTEST    Frequency synthesizer calibration.
     0x81,   // TEST2     Various test settings.
     0x35,   // TEST1     Various test settings.
     0x09,   // TEST0     Various test settings.
     0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
     0x29,   // IOCFG2    GDO2 output pin configuration.
     0x06,   // IOCFG0D   GDO0 output pin configuration.
     0x04,   // PKTCTRL1  Packet automation control.
     0x05,   // PKTCTRL0  Packet automation control.
     0x00,   // ADDR      Device address.
     0xFF    // PKTLEN    Packet length.
 };

extern RF_SETTINGS code TISettings; //it didnt work
*/


//Function definitions

HAL_StatusTypeDef __spi_write(uint8_t* addr, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef __spi_read(uint8_t* addr, uint8_t *pData, uint16_t size);

void TI_init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin);

void init_serial(UART_HandleTypeDef* huart);

void TI_write_reg(UINT8 addr, UINT8 value);
void TI_write_burst_reg(BYTE addr, BYTE * buffer, BYTE count);
void TI_write_burst_reg_c(BYTE addr, BYTE * buffer, BYTE count);
void TI_strobe(BYTE strobe);
BYTE TI_read_reg(BYTE addr);
BYTE TI_read_status(BYTE addr);
void TI_read_burst_reg(BYTE addr, BYTE * buffer, BYTE count);
BOOL TI_receive_packet(BYTE * rxBuffer, UINT8 *length);
void TI_send_packet(BYTE * txBuffer, UINT8 size);
void TI_write_settings();
UINT8 get_random_byte(void);
void Power_up_reset();

#endif /* INC_CC1101_H_ */
