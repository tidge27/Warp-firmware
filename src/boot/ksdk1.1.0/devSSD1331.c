#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	// kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOA, 2),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
	// The above was B0
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

static int
writeData(uint8_t *  dataPayloadBytes, int payloadLength)
{
	spi_status_t status;
	// uint8_t		dataInBuffer[payloadLength];

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(1);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC high (data).
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinDC);

	// status = SPI_DRV_MasterTransferBlocking(0		/* master instance */,
	status = SPI_DRV_MasterTransfer(0		/* master instance */,
						NULL		/* spi_master_user_config_t */,
						dataPayloadBytes,
						NULL,
						payloadLength	/* transfer size */
						// 1000		/* timeout in microseconds (unlike I2C which is ms) */);
					);

	while(SPI_DRV_MasterGetTransferStatus(0, NULL) == kStatus_SPI_Busy){}
	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	// writeCommand(0x72);				// RGB Color, 65k color formatt
	// writeCommand(0x32);				// RGB Color, 256 color formatt
	writeCommand(0x33);				// RGB Color, 256 color formatt
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	// 95
	writeCommand(0x3F);
	// 63

// Since this is a 96x64 diplay this makes sense!!!
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");



	/*
	 *	Read the manual for the SSD1331 (SSD1331_1.2.pdf) to figure
	 *	out how to fill the entire screen with the brightest shade
	 *	of green.
	 */

	 // Will leave this in for the time being to confirm that the screen is working
	 writeCommand(kSSD1331CommandDRAWRECT);
	 writeCommand(0x00);
   writeCommand(0x00);
 	 writeCommand(0x5F);
 	 writeCommand(0x3F);

	 writeCommand(0x00);
 	 writeCommand(0xFF);
	 writeCommand(0x00);

	 writeCommand(0x00);
	 writeCommand(0xFF);
	 writeCommand(0x00);



//	SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle...\n");



	return 0;
}


int devSSD1331striperect(void)
{
	writeCommand(kSSD1331CommandSETCOLUMN);
	writeCommand(0x0A);
	writeCommand(0x13);
	// Set the columns to scan over
	writeCommand(kSSD1331CommandSETROW);
	writeCommand(0x0A);
	writeCommand(0x15);
	uint8_t imageBuffer[240];
	for (int r = 0; r < 4; r++)
	{
	// bool r = 0;
		for (int i = 0; i < 10; i++)
		{
			imageBuffer[r*60+2*i] = (0xF8);
			imageBuffer[r*60+2*i+1] = (0x00);
		}
		for (int i = 0; i < 10; i++)
		{
			imageBuffer[r*60+20+2*i] = (0x07);
			imageBuffer[r*60+20+2*i+1] = (0xE0);
		}
		for (int i = 0; i < 10; i++)
		{
			imageBuffer[r*60+40+2*i] = (0x00);
			imageBuffer[r*60+40+2*i+1] = (0x1F);
		}
	}
	writeData(imageBuffer, 240);

	// Set the rows to scan over
	return 0;
}

int devSSD1331printDigit(int digit, int x, int y, uint8_t big_digit)
{
	SEGGER_RTT_WriteString(0, "\r\n\tprint digitty\n");
	// uint8_t imageBuffer[768];
	uint8_t imageBuffer[32];
	int multi = 1;
	if(big_digit){
		multi = 2;
	}
const uint8_t numBuffer[10][48] = {
	{0x00, 0x00, 0x07, 0xE0, 0x0F, 0xF0, 0x1C, 0x38, 0x38, 0x18, 0x30, 0x1C, 0x70, 0x3C, 0x70, 0x7C, 0x70, 0x6C, 0x70, 0xCC, 0x60, 0x8C, 0x61, 0x8E, 0x63, 0x0E, 0x62, 0x0C, 0x76, 0x0C, 0x7C, 0x0C, 0x7C, 0x0C, 0x38, 0x1C, 0x38, 0x1C, 0x38, 0x18, 0x1C, 0x38, 0x0F, 0xF0, 0x07, 0xC0, 0x00, 0x00},
	{0x00, 0x00, 0x03, 0xC0, 0x0F, 0xC0, 0x1F, 0xC0, 0x39, 0xC0, 0x20, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x01, 0xC0, 0x01, 0xE0, 0x1F, 0xFE, 0x1F, 0xFC, 0x00, 0x00},
	{0x00, 0x00, 0x1F, 0xE0, 0x3F, 0xF8, 0x30, 0x3C, 0x00, 0x1C, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x18, 0x00, 0x38, 0x00, 0x70, 0x00, 0xE0, 0x01, 0xC0, 0x03, 0x80, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x3C, 0x00, 0x3F, 0xFE, 0x3F, 0xFE, 0x00, 0x00},
	{0x00, 0x00, 0x1F, 0xE0, 0x3F, 0xF8, 0x00, 0x3C, 0x00, 0x1C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x38, 0x07, 0xF0, 0x07, 0xF0, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x1C, 0x30, 0x3C, 0x3F, 0xF8, 0x1F, 0xE0, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x70, 0x00, 0x78, 0x00, 0xF8, 0x01, 0xF8, 0x01, 0xB8, 0x03, 0x38, 0x02, 0x38, 0x06, 0x38, 0x0C, 0x38, 0x0C, 0x38, 0x18, 0x38, 0x30, 0x38, 0x30, 0x38, 0x60, 0x38, 0x7F, 0xFE, 0x7F, 0xFF, 0x00, 0x78, 0x00, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x00},
	{0x00, 0x00, 0x1F, 0xF8, 0x3F, 0xF8, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x38, 0x00, 0x3F, 0xE0, 0x3F, 0xF8, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x1C, 0x60, 0x78, 0x7F, 0xF0, 0x3F, 0xC0, 0x00, 0x00},
	{0x00, 0x00, 0x03, 0xF8, 0x0F, 0xF8, 0x1E, 0x00, 0x18, 0x00, 0x38, 0x00, 0x30, 0x00, 0x70, 0x00, 0x70, 0x00, 0x73, 0xF0, 0x6F, 0xF8, 0x78, 0x1C, 0x78, 0x0C, 0x70, 0x0E, 0x70, 0x0E, 0x70, 0x0E, 0x70, 0x0E, 0x30, 0x0E, 0x30, 0x0C, 0x38, 0x1C, 0x1C, 0x38, 0x0F, 0xF8, 0x07, 0xE0, 0x00, 0x00},
	{0x00, 0x00, 0x3F, 0xFE, 0x3F, 0xFE, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x18, 0x00, 0x38, 0x00, 0x38, 0x00, 0x30, 0x00, 0x70, 0x00, 0x60, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0xC0, 0x01, 0xC0, 0x01, 0x80, 0x03, 0x80, 0x03, 0x80, 0x03, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x0F, 0xE0, 0x1F, 0xF8, 0x38, 0x38, 0x30, 0x1C, 0x70, 0x1C, 0x70, 0x0C, 0x70, 0x1C, 0x30, 0x1C, 0x18, 0x38, 0x0F, 0xE0, 0x0F, 0xE0, 0x1C, 0x78, 0x30, 0x1C, 0x70, 0x0C, 0x60, 0x0C, 0x60, 0x0E, 0x60, 0x0C, 0x70, 0x0C, 0x70, 0x1C, 0x38, 0x38, 0x1F, 0xF8, 0x0F, 0xE0, 0x00, 0x00},
	{0x00, 0x00, 0x07, 0xE0, 0x1F, 0xF0, 0x1C, 0x18, 0x38, 0x1C, 0x30, 0x0C, 0x30, 0x0E, 0x70, 0x0E, 0x70, 0x0E, 0x30, 0x0E, 0x30, 0x0E, 0x38, 0x1E, 0x1C, 0x3E, 0x1F, 0xF6, 0x07, 0xC6, 0x00, 0x06, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x1C, 0x00, 0x18, 0x10, 0x78, 0x1F, 0xF0, 0x0F, 0xC0, 0x00, 0x00}
};
SEGGER_RTT_WriteString(0, "\r\n\tSetup zero\n");
writeCommand(kSSD1331CommandSETCOLUMN);
writeCommand(y);
writeCommand(y + multi * 0x18 - 1);
// Set the columns to scan over
writeCommand(kSSD1331CommandSETROW);
writeCommand(x);
writeCommand(x + multi * 0x10 -1);
uint8_t maskbit = 0x00;
for (int row = 0; row < 24; row++)
{
	for (int r = 0; r < 2; r++)
	{
		int not_r = 0;
		if(r==0)
		{
			not_r = 1;
		}
		maskbit = 0x80;
		for (int i = 7; i > -1; i--)
		{

			if ((numBuffer[digit][not_r+2*row]&maskbit)>0) {
				imageBuffer[r*8+i] = (0xFF);
				// imageBuffer[r*16+2*i+1] = (0x00);
			} else {
				imageBuffer[r*8+i] = (0x00);
				// imageBuffer[r*16+2*i+1] = (0x00);
			}
			if(maskbit > 0x01){
				maskbit = maskbit >> 1;
			}
		}
	}
	if (big_digit){
		for(int count = 15; count != -1; count--)
		{
			imageBuffer[count*2+1] = imageBuffer[count];
			imageBuffer[count*2] = imageBuffer[count];
		}
		writeData(imageBuffer, 32);
		writeData(imageBuffer, 32);
	} else {
		writeData(imageBuffer, 16);
	}

}
return 0;
}
