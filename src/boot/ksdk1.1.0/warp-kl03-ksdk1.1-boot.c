/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018: Jan Heck.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



/*
	This is a modified version of the Warp Firmware, as described above.

	This project uses an accelerometer, gyro, OLED display, and hall effect sensor
	to graphically display the wobble, and the cadence of a bike when attached to
	the frame.

	There are two important parts to the code in this file:
	(1) The while loop which continually updates the display every 30ms.
	(2) The Interupt handler for PORT A which is triggered when the magnet passes by
			the hall effect sensor.

	These two parts are documented with inline comments

	There are also 3 driver files: devL3GD20H.h, devMMA8451Q.h, devSSD1331.h
	These are modified versions of drivers available in the warp firmware, but
	include aditional functions, and modifications.  These are largeley aimed at
	increasing the speed to which data is read from and written to the devices,
	since the performance of the code is important to retain a high refresh rate.
		Some of these functions are extremely specific, and could perhaps have been
	better placed for readability of the code.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_gpio_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


// #include "devINA219.h"
#include "devL3GD20H.h"
#include "devMMA8451Q.h"
#include "devSSD1331.h"



#define					kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define					kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define					kWarpConstantStringErrorSanity		"\rSanity Check Failed!"


// volatile WarpI2CDeviceState		  deviceINA219State;
volatile WarpI2CDeviceState deviceL3GD20HState;
volatile WarpI2CDeviceState deviceMMA8451QState;
volatile uint32_t last_milliseconds;
volatile uint32_t cadence;
volatile uint32_t last_cadence;
volatile uint32_t writing_cadence;

volatile uint32_t previous_timestamp = 0;
volatile float previous_angle = 0;

volatile uint8_t time_delta;

/*
 *	TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t		i2cMasterState;
volatile spi_master_state_t		spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;


/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t			gWarpI2cBaudRateKbps	= 100000;
volatile uint32_t			gWarpUartBaudRateKbps	= 1;
volatile uint32_t			gWarpSpiBaudRateKbps	= 100000;
volatile uint32_t			gWarpSleeptimeSeconds	= 0;
volatile WarpModeMask			gWarpMode		= kWarpModeDisableAdcOnSleep;


/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus				writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus        writeBytesToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t *  payloadBytes, int payloadLength);
WarpStatus				writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void					warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);



/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}




// Override the GPIO handler
void PORTA_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PORT_HAL_ClearPortIntFlag(PORTA_BASE);
		SEGGER_RTT_WriteString(0, "Interrupting:  ");
		uint32_t milliseconds = OSA_TimeGetMsec();
		if (milliseconds > last_milliseconds + 100)
		{
			last_cadence = cadence;
			cadence = 60000/(milliseconds - last_milliseconds);
			last_milliseconds = milliseconds;
		}
		else if (milliseconds < last_milliseconds)
		{
			last_cadence = cadence;
			cadence = 60000/(milliseconds + 0xFFFF - last_milliseconds);
			last_milliseconds = milliseconds;
		}
		// SEGGER_RTT_printf(0, "%d, %d\n", last_milliseconds, milliseconds);
}

/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}



/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
					power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}

/*
 *	From KSDK power_manager_demo.c <<END>>>
 */





void
enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	// PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);
	//
	// /*	Warp KL03_SPI_MOSI	--> PTA7	(ALT3)		*/
	// PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);
	//
	// /*	Warp KL03_SPI_SCK	--> PTB0	(ALT3)		*/
	// PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);


	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}




void
enableI2Cpins(uint16_t pullupValue)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);


	/*
	 *	TODO: need to implement config of the DCP
	 */
	//...
}



void
disableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);


	/*	Warp KL03_I2C0_SCL	--> PTB3	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);


	/*
	 *	TODO: need to implement clearing of the DCP
	 */
	//...

	/*
	 *	Drive the I2C pins low
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);


	CLOCK_SYS_DisableI2cClock(0);
}






int
main(void)
{
	rtc_datetime_t				warpBootDate;

	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure			powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};



	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);



	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;



	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();



	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting in 3... ");
	OSA_TimeDelay(500);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(500);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(500);



	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM,
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);



	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);



	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);



	/*
	 *	Setup Power Manager Driver
	 */




	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	// warpSetLowPowerMode(kWarpPowerModeVLPR, 0);
	warpSetLowPowerMode(kWarpPowerModeRUN, 0);



	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetPullMode(PORTA_BASE, 7, kPortPullUp);
INT_SYS_EnableIRQGlobal();
	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	// lowPowerPinStates();



	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(500);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);



	/*
	 *	Initialize all the sensors
	 */
		SEGGER_RTT_WriteString(0, "Initialising sensors and display...\n");
		initL3GD20H(0x6B, &deviceL3GD20HState);
		initMMA8451Q(0x1D, &deviceMMA8451QState);
		devSSD1331init();


    OSA_TimeDelay(1000);
		configureSensorL3GD20H(0b11111111,/* ODR 800Hz, Cut-off 100Hz, see table 21, normal mode, x,y,z enable */
														0b00100000,
														0b00000000,/* normal mode, disable FIFO, disable high pass filter */
														65535
														);
		configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
														0x01,/* Normal read 8bit, 800Hz, normal, active mode */
														65535
														);


		while (1)
		{
			uint32_t sensor_timestamp = OSA_TimeGetMsec();

			// Wait in this while loop for 30ms to pass since the last sensor reading.
			// This is where the display's digits will be updated.
			while (sensor_timestamp < previous_timestamp +  30 && sensor_timestamp > previous_timestamp) {
				sensor_timestamp = OSA_TimeGetMsec();
				// SEGGER_RTT_printf(0, "I still got time, start: %d \n", sensor_timestamp);

				// Check to see if the cadence on the screen is up-to-date.
				if (last_cadence != cadence)
				{
					uint32_t cadence_print_start = OSA_TimeGetMsec();

					// The block below determines the color of the numeric display.  Since
					// the display is set to the 256 color mode, additional speed comes at
					// the expense of limited color resolution.  This code selects green
					// for a cadence between 80 and 100, and fades to red below 70, and
					// above 110.
					uint8_t digit_color = 0x1C;

					// Use 'writing_cadence' variable, to ensure there are no errors with
					// the value of cadence changing half way though it being written to
					// the screen
					writing_cadence = cadence;
					uint8_t color_mag = 0;

					if (writing_cadence > 100)
					{
						// Inform the user that they need to change up a gear, by displaying
						// the horizontal line above the cadence reading
						devSSD1331plotHigh(1);
						color_mag = writing_cadence-100;
					}
					else if (writing_cadence < 80)
					{
						// Inform the user that they need to change down a gear, by displaying
						// the horizontal line above the cadence reading
						devSSD1331plotHigh(2);
						color_mag = 80 - writing_cadence;
					} else {
						// Inform the user that their gear selection is sensible, by clearing
						// the gear suggestion lines.
						devSSD1331plotHigh(0);
					}
					if(color_mag > 14) {
						color_mag = 14;
					}
					if(color_mag%2){
						color_mag = color_mag/2;
						digit_color = digit_color - ((color_mag+1) << 3) + (color_mag << 5);
					} else {
						color_mag = color_mag/2;
						digit_color = digit_color - (color_mag << 3) + (color_mag << 5);
					}

					// Write each digit to the screen.  Specifying the number (0-9),
					// the coordinates, the size, and the color
					devSSD1331printDigit((writing_cadence/100)%10, 39 , 8, 0, digit_color);
					devSSD1331printDigit((writing_cadence/10)%10, 24 , 8, 0, digit_color);
					devSSD1331printDigit((writing_cadence)%10, 9 , 8, 0, digit_color);

				last_cadence = writing_cadence;
				// SEGGER_RTT_printf(0, "Milliseconds to print angle: %d\n", OSA_TimeGetMsec() - cadence_print_start);
				sensor_timestamp = OSA_TimeGetMsec();
			}
				// SEGGER_RTT_printf(0, "I still got time, end: %d \n", sensor_timestamp);
			}

			// The time since the last reading has now ben 30ms, since the while loop has exited.
			// Read the sensors, and calculate the angles.  Then update the live plot.

			// section for calculating the wobble angle
			uint32_t sensor_reading_start = OSA_TimeGetMsec();



			// Start reading from the I2C devices here:

			enableI2Cpins(65535 /* pullupValue*/);
			readSensorRegisterL3GD20H(0x28);
			uint8_t xlsb = deviceL3GD20HState.i2cBuffer[0];
			readSensorRegisterL3GD20H(0x29);
			uint8_t xmsb = deviceL3GD20HState.i2cBuffer[0];
			// disableI2Cpins();
			int16_t gyro_reading = (int16_t)((xmsb)<<8) | (xlsb);
			gyro_reading *= 0.00875F;

			// enableI2Cpins(65535 /* pullupValue*/);
			// y msb = 64*g
			readSensorRegisterMMA8451Q(0x01);
			int8_t yaccel = (int8_t)deviceMMA8451QState.i2cBuffer[0];
			// z msb = 64*g
			readSensorRegisterMMA8451Q(0x05);
			int8_t zaccel = (int8_t)deviceMMA8451QState.i2cBuffer[0];
			disableI2Cpins();

			// Finished reading from the I2C devices!



			float adjustval = 0;
			if (abs(zaccel)>abs(yaccel))
			{
				adjustval = (float)yaccel*57.29578/abs((float)zaccel);
				// This is a very crude linear approximation for arctan() for the small angles I am looking at
				// This if statement first checks that the zaccel > yaccel, to make sure that the angle is less than 45deg
				// from vertical, since anything outisde of this would not be at all meaningful.
			}

			float accelmag = abs((float)(zaccel*zaccel + yaccel*yaccel))/4096.0;
			// This value is the magnitude of the acceleration in the z,y plane.  This
			// is later used to ensure that the value is reasonably close to g, and
			// hence due to gravity, not wild movements of the device.


			if (sensor_timestamp < previous_timestamp) {
				previous_timestamp = 0;
				// Avoid any errors if the clock counter has looped
			}

			// convert milliseconds to seconds
			// Typecast the timestamp as a float, ensures that the angle is calculated correctly
			float angle = (gyro_reading * (float)(sensor_timestamp - previous_timestamp)/1000.0) + previous_angle;
			// The angle calculated iteratively from the previous value, with only the gyro reading incorporated


			if (accelmag > 0.5 && accelmag < 2.0)
			{
				angle *= 0.99;
				angle += adjustval*0.01;
				//  Checks that the magnitude of the accelration is close to 1g.  This
				// stops bad measuremtns affecting the calculated angle
				// If so, the angle from the accelerometer affects 1% of the angle reading.
			}


			// SEGGER_RTT_printf(0, "%d, %d, %d, %d, %d, %d\n", sensor_timestamp - previous_timestamp, zaccel, yaccel, gyro_reading, (int)angle, adjustval);

			// End of angle read and calculate iteration
			// SEGGER_RTT_printf(0, "Milliseconds to read and calculate angle: %d\n", OSA_TimeGetMsec() - sensor_reading_start);


			uint32_t sensor_plot_start = OSA_TimeGetMsec();


g


			int rounded_angle = (int)angle;
			devSSD1331plotAngle(rounded_angle);
			// Plot the angle (to the nearest degree) on the screen, using the above
			// function.  The function accepts an integer, so the angle (a float) is
			// converted to the rounded_angle variable which is an integer.


			// Plot angle to the nearest degree (rounds down always)
			// SEGGER_RTT_printf(0, "Milliseconds to plot angle: %d\n", OSA_TimeGetMsec() - sensor_plot_start);



			// SEGGER_RTT_printf(0, "%d deg.\n", rounded_angle);


			previous_timestamp = sensor_timestamp;
			previous_angle = angle;
			// Update some of the variables for the iterative algorithm,  These could
			// be changed to arrays to improve the readability of the code
		}

	// }

	return 0;
}





WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	/*
	 *	TODO: Need to appropriately set the pullup value here
	 */
	enableI2Cpins(65535 /* pullupValue*/);

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
						1000	/* timeout in milliseconds */);
	disableI2Cpins();

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}


WarpStatus
writeBytesToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t *  payloadBytes, int payloadLength)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[payloadLength];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	// payloadBuffer[0] = payloadByte;

	/*
	 *	TODO: Need to appropriately set the pullup value here
	 */
	enableI2Cpins(65535 /* pullupValue*/);

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBytes,
						payloadLength,
						1000	/* timeout in milliseconds */);
	disableI2Cpins();

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;

	enableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0		/* master instance */,
						NULL		/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength	/* transfer size */,
						1000		/* timeout in microseconds (unlike I2C which is ms) */);
	disableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}
