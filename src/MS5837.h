/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------

Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library

Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature
sensor.

Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS

#include "Arduino.h"
#include <Wire.h>

class LANDSHARKS_MS5837 {
public:
	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;
	static const uint8_t MS5837_UNRECOGNISED;

	static const uint8_t NEEDS_RESET   = 0;
	static const uint8_t RESET_SUCCESS = 1;
	static const uint8_t INIT_SUCCESS  = 2;
	static const uint8_t CONN_GOOD     = 3;

	/* initialize the I2C interface. 
	 * Keep running this until the status variable is INIT_SUCCESS
	 */
	void init(TwoWire &wirePort = Wire);

	/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
	 * and MS5837::MS5837_02BA.
	 */
	void setModel(uint8_t model);
	uint8_t getModel();

	/** Provide the density of the working fluid in kg/m^3. Default is for
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float density);

	/* return the status variable
	 */
	uint8_t getStatus();

	/* try to update the pressure and temperature values
	 */
	void updateValues();

	/** Pressure returned in mbar or mbar*conversion rate.
	 */
	float pressure(float conversion = 1.0f);

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 */
	float depth();

	/** Altitude returned in meters (valid for operation in air only).
	 */
	float altitude();

private:
	//This stores the requested i2c port
	TwoWire * _i2cPort;

	uint16_t C[8]; //stores calibration constants from sensor PROM
	
	uint32_t D1_pres, D2_temp;
	
	int32_t TEMP; //temperature
	int32_t P; //pressure
	uint8_t _model; //sensor variant
	
	uint8_t status = 0; //status variable. Gets set to a higher value each time the code reaches a new checkpoint

	uint32_t readStartTime = 0; //holds the time of each read request sent. Only attempt to read after this has incremented 20ms

	float fluidDensity = 1029; //default fluid density for depth calcuation

	/** Performs calculations per the sensor data sheet for conversion and
	 *  second order compensation.
	 */
	
	/* send a byte to the device, return true if successful
	*/
	bool sendByte(uint8_t value); 
	 
	void calculate();

	uint8_t crc4(uint16_t n_prom[]); //PROM data integrity check
};

#endif