'''
 * BME280.py
 *
 *  Copyright (C) Daniel Kampert, 2020
 *	Website: www.kampis-elektroecke.de
 *  File info: Module for the BME280 I2C pressure sensor.

  GNU GENERAL PUBLIC LICENSE:
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

  Errors and commissions should be reported to DanielKampert@kampis-elektroecke.de
'''

import smbus
from enum import Enum

BME280_REGISTER_HUM_LSB 	= 0xFE
BME280_REGISTER_HUM_MSB 	= 0xFD
BME280_REGISTER_TEMP_XLSB 	= 0xFC
BME280_REGISTER_TEMP_LSB 	= 0xFB
BME280_REGISTER_TEMP_MSB 	= 0xFA
BME280_REGISTER_PRESS_XLSB	= 0xF9
BME280_REGISTER_PRESS_LSB	= 0xF8
BME280_REGISTER_PRESS_MSB	= 0xF7
BME280_REGISTER_CONFIG		= 0xF5
BME280_REGISTER_CTRL_MEAS	= 0xF4
BME280_REGISTER_CTRL_HUM	= 0xF2
BME280_REGISTER_STATUS		= 0xF3
BME280_REGISTER_SOFT_RESET	= 0xE0
BME280_REGISTER_ID			= 0xD0

BME280_ID 					= 0x60

BME280_CMD_RESET			= 0xB6

BME280_BIT_MEASURE			= 0x03

BME280_ADDRESS 				= 0x76

class BME280_OSS(Enum):
	SKIP					= 0x00
	X1						= 0x01
	X2						= 0x02
	X4						= 0x03
	X8						= 0x04
	X16						= 0x05

class BME280_Mode(Enum):
	SLEEP					= 0x00
	FORCED					= 0x01
	NORMAL					= 0x03

class BME280_Standby(Enum):
	STANDBY_05				= 0x00
	STANDBY_62				= 0x01
	STANDBY_125				= 0x02
	STANDBY_250				= 0x03
	STANDBY_500				= 0x04
	STANDBY_1000			= 0x05
	STANDBY_10				= 0x06
	STANDBY_20				= 0x07

class BME280_Filter(Enum):
	OFF						= 0x00
	COEF_2					= 0x01
	COEF_4					= 0x02
	COEF_8					= 0x03
	COEF_16					= 0x04

class BME280_Data:
	def __init__(self, Temperature, Pressure, Humidity):
		self.__Temperature = Temperature
		self.__Pressure = Pressure
		self.__Humidity = Humidity

	def __repr__(self):
		return "BME280_Data()"

	def __str__(self):
		return "Temperature : {} Degree Celsius\n" \
		       "Pressure : {} hPa\n" \
		       "Humidity : {} %RH".format(self.__Temperature, self.__Pressure, self.__Humidity)

	@property
	def temperature(self):
		return self.__Temperature

	@property
	def pressure(self):
		return self.__Pressure

	@property
	def humidity(self):
		return self.__Humidity

class BME280:
	def __init__(self, Interface, SDO):
		"""Object constructor. Initializes the I2C interface, check the sensor device ID and load the calibration coefficients.

			Parameters:
				Interface (int): I2C interface number.
				SDO (boolean): Boolean value to represent the sensor SDO pin state.

			Returns:
				None
		"""
		self.__CalibCoef = dict()
		self.__Interface = smbus.SMBus(Interface)
		self.__Address = BME280_ADDRESS | ((SDO & 0x01) << 0x00)

		ID = self.GetID()
		while(not(ID)):
			ID = self.GetID()

		if(ID != BME280_ID):
			raise ValueError("[ERROR] Wrong device ID: {}!".format(ID))

		self.__LoadCalibrationCoef()
		self.SetFilter(BME280_Filter.OFF)

	def __del__(self):
		"""Object deconstructor.
			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.close()

	def __ReadSInt(self, Address):
		"""Read a signed integer from two sensor registers.

			Parameters:
				Address (int): Register address.

			Returns:
				int: Signed register value.
		"""
		Data = self.__ReadUInt(Address)

		if(Data & (0x01 << 0x0F)):
			Data -= 65536

		return Data

	def __ReadUInt(self, Address):
		"""Read a unsigned integer from two sensor registers.

			Parameters:
				Address (int): Register address.

			Returns:
				int: Unsigned register value.
		"""
		Data = self.__Interface.read_i2c_block_data(self.__Address, Address, 2)

		return (Data[1] << 0x08) | Data[0]

	def __ReadUChar(self, Address):
		"""Read a unsigned char from one sensor register.

			Parameters:
				Address (int): Register address.

			Returns:
				char: Unsigned register value.
		"""
		return self.__Interface.read_byte_data(self.__Address, Address)

	def __ReadSChar(self, Address):
		"""Read a signed char from one sensor register.

			Parameters:
				Address (int): Register address.

			Returns:
				char: Signed register value.
		"""
		Data = self.__ReadUChar(Address)

		if(Data & (0x01 << 0x07)):
			Data -= 256

		return Data

	def __LoadCalibrationCoef(self):
		"""Load the sensor calibration parameters.

			Parameters:
				None

			Returns:
				None
		"""
		self.__CalibCoef.update({"T1": self.__ReadUInt(0x88)})
		self.__CalibCoef.update({"T2": self.__ReadSInt(0x8A)})
		self.__CalibCoef.update({"T3": self.__ReadSInt(0x8C)})
		self.__CalibCoef.update({"P1": self.__ReadUInt(0x8E)})
		self.__CalibCoef.update({"P2": self.__ReadSInt(0x90)})
		self.__CalibCoef.update({"P3": self.__ReadSInt(0x92)})
		self.__CalibCoef.update({"P4": self.__ReadSInt(0x94)})
		self.__CalibCoef.update({"P5": self.__ReadSInt(0x96)})
		self.__CalibCoef.update({"P6": self.__ReadSInt(0x98)})
		self.__CalibCoef.update({"P7": self.__ReadSInt(0x9A)})
		self.__CalibCoef.update({"P8": self.__ReadSInt(0x9C)})
		self.__CalibCoef.update({"P9": self.__ReadSInt(0x9E)})
		self.__CalibCoef.update({"H1": self.__ReadUChar(0xA1)})
		self.__CalibCoef.update({"H2": self.__ReadSInt(0xE1)})
		self.__CalibCoef.update({"H3": self.__ReadUChar(0xE3)})
		Data = self.__Interface.read_i2c_block_data(self.__Address, 0xE4, 2)
		self.__CalibCoef.update({"H4": (Data[0] << 0x04) | (Data[1] >> 0x04)})
		Data = self.__Interface.read_i2c_block_data(self.__Address, 0xE5, 2)
		self.__CalibCoef.update({"H5": (Data[1] << 0x04) | (Data[0] >> 0x04)})
		self.__CalibCoef.update({"H6": self.__ReadSChar(0xE7)})

	def __ReadTemperature(self, OSS_Temperature):
		"""Start a new temperature measurement and read the raw result from the sensor.

			Parameters:
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling.

			Returns:
				int: 20 bit raw temperature value.
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0x1F
		Data |= OSS_Temperature.value << 0x05
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)

		while(self.ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_TEMP_MSB, 3)

		return (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

	def __ReadPressure(self, OSS_Pressure):
		"""Start a new pressure measurement and read the raw result from the sensor.

			Parameters:
				OSS_Pressure (BME280_OSS): Pressure measurement oversampling.

			Returns:
				int: 20 bit raw pressure value.
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0xE3
		Data |= OSS_Pressure.value << 0x02
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)

		while(self.ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_PRESS_MSB, 3)

		return (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

	def __ReadHumidity(self, OSS_Humidity):
		"""Read the raw humidity from the sensor.

			Parameters:
				OSS_Humidity (BME280_OSS): Humidity measurement oversampling.

			Returns:
				int: 16 bit raw humidity value.
		"""
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_HUM, OSS_Humidity.value)

		while(self.ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_HUM_MSB, 2)

		return (Data[0] << 0x08) | Data[1]

	def __CalcTemperature(self, RawTemp):
		"""Calculate the calibrated temperature from the raw temperature value.

			Parameters:
				RawTemp (int): Raw temperature from the sensor.

			Returns:
				float: Temperature value in Degree Celsius.
		"""
		self.__var1 = (((RawTemp >> 0x03) - (self.__CalibCoef["T1"] << 0x01)) * self.__CalibCoef["T2"]) >> 0x0B
		self.__var2 = (((((RawTemp >> 0x04) - self.__CalibCoef["T1"]) * ((RawTemp >> 0x04) - self.__CalibCoef["T1"])) >> 0x0C) * self.__CalibCoef["T3"]) >> 0x0E
		self.__CalibCoef.update({"t_fine": self.__var1 + self.__var2})

		return round(((self.__CalibCoef["t_fine"] * 0x05 + 0x80) >> 0x08) / 100.0, 4)

	def __CalcPressure(self, RawPressure):
		"""Calculate the calibrated pressure from the raw pressure value. A temperature measurement has to be done before to get the correct results.

			Parameters:
				RawPressure (int): Raw pressure from the sensor.

			Returns:
				float: Pressure value in hPa.
		"""
		self.__var1 = self.__CalibCoef["t_fine"] - 128000
		self.__var2 = self.__var1 * self.__var1 * self.__CalibCoef["P6"]
		self.__var2 = self.__var2 + ((self.__var1 * self.__CalibCoef["P5"]) << 0x11)
		self.__var2 = self.__var2 + (self.__CalibCoef["P4"] << 0x23)
		self.__var1 = (((self.__var1 * self.__var1 * self.__CalibCoef["P3"]) >> 0x08) + ((self.__var1 * self.__CalibCoef["P2"]) << 0x0C))
		self.__var1 = (((0x01 << 0x2F) + self.__var1) * self.__CalibCoef["P1"]) >> 0x21
		if(self.__var1 == 0x00):
			return 0x00

		self.__p = 1048576 - RawPressure
		self.__p = (((self.__p << 0x1F) - self.__var2) * 3125) // self.__var1
		self.__var1 = (self.__CalibCoef["P9"] * (self.__p >> 0x0D) * (self.__p >> 0x0D)) >> 0x19
		self.__var2 = (self.__CalibCoef["P8"] * self.__p) >> 0x13

		return round((((self.__p + self.__var1 + self.__var2) >> 0x08) + (self.__CalibCoef["P7"] << 0x04)) / 25600, 2)

	def __CalcHum(self, RawHumidity):
		"""Calculate the calibrated humidity from the raw humidity value. A temperature measurement has to be done before to get the correct results.

			Parameters:
				RawHumidity (int): Raw humidity from the sensor.

			Returns:
				float: Humidity value in %RH.
		"""
		self.__var1 = self.__CalibCoef["t_fine"] - 76800
		self.__var2 = RawHumidity * 16384
		self.__var3 = (self.__CalibCoef["H4"] * 1048576);
		self.__var4 = self.__CalibCoef["H5"] * self.__var1
		self.__var5 = (((self.__var2 - self.__var3) - self.__var4) + 16384) / 32768
		self.__var2 = (self.__var1 * self.__CalibCoef["H6"]) >> 0x0A
		self.__var3 = (self.__var1 * self.__CalibCoef["H3"]) >> 0x0B
		self.__var4 = ((self.__var2 * (self.__var3 + 32768)) >> 0x0A) + 2097152
		self.__var2 = ((self.__var4 * self.__CalibCoef["H2"]) + 8192) >> 0x0E
		self.__var3 = self.__var5 * self.__var2
		self.__var4 = ((self.__var3 / 32768) * (self.__var3 / 32768)) >> 0x07
		self.__var5 = self.__var3 - ((self.__var4 * self.__CalibCoef["H1"]) >> 0x04)
		
		if(self.__var5 < 0):
			self.__var5 = 0
		
		if(self.__var5 > 419430400):
			self.__var5 = 419430400

		self.__Humidity = (self.__var5 / 4096)
		if(self.__Humidity > 102400):
			self.__Humidity = 102400;

		return round(float(self.__Humidity) / 1000, 2)

	def Reset(self):
		"""Reset the sensor.

			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_SOFT_RESET, BME280_CMD_RESET)

	def ConversionRunning(self):
		"""Check if a conversion is active.

			Parameters:
				None

			Returns:
				boolean: True when conversion is running.
		"""
		return bool((self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_STATUS) & (0x01 << BME280_BIT_MEASURE)) >> BME280_BIT_MEASURE)

	def GetID(self):
		"""Return the device ID.

			Parameters:
				None

			Returns:
				int: Device ID.
		"""
		return self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_ID)

	def GetCalibrationCoef(self):
		"""Return a list with the calibration coefficients.

			Parameters:
				None

			Returns:
				list: Calibration coefficients.
		"""
		return self.__CalibCoef

	def GetFilter(self):
		"""Get the filter coefficient number from the sensor.

			Parameters:
				None

			Returns:
				BME280_Filter: Filter coefficient number.
		"""
		return BME280_Filter((self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CONFIG) >> 0x02) & 0x07)

	def SetFilter(self, Filter):
		"""Set the filter coefficient number for the sensor.

			Parameters:
				Filter (BME280_Filter): Filter coefficient number.

			Returns:
				None
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CONFIG)
		Data &= 0xE3
		Data |= (Filter.value << 0x02)
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CONFIG, Data)

	def SetStandby(self, Standby):
		"""Get the current standby time from the sensor.

			Parameters:
				Standby (BME280_Standby): Standby time.

			Returns:
				None
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CONFIG)
		Data &= 0xE0
		Data |= (Standby.value << 0x05)
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CONFIG, Data)

	def GetStandby(self):
		"""Get the current standby time from the sensor.

			Parameters:
				None

			Returns:
				BME280_Standby: Sensor standby time.
		"""
		return BME280_Standby(self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CONFIG) >> 0x05)

	def GetMode(self):
		"""Get the current device mode from the sensor.

			Parameters:
				None

			Returns:
				BME280_Mode: Sensor operation mode.
		"""
		return BME280_Mode(self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS) & 0x03)

	def SetMode(self, Mode):
		"""Set the current device mode for the sensor.

			Parameters:
				Mode (BME280_Mode): Sensor operation mode.

			Returns:
				None
				
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0xFC
		Data |= Mode.value
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)

	def MeasureTemperature(self, OSS_Temperature = BME280_OSS.X1):
		"""Read the calibrated temperature in degree Celsius from the sensor.

			Parameters:
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling.

			Returns:
				float: Temperature value in Degree Celsius.
		"""
		return self.__CalcTemperature(self.__ReadTemperature(OSS_Temperature))

	def MeasurePressure(self, OSS_Pressure = BME280_OSS.X1, OSS_Temperature = BME280_OSS.X1):
		"""Read the calibrated pressure in hPa from the sensor.

			Parameters:
				OSS_Pressure (BME280_OSS): Pressure measurement oversampling.
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling.

			Returns:
				float: Pressure value in hPa.
		"""
		self.MeasureTemperature(OSS_Temperature)

		return self.__CalcPressure(self.__ReadPressure(OSS_Pressure))

	def MeasureHumidity(self, OSS_Humidity = BME280_OSS.X1, OSS_Temperature = BME280_OSS.X1):
		"""Read the calibrated humidity in %RH from the sensor.

			Parameters:
				OSS_Humidity (BME280_OSS): Humidity measurement oversampling.
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling.

			Returns:
				float: Humidity value %RH.
		"""
		self.MeasureTemperature(OSS_Temperature)

		return self.__CalcHum(self.__ReadHumidity(OSS_Humidity))

	def Measure(self, OSS_Humidity = BME280_OSS.X1, OSS_Pressure = BME280_OSS.X1, OSS_Temperature = BME280_OSS.X1):
		"""Run a complete measurement cycle with each sensor.

			Parameters:
				OSS_Humidity (BME280_OSS): Humidity measurement oversampling.
				OSS_Pressure (BME280_OSS): Pressure measurement oversampling.
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling.

			Returns:
				BME280_Data: Temperature value in Degree Celsius, Pressure value in hPa and Humidity value in %RH.
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0xFC
		Data |= (OSS_Temperature.value << 0x05) | (OSS_Pressure.value << 0x02)
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_HUM, OSS_Humidity.value)

		while(self.ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_TEMP_MSB, 3)
		RawTemp = (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_PRESS_MSB, 3)
		RawPress = (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_HUM_MSB, 2)
		RawHumidity = (Data[0] << 0x08) | Data[1]

		return BME280_Data(self.__CalcTemperature(RawTemp), self.__CalcPressure(RawPress), self.__CalcHum(RawHumidity))

	def Start(self, Standby, Filter, OSS_Temperature = BME280_OSS.X1, OSS_Pressure = BME280_OSS.X1, OSS_Humidity = BME280_OSS.X1):
		"""Put the device into normal mode and start the continouus measurement.

			Parameters:
				OSS_Pressure (BME280_OSS): Pressure measurement oversampling.
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling.
				OSS_Humidity (BME280_OSS): Humidity measurement oversampling.
				Standby (BME280_Standby): Standby time.
				Filter (BME280_Filter): Filter coefficient number.

			Returns:
				None
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0xFC
		Data |= (OSS_Temperature.value << 0x05) | (OSS_Pressure.value << 0x02)

		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_HUM, OSS_Humidity.value)
		self.SetStandby(Standby)
		self.SetFilter(Filter)
		self.SetMode(BME280_Mode.NORMAL)

	def Get(self):
		"""Used in normal mode to read out a new temperature, pressure and humidity value.

			Parameters:
				None

			Returns:
				BME280_Data: Temperature value in Degree Celsius, pressure value in hPA and humidity value in %RH.
		"""
		while(self.ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_TEMP_MSB, 3)
		RawTemp = (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_PRESS_MSB, 3)
		RawPressure = (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME280_REGISTER_HUM_LSB, 2)
		RawHum = (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

		return BME280_Data(self.__CalcTemperature(RawTemp), self.__CalcPressure(RawPressure), self.__CalcHumidity(RawHum))

	def Stop(self):
		"""Stop the continouus measurement and put the device back into sleep mode.

			Parameters:
				None

			Returns:
				None
		"""
		self.SetMode(BME280_Mode.SLEEP)