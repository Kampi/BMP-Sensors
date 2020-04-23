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
BME280_REGISTER_STATUS		= 0xF3
BME280_REGISTER_SOFT_RESET	= 0xE0
BME280_REGISTER_ID			= 0xD0

BME280_ID 					= 0x60
BME280_ADDRESS 				= 0x76

BME280_CMD_RESET			= 0xB6

BME280_BIT_MEASURE			= 0x03

class BME280_OSS(Enum):
	x1						= 0x01
	x2						= 0x02
	x4						= 0x03
	x8						= 0x04
	x16						= 0x05

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

class BME280:
	def __init__(self, Interface, SDO):
		"""Object constructor. Initializes the I2C interface, check the sensor device ID and load the calibration coefficients.

			Parameters:
				Interface (int): I2C interface number
				SDO (boolean): Boolean value to represent the sensor SDO pin state

			Returns:
				None
		"""
		self.__CalibCoef = dict()
		self.__Interface = smbus.SMBus(Interface)
		self.__Address = BME280_ADDRESS | ((SDO & 0x01) << 0x00)

		ID = self.GetID()
		if(ID != BME280_ID):
			raise ValueError("[ERROR] Wrong device ID: {}".format(ID))
			return

		self.__LoadCalibrationCoef()

	def _ReadSInt(self, Address):
		"""Read a signed integer from two sensor registers.

			Parameters:
				Address (int): Register address

			Returns:
				int: Signed register value
		"""
		Data = self._ReadUInt(Address)

		if(Data & (0x01 << 0x0F)):
			Data -= 65536

		return Data

	def _ReadUInt(self, Address):
		"""Read a unsigned integer from two sensor registers.

			Parameters:
				Address (int): Register address

			Returns:
				int: Unsigned register value
		"""
		LSB = self.__Interface.read_byte_data(self.__Address, Address)
		MSB = self.__Interface.read_byte_data(self.__Address, Address + 1)
		
		return (MSB << 0x08) | LSB

	def _ReadUChar(self, Address):
		"""Read a unsigned char from one sensor register.

			Parameters:
				Address (int): Register address

			Returns:
				int: Unsigned register value
		"""
		return self.__Interface.read_byte_data(self.__Address, Address)

	def __LoadCalibrationCoef(self):
		"""Load the sensor calibration parameters.

			Parameters:
				None

			Returns:
				None
		"""
		T1 = self._ReadUInt(0x88)
		T2 = self._ReadSInt(0x8A)
		T3 = self._ReadSInt(0x8C)
		P1 = self._ReadUInt(0x8E)
		P2 = self._ReadSInt(0x90)
		P3 = self._ReadSInt(0x92)
		P4 = self._ReadSInt(0x94)
		P5 = self._ReadSInt(0x96)
		P6 = self._ReadSInt(0x98)
		P7 = self._ReadSInt(0x9A)
		P8 = self._ReadSInt(0x9C)
		P9 = self._ReadSInt(0x9E)
		H1 = self._ReadUChar(0xA1)
		H2 = self._ReadSInt(0xE1)
		H3 = self._ReadUChar(0xE3)
		H4 = self._ReadSInt(0xE4)
		H5 = self._ReadSInt(0xE5)

		self.__CalibCoef.update({"T1": T1})
		self.__CalibCoef.update({"T2": T2})
		self.__CalibCoef.update({"T3": T3})
		self.__CalibCoef.update({"P1": P1})
		self.__CalibCoef.update({"P2": P2})
		self.__CalibCoef.update({"P3": P3})
		self.__CalibCoef.update({"P4": P4})
		self.__CalibCoef.update({"P5": P5})
		self.__CalibCoef.update({"P6": P6})
		self.__CalibCoef.update({"P7": P7})
		self.__CalibCoef.update({"P8": P8})
		self.__CalibCoef.update({"P9": P9})
		self.__CalibCoef.update({"H1": H1})
		self.__CalibCoef.update({"H2": H2})
		self.__CalibCoef.update({"H3": H3})
		self.__CalibCoef.update({"H4": H4})
		self.__CalibCoef.update({"H5": H5})

	def GetID(self):
		"""Return the device ID.

			Parameters:
				None

			Returns:
				int: Device ID
		"""
		return self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_ID)

	def GetCalibrationCoef(self):
		"""Return a list with the calibration coefficients.

			Parameters:
				None

			Returns:
				list: Calibration coefficients
		"""
		return self.__CalibCoef
		
	def ConversionRunning(self):
		"""Check if a conversion is active.

			Parameters:
				None

			Returns:
				boolean: True when conversion is running
		"""
		return bool((self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_STATUS) & (0x01 << BME280_BIT_MEASURE)) >> BME280_BIT_MEASURE)

	def Reset(self):
		"""Reset the sensor.

			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_SOFT_RESET, BME280_CMD_RESET)

	def GetFilter(self):
		"""Get the filter coefficient number from the sensor.

			Parameters:
				None

			Returns:
				BME280_Filter: Filter coefficient number
		"""
		return BME280_Filter((self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CONFIG) >> 0x02) & 0x07)

	def SetFilter(self, Filter):
		"""Set the filter coefficient number for the sensor.

			Parameters:
				Filter (BME280_Filter): Filter coefficient number

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
				Standby (BME280_Standby): Standby time

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
				BME280_Standby: Current sensor standby time
		"""
		return BME280_Standby(self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CONFIG) >> 0x05)

	def GetMode(self):
		"""Get the current device mode from the sensor.

			Parameters:
				None

			Returns:
				BME280_Mode: Current sensor operation mode
		"""
		return BME280_Mode(self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS) & 0x03)

	def SetMode(self, Mode):
		"""Set the current device mode for the sensor.

			Parameters:
				Mode (BME280_Mode): Sensor operation mode

			Returns:
				None
				
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0xFC
		Data |= Mode.value
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)

	def ReadTemperature(self, OSS_Temperature):
		"""Read the raw temperature from the sensor.

			Parameters:
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling

			Returns:
				int: 16 bit raw temperature value
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0x1F
		Data |= OSS_Temperature.value << 0x05
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)

		while(self.ConversionRunning()):
			pass

		MSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_TEMP_MSB)
		LSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_TEMP_LSB)
		XLSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_TEMP_XLSB)

		return ((MSB << 0x0C) | (LSB << 0x04) | (XLSB >> 0x04))

	def MeasureTemperature(self, OSS_Temperature):
		"""Read the calibrated temperature in degree Celsius from the sensor.

			Parameters:
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling

			Returns:
				float: Temperature value
		"""
		self.__RawTemp = self.ReadTemperature(OSS_Temperature)
		self.__var1 = (((self.__RawTemp >> 0x03) - (self.__CalibCoef["T1"]<<1)) * self.__CalibCoef["T2"]) >> 0x0B
		self.__var2 = (((((self.__RawTemp >> 0x04) - (self.__CalibCoef["T1"])) * ((self.__RawTemp >> 0x04) - (self.__CalibCoef["T1"]))) >> 0x0C) * (self.__CalibCoef["T3"])) >> 0x0E
		self.__temperature_fine = self.__var1 + self.__var2

		return round(((self.__temperature_fine * 0x05 + 0x80) >> 0x08) / 100.0, 4)

	def ReadPressure(self, OSS_Pressure):
		"""Read the raw pressure from the sensor.

			Parameters:
				OSS_Pressure (BME280_OSS): Pressure measurement oversampling

			Returns:
				int: 16 bit raw pressure value
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS)
		Data &= 0xE3
		Data |= OSS_Pressure.value << 0x02
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_MEAS, Data)

		while(self.ConversionRunning()):
			pass
	
		MSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_PRESS_MSB)
		LSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_PRESS_LSB)
		XLSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_PRESS_XLSB)

		return ((MSB << 0x0C) | (LSB << 0x04) | (XLSB >> 0x04))

	def MeasurePressure(self, OSS_Pressure, OSS_Temperature):
		"""Read the calibrated pressure in hPa from the sensor.

			Parameters:
				OSS_Pressure (BME280_OSS): Pressure measurement oversampling
				OSS_Temperature (BME280_OSS): Temperature measurement oversampling

			Returns:
				float: Pressure value
		"""
		self.MeasureTemperature(OSS_Temperature)
		RawPressure = self.ReadPressure(OSS_Pressure)
		self.__var1 = self.__temperature_fine - 128000
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

	def ReadHumidity(self, OSS_Humidity):
		"""Read the raw humidity from the sensor.

			Parameters:
				OSS_Humidity (BME280_OSS): Humidity measurement oversampling

			Returns:
				int: 16 bit raw humidity value
		"""
		self.__Interface.write_byte_data(self.__Address, BME280_REGISTER_CTRL_HUM, OSS_Humidity.value)

		while(self.ConversionRunning()):
			pass
	
		MSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_HUM_MSB)
		LSB = self.__Interface.read_byte_data(self.__Address, BME280_REGISTER_HUM_LSB)

		return ((MSB << 0x08) | LSB)

	def MeasureHumidity(self, OSS_Humidity):
		"""Read the calibrated humidity in %RH from the sensor.

			Parameters:
				OSS_Humidity (BME280_OSS): Humidity measurement oversampling

			Returns:
				float: Humidity value
		"""
		pass