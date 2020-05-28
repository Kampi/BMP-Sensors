'''
 * BMP180.py
 *
 *  Copyright (C) Daniel Kampert, 2020
 *	Website: www.kampis-elektroecke.de
 *  File info: Module for the BMP180 I2C pressure sensor.

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

BMP180_REGISTER_OUT_XLSB 	= 0xF8
BMP180_REGISTER_OUT_LSB 	= 0xF7
BMP180_REGISTER_OUT_MSB 	= 0xF6
BMP180_REGISTER_CTRL_MEAS	= 0xF4
BMP180_REGISTER_SOFT_RESET	= 0xE0
BMP180_REGISTER_ID 			= 0xD0

BMP180_ID 					= 0x55

BMP180_CMD_RESET			= 0xB6
BMP180_CMD_GETTEMP      	= 0x2E
BMP180_CMD_GETPRESSURE   	= 0x34

BMP180_BIT_SCO				= 0x05

BMP180_ADDRESS 				= 0x77

class BMP180_OSS(Enum):
	X1						= 0x00
	X2						= 0x01
	X4						= 0x02
	X8						= 0x03

class BMP180_Data:
	def __init__(self, Temperature, Pressure):
		self.__Temperature = Temperature
		self.__Pressure = Pressure

	def __repr__(self):
		return "BMP180_Data()"

	def __str__(self):
		return "Temperature : {} Degree Celsius\n" \
		       "Pressure : {} hPa".format(self.__Temperature, self.__Pressure)

	@property
	def temperature(self):
		return self.__Temperature

	@property
	def pressure(self):
		return self.__Pressure

class BMP180:
	def __init__(self, Interface):
		"""Object constructor. Initializes the I2C interface, check the sensor device ID
			and load the calibration coefficients.

			Parameters:
				argument1 (int): I2C interface number.

			Returns:
				None
		"""
		self.__CalibCoef = dict()
		self.__Interface = smbus.SMBus(Interface)

		ID = self.GetID()
		while(not(ID)):
			ID = self.GetID()

		if(ID != BMP180_ID):
			raise ValueError("[ERROR] Wrong device ID: {}!".format(ID))

		self.__LoadCalibrationCoef()

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
		Data = self.__Interface.read_i2c_block_data(BMP180_ADDRESS, Address, 2)

		return (Data[0] << 0x08) | Data[1]

	def __LoadDefaultCalibrationCoef(self):
		"""Load the sensor calibration parameters for the example calculation (see the official datasheet for the parameters).

			Parameters:
				None

			Returns:
				None
		"""
		self.__CalibCoef.update({"AC1": 408})
		self.__CalibCoef.update({"AC2": -72})
		self.__CalibCoef.update({"AC3": -14383})
		self.__CalibCoef.update({"AC4": 32741})
		self.__CalibCoef.update({"AC5": 32757})
		self.__CalibCoef.update({"AC6": 23153})
		self.__CalibCoef.update({"B1": 6190})
		self.__CalibCoef.update({"B2": 4})
		self.__CalibCoef.update({"MB": -32768})
		self.__CalibCoef.update({"MC": -8711})
		self.__CalibCoef.update({"MD": 2868})

	def __LoadCalibrationCoef(self):
		"""Load the sensor calibration parameters.

			Parameters:
				None

			Returns:
				None
		"""
		self.__CalibCoef.update({"AC1": self.__ReadSInt(0xAA)})
		self.__CalibCoef.update({"AC2": self.__ReadSInt(0xAC)})
		self.__CalibCoef.update({"AC3": self.__ReadSInt(0xAE)})
		self.__CalibCoef.update({"AC4": self.__ReadUInt(0xB0)})
		self.__CalibCoef.update({"AC5": self.__ReadUInt(0xB2)})
		self.__CalibCoef.update({"AC6": self.__ReadUInt(0xB4)})
		self.__CalibCoef.update({"B1": self.__ReadSInt(0xB6)})
		self.__CalibCoef.update({"B2": self.__ReadSInt(0xB8)})
		self.__CalibCoef.update({"MB": self.__ReadSInt(0xBA)})
		self.__CalibCoef.update({"MC": self.__ReadSInt(0xBC)})
		self.__CalibCoef.update({"MD": self.__ReadSInt(0xBE)})

	def __ReadTemperature(self):
		"""Start a new temperature measurement and read the raw result from the sensor.

			Parameters:
				None

			Returns:
				int: 16 bit raw temperature value.
		"""
		self.__Interface.write_byte_data(BMP180_ADDRESS, BMP180_REGISTER_CTRL_MEAS, BMP180_CMD_GETTEMP)

		while(self.__Interface.read_byte_data(BMP180_ADDRESS, BMP180_REGISTER_CTRL_MEAS) & (0x01 << BMP180_BIT_SCO)):
			pass

		Data = self.__Interface.read_i2c_block_data(BMP180_ADDRESS, BMP180_REGISTER_OUT_MSB, 2)

		return (Data[0] << 0x08) | Data[1]

	def __ReadPressure(self, OSS):
		"""Start a new pressure measurement and read the raw result from the sensor.

			Parameters:
				OSS (BMP180_OSS): Pressure measurement oversampling.

			Returns:
				int: 20 bit raw pressure value.
		"""
		Data = ((OSS.value & 0x03) << 0x06) | BMP180_CMD_GETPRESSURE

		self.__Interface.write_byte_data(BMP180_ADDRESS, BMP180_REGISTER_CTRL_MEAS, Data)

		while(self.__Interface.read_byte_data(BMP180_ADDRESS, BMP180_REGISTER_CTRL_MEAS) & (0x01 << BMP180_BIT_SCO)):
			pass

		Data = self.__Interface.read_i2c_block_data(BMP180_ADDRESS, BMP180_REGISTER_OUT_MSB, 3)

		return ((Data[0] << 0x10) | (Data[1] << 0x08) | Data[2]) >> (0x08 - OSS.value)

	def __CalcTemperature(self, RawTemp):
		"""Calculate the calibrated temperature from the raw temperature value.

			Parameters:
				RawTemp (int): Raw temperature from the sensor.

			Returns:
				float: Temperature value in Degree Celsius.
		"""
		self.__X1 = ((RawTemp - self.__CalibCoef["AC6"]) * self.__CalibCoef["AC5"]) >> 0x0F
		self.__X2 = int((self.__CalibCoef["MC"] << 0x0B) / (self.__X1 + self.__CalibCoef["MD"]))
		self.__B5 = self.__X1 + self.__X2

		return round(float((self.__B5 + 0x08) >> 0x04) * 0.1, 1)

	def __CalcPressure(self, RawPressure):
		"""Calculate the calibrated pressure from the raw pressure value. A temperature measurement has to be done before to get the correct results.

			Parameters:
				RawPressure (int): Raw pressure from the sensor.

			Returns:
				float: Pressure value in hPa.
		"""
		self.__B6 = self.__B5 - 4000
		self.__X1 = (self.__CalibCoef["B2"]  * ((self.__B6 * self.__B6) >> 0x0B)) >> 0x0B
		self.__X2 = int(self.__CalibCoef["AC2"] * self.__B6 >> 0x0B)
		self.__X3 = self.__X1 + self.__X2
		self.__B3 = ((((self.__CalibCoef["AC1"] << 0x02) + self.__X3) << OSS.value) + 0x02) >> 0x02
		self.__X1 = (self.__CalibCoef["AC3"] * self.__B6) >> 0x0D
		self.__X2 = (self.__CalibCoef["B1"] * (self.__B6 * self.__B6 >> 0x0C)) >> 0x10
		self.__X3 = ((self.__X1 + self.__X2) + 0x02) >> 0x02
		self.__B4 = self.__CalibCoef["AC4"] * (self.__X3 + 32768) >> 0x0F
		self.__B7 = (RawPressure - self.__B3) * (50000 >> OSS.value)
		if(self.__B7 < 0x80000000):
			self.__p = int((self.__B7 << 0x01) / self.__B4)
		else:
			self.__p = int(self.__B7 / self.__B4) << 0x01

		self.__X1 = (self.__p >> 0x08) * (self.__p >> 0x08)
		self.__X1 = (self.__X1 * 3038) >> 0x10
		self.__X2 = (-7357 * self.__p) >> 0x10

		return round((self.__p + ((self.__X1 + self.__X2 + 3791) >> 0x04)), 2) / 100.0

	def GetID(self):
		"""Return the device ID.

			Parameters:
				None

			Returns:
				int: Device ID.
		"""
		return self.__Interface.read_byte_data(BMP180_ADDRESS, BMP180_REGISTER_ID)

	def GetCalibrationCoef(self):
		"""Return a list with the calibration coefficients.

			Parameters:
				None

			Returns:
				list: Calibration coefficients.
		"""
		return self.__CalibCoef

	def Reset(self):
		"""Reset the sensor.

			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.write_byte_data(BMP180_ADDRESS, BMP180_REGISTER_SOFT_RESET, BMP180_CMD_RESET)

	def MeasureTemperature(self):
		"""Read the calibrated temperature in degree Celsius from the sensor.

			Parameters:
				None

			Returns:
				float: Temperature value in Degree Celsius.
		"""
		return self.__CalcTemperature(self.__ReadTemperature())

	def MeasurePressure(self, OSS = BMP180_OSS.X1):
		"""Read the calibrated pressure in hPa from the sensor.

			Parameters:
				OSS (BMP180_OSS): Pressure measurement oversampling

			Returns:
				float: Pressure value in hPa.
		"""
		self.MeasureTemperature()

		return self.__CalcPressure(self.__ReadPressure(OSS))

	def Measure(self, OSS = BMP180_OSS.X1):
		"""Run a complete measurement cycle with each sensor.

			Parameters:
				OSS (BMP180_OSS): Pressure measurement oversampling.

			Returns:
				BMP180_Data: Temperature value in Degree Celsius and pressure value in hPa.
		"""

		return BMP180_Data(self.MeasureTemperature(), self.__CalcPressure(self.__ReadPressure(OSS)))