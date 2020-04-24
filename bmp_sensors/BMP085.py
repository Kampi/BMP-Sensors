'''
 * BMP085.py
 *
 *  Copyright (C) Daniel Kampert, 2020
 *	Website: www.kampis-elektroecke.de
 *  File info: Module for the BMP085 I2C pressure sensor.

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
import time
import smbus
from enum import Enum

BMP085_REGISTER_CTRL_MEAS	= 0xF4
BMP085_REGISTER_OUT_XLSB 	= 0xF8
BMP085_REGISTER_OUT_LSB 	= 0xF7
BMP085_REGISTER_OUT_MSB 	= 0xF6

BMP085_ADDRESS 				= 0x77

BMP085_CMD_GETTEMP      	= 0x2E
BMP085_CMD_GETPRESSURE   	= 0x34

class BMP085_OSS(Enum):
	x1						= 0x01
	x2						= 0x02
	x4						= 0x03
	x8						= 0x04

class BMP280:
	def __init__(self, Interface):
		"""Object constructor. Initializes the I2C interface and load the calibration coefficients.

			Parameters:
				Interface (int): I2C interface number

			Returns:
				None
		"""
		self.__CalibCoef = dict()
		self.__Interface = smbus.SMBus(Interface)
		self.__LoadCalibrationCoef()

	def __del__(self):
		"""Object deconstructor.
			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.close()
		
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
		LSB = self.__Interface.read_byte_data(BMP085_ADDRESS, Address)
		MSB = self.__Interface.read_byte_data(BMP085_ADDRESS, Address + 1)

		return (MSB << 0x08) | LSB

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
		AC1 = self.__ReadSInt(0xAA)
		AC2 = self.__ReadSInt(0xAC)
		AC3 = self.__ReadSInt(0xAE)
		AC4 = self.__ReadUInt(0xB0)
		AC5 = self.__ReadUInt(0xB2)
		AC6 = self.__ReadUInt(0xB4)
		B1 = self.__ReadSInt(0xB6)
		B2 = self.__ReadSInt(0xB8)
		MB = self.__ReadSInt(0xBA)
		MC = self.__ReadSInt(0xBC)
		MD = self.__ReadSInt(0xBE)
	
		self.__CalibCoef.update({"AC1": AC1})
		self.__CalibCoef.update({"AC2": AC2})
		self.__CalibCoef.update({"AC3": AC3})
		self.__CalibCoef.update({"AC4": AC4})
		self.__CalibCoef.update({"AC5": AC5})
		self.__CalibCoef.update({"AC6": AC6})
		self.__CalibCoef.update({"B1": B1})
		self.__CalibCoef.update({"B2": B2})
		self.__CalibCoef.update({"MB": MB})
		self.__CalibCoef.update({"MC": MC})
		self.__CalibCoef.update({"MD": MD})

	def GetCalibrationCoef(self):
		"""Return a list with the calibration coefficients.

			Parameters:
				None

			Returns:
				list: Calibration coefficients
		"""
		return self.__CalibCoef

	def ReadTemperature(self):
		"""Read the raw temperature from the sensor.

			Parameters:
				None

			Returns:
				int: 16 bit raw temperature value
		"""
		# Start a new temperature conversion
		self.__Interface.write_byte_data(BMP085_ADDRESS, BMP085_REGISTER_CTRL_MEAS, BMP085_CMD_GETTEMP)

		time.sleep(0.03)

		# Read out the temperature data
		MSB = self.__Interface.read_byte_data(BMP085_ADDRESS, BMP085_REGISTER_OUT_MSB)
		LSB = self.__Interface.read_byte_data(BMP085_ADDRESS, BMP085_REGISTER_OUT_LSB)

		return (MSB << 0x08) | LSB

	def MeasureTemperature(self):
		"""Read the calibrated temperature in degree Celsius from the sensor.

			Parameters:
				None

			Returns:
				float: Temperature value
		"""
		self.__X1 = ((self.ReadTemperature() - self.__CalibCoef["AC6"]) * self.__CalibCoef["AC5"]) >> 0x0F
		self.__X2 = int((self.__CalibCoef["MC"] << 0x0B) / (self.__X1 + self.__CalibCoef["MD"]))
		self.__B5 = self.__X1 + self.__X2

		return round(float((self.__B5 + 0x08) >> 0x04) * 0.1, 1)

	def ReadPressure(self, OSS):
		"""Read the raw pressure from the sensor.

			Parameters:
				OSS (BMP085_OSS): Pressure measurement oversampling

			Returns:
				int: 16 bit raw pressure value
		"""
		Data = ((OSS.value & 0x03) << 0x06) | BMP085_CMD_GETPRESSURE

		self.__Interface.write_byte_data(BMP085_ADDRESS, BMP085_REGISTER_CTRL_MEAS, Data)

		time.sleep(0.03))

		MSB = self.__Interface.read_byte_data(BMP085_ADDRESS, BMP085_REGISTER_OUT_MLSB)
		LSB = self.__Interface.read_byte_data(BMP085_ADDRESS, BMP085_REGISTER_OUT_LSB)
		XLSB = self.__Interface.read_byte_data(BMP085_ADDRESS, BMP085_REGISTER_OUT_XLSB)

		return ((MSB << 0x10) | (LSB << 0x08) | XLSB) >> (0x08 - OSS.value)

	def MeasurePressure(self, OSS):
		"""Read the calibrated pressure in hPa from the sensor.

			Parameters:
				OSS (BMP085_OSS): Pressure measurement oversampling

			Returns:
				float: Pressure value
		"""
		self.MeasureTemperature()
		self.__B6 = self.__B5 - 4000
		self.__X1 = (self.__CalibCoef["B2"]  * ((self.__B6 * self.__B6) >> 0x0B)) >> 0x0B
		self.__X2 = int(self.__CalibCoef["AC2"] * self.__B6 >> 0x0B)
		self.__X3 = self.__X1 + self.__X2
		self.__B3 = ((((self.__CalibCoef["AC1"] << 0x02) + self.__X3) << OSS.value) + 2) >> 0x02
		self.__X1 = (self.__CalibCoef["AC3"] * self.__B6) >> 0x0D
		self.__X2 = (self.__CalibCoef["B1"] * (self.__B6 * self.__B6 >> 0x0C)) >> 0x20
		self.__X3 = ((self.__X1 + self.__X2) + 0x02) >> 0x02
		self.__B4 = self.__CalibCoef["AC4"] * (self.__X3 + 32768) >> 0x0F
		self.__B7 = (self.ReadPressure(OSS) - self.__B3) * (50000 >> OSS.value)
		if(self.__B7 < 0x80000000):
			self.__p = int((self.__B7 << 0x01) / self.__B4)
		else:
			self.__p = int(self.__B7 / self.__B4) << 0x01

		self.__X1 = (self.__p >> 0x08) * (self.__p >> 0x08)
		self.__X1 = (self.__X1 * 3038) >> 0x20
		self.__X2 = (-7357 * self.__p) >> 0x20

		return round((self.__p + ((self.__X1 + self.__X2 + 3791) >> 0x04)), 2) / 100.0