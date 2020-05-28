'''
 * BME680.py
 *
 *  Copyright (C) Daniel Kampert, 2020
 *	Website: www.kampis-elektroecke.de
 *  File info: Module for the BME680 I2C environment sensor.

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

BME680_REGISTER_CONFIG				= 0x75
BME680_REGISTER_CTRL_MEAS			= 0x74
BME680_REGISTER_STATUS				= 0x73
BME680_REGISTER_CTRL_HUM			= 0x72
BME680_REGISTER_CTRL_GAS_1			= 0x71
BME680_REGISTER_CTRL_GAS_0			= 0x70
BME680_REGISTER_GAS_WAIT_0			= 0x64
BME680_REGISTER_RES_HEAT_0			= 0x5A
BME680_REGISTER_IDAC_HEAT_0			= 0x50
BME680_REGISTER_GAS_R_LSB 			= 0x2B
BME680_REGISTER_GAS_R_MSB 			= 0x2A
BME680_REGISTER_HUM_LSB 			= 0x26
BME680_REGISTER_HUM_MSB 			= 0x25
BME680_REGISTER_TEMP_XLSB 			= 0x24
BME680_REGISTER_TEMP_LSB 			= 0x23
BME680_REGISTER_TEMP_MSB 			= 0x22
BME680_REGISTER_PRESS_XLSB			= 0x21
BME680_REGISTER_PRESS_LSB			= 0x20
BME680_REGISTER_PRESS_MSB			= 0x1F
BME680_REGISTER_EAS_STATUS			= 0x1D
BME680_REGISTER_SOFT_RESET			= 0xE0
BME680_REGISTER_ID					= 0xD0

BME680_ID 							= 0x61

BME680_CMD_RESET					= 0xB6

BME680_BIT_GAS_MEASURE				= 0x06
BME680_BIT_MEASURE					= 0x05
BME680_BIT_RUN_GAS					= 0x04
BME680_BIT_GAS_VALID				= 0x05
BME680_BIT_HEAT_STAB				= 0x04
BME680_BIT_HEAT_OFF					= 0x03

BME680_ADDRESS 						= 0x76

class BME680_OSS(Enum):
	SKIP					= 0x00
	X1						= 0x01
	X2						= 0x02
	X4						= 0x03
	X8						= 0x04
	X16						= 0x05

class BME680_Mode(Enum):
	SLEEP					= 0x00
	FORCED					= 0x01

class BME680_Filter(Enum):
	COEF_0					= 0x00
	COEF_1					= 0x01
	COEF_3					= 0x02
	COEF_7					= 0x03
	COEF_15 				= 0x04
	COEF_31 				= 0x05
	COEF_63 				= 0x06
	COEF_127 				= 0x07

class BME680_Heater:
	def __init__(self, Index, Temperature, Duration, Current = 0):
		self.__Index = min(max(Index, 0), 9)
		self.__Temperature = min(max(Temperature, 200), 400)
		self.__Duration = min(max(Duration, 1), 4032)
		self.__Current = min(max(Current, 0), 16)

	@property
	def index(self):
		return self.__Index

	@property
	def temperature(self):
		return self.__Temperature

	@property
	def duration(self):
		return self.__Duration

	@property
	def current(self):
		return self.__Current

class BME680_Data:
	def __init__(self, Temperature, Pressure, Humidity, GasResistance):
		self.__Temperature = Temperature
		self.__Pressure = Pressure
		self.__Humidity = Humidity
		self.__GasResistance = GasResistance

	def __repr__(self):
		return "BME680_Data()"

	def __str__(self):
		return "Temperature : {} Degree Celsius\n" \
		       "Pressure : {} hPa\n" \
		       "Humidity : {} %RH\n" \
		       "Gas resistance : {} Ohms".format(self.__Temperature, self.__Pressure, self.__Humidity, self.__GasResistance)

	@property
	def temperature(self):
		return self.__Temperature

	@property
	def pressure(self):
		return self.__Pressure

	@property
	def humidity(self):
		return self.__Humidity

	@property
	def gasResistance(self):
		return self.__GasResistance

class BME680:
	__const_array1_int = [2147483647, 2147483647, 2147483647, 2147483647,
							2147483647, 2126008810, 2147483647, 2130303777, 2147483647,
							2147483647, 2143188679, 2136746228, 2147483647, 2126008810,
							2147483647, 2147483647
						]

	__const_array2_int = [4096000000, 2048000000, 1024000000, 512000000,
							255744255, 127110228, 64000000, 32258064,
							16016016, 8000000, 4000000, 2000000,
							1000000, 500000, 250000, 125000
						]

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
		self.__Address = BME680_ADDRESS | ((SDO & 0x01) << 0x00)

		ID = self.GetID()
		while(not(ID)):
			ID = self.GetID()

		if(ID != BME680_ID):
			raise ValueError("[ERROR] Wrong device ID: {}".format(ID))

		self.__LoadCalibrationCoef()
		self.HeaterOff()
		self.SetFilter(BME680_Filter.COEF_0)

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
				Address (char): Register address.

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
		self.__CalibCoef.update({"T1": self.__ReadUInt(0xE9)})
		self.__CalibCoef.update({"T2": self.__ReadSInt(0x8A)})
		self.__CalibCoef.update({"T3": self.__ReadSChar(0x8C)})

		self.__CalibCoef.update({"P1": self.__ReadUInt(0x8E)})
		self.__CalibCoef.update({"P2": self.__ReadSInt(0x90)})
		self.__CalibCoef.update({"P3": self.__ReadSChar(0x92)})
		self.__CalibCoef.update({"P4": self.__ReadSInt(0x94)})
		self.__CalibCoef.update({"P5": self.__ReadSInt(0x96)})
		self.__CalibCoef.update({"P6": self.__ReadSChar(0x99)})
		self.__CalibCoef.update({"P7": self.__ReadSChar(0x98)})
		self.__CalibCoef.update({"P8": self.__ReadSInt(0x9C)})
		self.__CalibCoef.update({"P9": self.__ReadSInt(0x9E)})
		self.__CalibCoef.update({"P10": self.__ReadUChar(0xA0)})

		Data = self.__Interface.read_i2c_block_data(self.__Address, 0xE2, 2)
		self.__CalibCoef.update({"H1": (Data[1] << 0x04) | (Data[0] >> 0x04)})
		Data = self.__Interface.read_i2c_block_data(self.__Address, 0xE1, 2)
		self.__CalibCoef.update({"H2": (Data[0] << 0x04) | (Data[1] >> 0x04)})
		self.__CalibCoef.update({"H3": self.__ReadSChar(0xE4)})
		self.__CalibCoef.update({"H4": self.__ReadSChar(0xE5)})
		self.__CalibCoef.update({"H5": self.__ReadSChar(0xE6)})
		self.__CalibCoef.update({"H6": self.__ReadUChar(0xE7)})
		self.__CalibCoef.update({"H7": self.__ReadSChar(0xE8)})

		self.__CalibCoef.update({"G1": self.__ReadSChar(0xED)})
		self.__CalibCoef.update({"G2": self.__ReadSInt(0xEB)})
		self.__CalibCoef.update({"G3": self.__ReadSChar(0xEE)})

		Data = self.__ReadUChar(0x02)
		self.__CalibCoef.update({"Heat_Range": Data >> 0x04})

		self.__CalibCoef.update({"Heat_Val": self.__ReadSChar(0x00)})

		Data = self.__ReadSChar(0x04) >> 0x04
		self.__CalibCoef.update({"Range_sw_error": Data})

	def __ConversionRunning(self):
		"""Check if a conversion is active.

			Parameters:
				None

			Returns:
				boolean: True when conversion is running.
		"""
		return bool((self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_EAS_STATUS) & (0x01 << BME680_BIT_MEASURE)) >> BME680_BIT_MEASURE)

	def __GasConversionRunning(self):
		"""Check if a gas conversion is active.

			Parameters:
				None

			Returns:
				boolean: True when gas conversion is running.
		"""
		return bool((self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_EAS_STATUS) & (0x01 << BME680_BIT_GAS_MEASURE)) >> BME680_BIT_GAS_MEASURE)

	def __SetHeaterProfile(self, Profile):
		"""Set the heater profile for the gas measurement.

			Parameters:
				Profile (int): Heater profile index.

			Returns:
				None
		"""
		self.HeaterOff()
		Profile = min(max(Profile, 0), 9)
		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_1)
		Data &= 0x10
		Data |= Profile

		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_1, Data)

	def __SetHeaterTemperature(self, Profile, Temperature):
		"""Set the target heater temperature for the gas measurement.

			Parameters:
				Profile (int): Heater profile index.
				Temperature (int): Heater temperatur. Must be between 200 and 400 Degree Celsius.

			Returns:
				None
		"""
		Profile = min(max(Profile, 0), 9)
		Temperature = min(max(Temperature, 200), 400)
		self.__amb_temp = int(self.__CalcTemperature(self.__ReadTemperature(BME680_OSS.X16)) * 100.0)

		self.__var1 = ((self.__amb_temp * self.__CalibCoef["G3"]) // 1000) << 0x09
		self.__var2 = (self.__CalibCoef["G1"] + 784) * (((((self.__CalibCoef["G2"] + 154009) * Temperature * 0x05) // 100) + 3276800) // 10)
		self.__var3 = self.__var1 + (self.__var2 >> 0x01)
		self.__var4 = (self.__var3 / (self.__CalibCoef["Heat_Range"] + 0x04))
		self.__var5 = (131 * self.__CalibCoef["Heat_Val"]) + 65536
		self.__res_heat_x100 = (((self.__var4 / self.__var5) - 250) * 34)
		self.__heat_x = int((self.__res_heat_x100 + 50) / 100)

		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_RES_HEAT_0 + Profile, self.__heat_x)

	def __SetHeaterDuration(self, Profile, Duration):
		"""Set the target heating duration for the gas measurement.

			Parameters:
				Profile (int): Heater profile index. Must be between 0 and 9.
				Duration (int): Heating duration in ms. Must be between 1 and 4032.

			Returns:
				None
		"""
		Profile = min(max(Profile, 0), 9)
		Duration = min(max(Duration, 1), 4032)
		Multiplier = 0

		while(Duration > 63):
			Duration >>= 2;
			Multiplier += 1

		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_GAS_WAIT_0 + Profile, (Multiplier << 0x06) | Duration)

	def __SetHeaterCurrent(self, Profile, Current):
		"""Enable the additional heater current for the current heating profile.

			Parameters:
				Profile (int): Heater profile index. Must be between 0 and 9.
				Current (int): Heating current in mA. Must be between 0 and 16.

			Returns:
				None
		"""
		if(Current > 0):
			Profile = min(max(Profile, 0), 9)
			Current = min(max(Current, 0), 16)

			Current <<= 0x03
			Current -= 1
			Current = int(Current) & 0x7F
			Current <<= 0x01
		else:
			Current = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_1)
			Current &= ~0xFE

		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_IDAC_HEAT_0 + Profile, Current)

	def __ReadTemperature(self, OSS_Temperature):
		"""Start a new temperature measurement and read the raw result from the sensor.

			Parameters:
				OSS_Temperature (BME680_OSS): Temperature measurement oversampling.

			Returns:
				int: 20 bit raw temperature value.
		"""	
		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS)
		Data &= 0x1F
		Data |= OSS_Temperature.value << 0x05
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS, Data)

		while(self.__ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME680_REGISTER_TEMP_MSB, 3)

		return (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

	def __ReadPressure(self, OSS_Pressure):
		"""Start a new pressure measurement and read the raw result from the sensor.

			Parameters:
				OSS_Pressure (BME680_OSS): Pressure measurement oversampling.

			Returns:
				int: 20 bit raw pressure value.
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS)
		Data &= 0xE3
		Data |= OSS_Pressure.value << 0x02
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS, Data)

		while(self.__ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME680_REGISTER_PRESS_MSB, 3)

		return (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)

	def __ReadHumidity(self, OSS_Humidity):
		"""Read the raw humidity from the sensor.

			Parameters:
				OSS_Humidity (BME680_OSS): Humidity measurement oversampling.

			Returns:
				int: 16 bit raw humidity value.
		"""
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_HUM, OSS_Humidity.value)

		while(self.__ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME680_REGISTER_HUM_MSB, 2)

		return (Data[0] << 0x08) | Data[1]

	def __ReadGas(self, Profile):
		"""Read the raw gas values from the sensor.

			Parameters:
				Profile (BME680_Heater): Heater profile.

			Returns:
				int, int: 10 bit gas ADC and 4 bit gas range value.
		"""		
		self.__SetHeaterTemperature(Profile.index, Profile.temperature)
		self.__SetHeaterDuration(Profile.index, Profile.duration)
		self.__SetHeaterProfile(Profile.index)
		self.__SetHeaterCurrent(Profile.index, Profile.current)

		self.HeaterOn()

		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_1)
		Data |= (0x01 << BME680_BIT_RUN_GAS)
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_1, Data)

		while(self.__GasConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME680_REGISTER_GAS_R_MSB, 2)

		Valid = True
		if(not(Data[1] & (0x01 << BME680_BIT_GAS_VALID))):
			Valid = False

		if(not(Data[1] & (0x01 << BME680_BIT_HEAT_STAB))):
			Valid = False
	
		self.__gas_range_r = Data[1] & 0x0F
		self.__gas_r = (Data[0] << 0x02) | ((Data[1] & 0xC0) >> 0x06)

		return self.__gas_r, self.__gas_range_r, Valid

	def __CalcTemperature(self, RawTemp):
		"""Calculate the calibrated temperature from the raw temperature value.

			Parameters:
				RawTemp (int): Raw temperature from the sensor.

			Returns:
				float: Temperature value in Degree Celsius.
		"""
		self.__var1 = (RawTemp >> 0x03) - (self.__CalibCoef["T1"] << 0x01)
		self.__var2 = (self.__var1 * self.__CalibCoef["T2"]) >> 0x0B
		self.__var3 = ((self.__var1 >> 0x01) * (self.__var1 >> 0x01)) >> 0x0C
		self.__var3 = ((self.__var3) * (self.__CalibCoef["T3"] << 0x04)) >> 0x0E
		self.__CalibCoef.update({"t_fine": self.__var2 + self.__var3})

		return round(((self.__CalibCoef["t_fine"] * 0x05 + 0x80) >> 0x08) / 100.0, 4)

	def __CalcPressure(self, RawPressure):
		"""Calculate the calibrated pressure from the raw pressure value. A temperature measurement has to be done before to get the correct results.

			Parameters:
				RawPressure (int): Raw pressure from the sensor.

			Returns:
				float: Pressure value in hPa.
		"""
		self.__var1 = (self.__CalibCoef["t_fine"] >> 0x01) - 64000
		self.__var2 = ((((self.__var1 >> 0x02) * (self.__var1 >> 0x02)) >> 0x0B) * self.__CalibCoef["P6"]) >> 0x02
		self.__var2 = self.__var2 + ((self.__var1 * self.__CalibCoef["P5"]) << 0x01)
		self.__var2 = (self.__var2 >> 0x02) + (self.__CalibCoef["P4"] << 0x10)
		self.__var1 = (((((self.__var1 >> 0x02) * (self.__var1 >> 2)) >> 0x0D) * ((self.__CalibCoef["P3"] << 0x05)) >> 0x03) + ((self.__CalibCoef["P2"] * self.__var1) >> 0x01))
		self.__var1 = self.__var1 >> 0x12
		self.__var1 = ((0x8000 + self.__var1) * self.__CalibCoef["P1"]) >> 0x0F
		self.__press_comp = 0x100000 - RawPressure
		self.__press_comp = ((self.__press_comp - (self.__var2 >> 0x0C)) * 3125)

		if(self.__press_comp >= (0x01 << 0x1E)):
			self.__press_comp = ((self.__press_comp // self.__var1) << 0x01)
		else:
			self.__press_comp = ((self.__press_comp << 0x01) // self.__var1)

		self.__var1 = (self.__CalibCoef["P9"] * (((self.__press_comp >> 0x03) * (self.__press_comp >> 0x03)) >> 0x0D)) >> 0x0C
		self.__var2 = ((self.__press_comp >> 0x02) * self.__CalibCoef["P8"]) >> 0x0D
		self.__var3 = ((self.__press_comp >> 0x08) * (self.__press_comp >> 0x08) * (self.__press_comp >> 0x08) * self.__CalibCoef["P10"]) >> 0x11

		return round(((self.__press_comp) + ((self.__var1 + self.__var2 + self.__var3 + (self.__CalibCoef["P7"] << 0x07)) >> 0x04)) / 100.0, 2)

	def __CalcHumidity(self, RawHumidity):
		"""Calculate the calibrated humidity from the raw humidity value. A temperature measurement has to be done before to get the correct results.

			Parameters:
				RawHumidity (int): Raw humidity from the sensor.

			Returns:
				float: Humidity value in %RH.
		"""
		self.__temp_scaled = ((self.__CalibCoef["t_fine"] * 0x05) + 0x80) >> 0x08
		self.__var1 = (RawHumidity - ((self.__CalibCoef["H1"] * 0x10))) - (((self.__temp_scaled * self.__CalibCoef["H3"]) // 100) >> 0x01)
		self.__var2 = (self.__CalibCoef["H2"] * (((self.__temp_scaled * self.__CalibCoef["H4"]) // 100) + (((self.__temp_scaled * ((self.__temp_scaled * self.__CalibCoef["H5"]) // 100)) >> 0x06) // 100) + 0x4000)) >> 0x0A
		self.__var3 = self.__var1 * self.__var2
		self.__var4 = self.__CalibCoef["H6"] << 0x07
		self.__var4 = (self.__var4 + ((self.__temp_scaled * self.__CalibCoef["H7"]) // 100)) >> 0x04
		self.__var5 = ((self.__var3 >> 0x0E) * (self.__var3 >> 0x0E)) >> 0x0A
		self.__var6 = (self.__var4 * self.__var5) >> 0x01

		return round(((self.__var3 + self.__var6) >> 0x0A) / 4096.0, 2)

	def __CalcGasResistance(self, GasADC, GasRange):
		"""Calculate the gas resistance from the gas ADC and the gas range value.

			Parameters:
				GasADC (int): Raw gas ADC value from the sensor.
				GasRange (int): Raw gas ADC range from the sensor.

			Returns:
				float: Gas resistance in Ohm.
		"""
		self.__var1 = ((1340 + (5 * self.__CalibCoef["Range_sw_error"])) * (self.__const_array1_int[GasRange])) >> 0x10
		self.__var2 = (((GasADC << 0x0F) - 16777216) + self.__var1)
		self.__var3 = ((self.__const_array2_int[GasRange] * self.__var1) >> 0x09)
		calc_gas_res = (float(self.__var3 + (self.__var2 >> 0x01)) / self.__var2)

		return round(calc_gas_res, 2)

	def Reset(self):
		"""Reset the sensor.

			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_SOFT_RESET, BME680_CMD_RESET)

	def HeaterOn(self):
		"""Enable the heater of the gas sensor.

			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_0, 0x00)

	def HeaterOff(self):
		"""Disable the heater of the gas sensor.

			Parameters:
				None

			Returns:
				None
		"""
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_GAS_0, (0x01 << BME680_BIT_HEAT_OFF))

	def GetID(self):
		"""Return the device ID.

			Parameters:
				None

			Returns:
				int: Device ID.
		"""
		return self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_ID)

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
				BME680_Filter: Filter coefficient number.
		"""
		return BME680_Filter((self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CONFIG) >> 0x02) & 0x07)

	def SetFilter(self, Filter):
		"""Set the filter coefficient number for the sensor.

			Parameters:
				Filter (BME680_Filter): Filter coefficient number.

			Returns:
				None
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CONFIG)
		Data &= 0xE3
		Data |= (Filter.value << 0x02)
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CONFIG, Data)

	def GetMode(self):
		"""Get the current device mode from the sensor.

			Parameters:
				None

			Returns:
				BME680_Mode: Current sensor operation mode.
		"""
		return BME680_Mode(self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS) & 0x03)

	def SetMode(self, Mode):
		"""Set the current device mode for the sensor.

			Parameters:
				Mode (BME680_Mode): Sensor operation mode.

			Returns:
				None
				
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS)
		Data &= 0xFC
		Data |= Mode.value
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS, Data)

	def MeasureTemperature(self, OSS_Temperature = BME680_OSS.X1):
		"""Read the calibrated temperature in degree Celsius from the sensor.

			Parameters:
				OSS_Temperature (BMP280_OSS): Temperature measurement oversampling.

			Returns:
				float: Temperature value in Degree Celsius.
		"""
		return self.__CalcTemperature(self.__ReadTemperature(OSS_Temperature))

	def MeasurePressure(self, OSS_Pressure = BME680_OSS.X1, OSS_Temperature = BME680_OSS.X1):
		"""Read the calibrated pressure in hPa from the sensor.

			Parameters:
				OSS_Pressure (BMP280_OSS): Pressure measurement oversampling.
				OSS_Temperature (BMP280_OSS): Temperature measurement oversampling.

			Returns:
				float: Pressure value in hPa.
		"""
		self.MeasureTemperature(OSS_Temperature)

		return self.__CalcPressure(self.__ReadPressure(OSS_Pressure))

	def MeasureHumidity(self, OSS_Humidity = BME680_OSS.X1, OSS_Temperature = BME680_OSS.X1):
		"""Read the calibrated humidity in %RH from the sensor.

			Parameters:
				OSS_Humidity (BME680_OSS): Humidity measurement oversampling.
				OSS_Temperature (BMP280_OSS): Temperature measurement oversampling.

			Returns:
				float: Humidity value in %RH.
		"""
		self.MeasureTemperature(OSS_Temperature)

		return self.__CalcHumidity(self.__ReadHumidity(OSS_Humidity))

	def MeasureGas(self, Profile):
		"""Read the calibrated gas resistance in Ohms from the sensor.

			Parameters:
				Profile (BME680_Heater): Heater profile.

			Returns:
				float: Gas resistance value in Ohms.
				bool: Valid gas result.
		"""
		self.__gas_r, self.__gas_range_r, Valid = self.__ReadGas(Profile)

		return self.__CalcGasResistance(self.__gas_r, self.__gas_range_r), Valid

	def Measure(self, Profile, OSS_Humidity = BME680_OSS.X1, OSS_Pressure = BME680_OSS.X1, OSS_Temperature = BME680_OSS.X1):
		"""Run a complete measurement cycle with each sensor.

			Parameters:
				Profile (BME680_Heater): Heater profile
				OSS_Humidity (BME680_OSS): Humidity measurement oversampling.
				OSS_Pressure (BME680_OSS): Pressure measurement oversampling.
				OSS_Temperature (BME680_OSS): Temperature measurement oversampling.

			Returns:
				BME680_Data: Temperature value in Degree Celsius, Pressure value in hPa, Humidity value in %RH and gas resistance in Ohms.
				bool: Valid gas flag.
		"""
		Data = self.__Interface.read_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS)
		Data &= ~0xFC
		Data |= (OSS_Temperature.value << 0x05) | (OSS_Pressure.value << 0x02)
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_MEAS, Data)
		self.__Interface.write_byte_data(self.__Address, BME680_REGISTER_CTRL_HUM, OSS_Humidity.value)

		while(self.__ConversionRunning()):
			pass

		Data = self.__Interface.read_i2c_block_data(self.__Address, BME680_REGISTER_PRESS_MSB, 8)
		RawPressure = (Data[0] << 0x0C) | (Data[1] << 0x04) | (Data[2] >> 0x04)
		RawTemperature = (Data[3] << 0x0C) | (Data[4] << 0x04) | (Data[5] >> 0x04)
		RawHumidity = (Data[6] << 0x08) | Data[7]

		Gas, Valid = self.MeasureGas(Profile)

		return BME680_Data(self.__CalcTemperature(RawTemperature), self.__CalcPressure(RawPressure), self.__CalcHumidity(RawHumidity), Gas), Valid
