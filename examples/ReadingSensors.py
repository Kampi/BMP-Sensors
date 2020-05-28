#!/usr/bin/python3
'''
 * ReadingSensors.py
 *
 *  Copyright (C) Daniel Kampert, 2020
 *	Website: www.kampis-elektroecke.de
 *  File info: Example application for the bmp-sensors Python module.

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
import argparse
import bmp_sensors as Sensors

Parser = argparse.ArgumentParser(description = "Bosch environment sensors test programm", formatter_class = argparse.RawTextHelpFormatter)
Parser.add_argument("-d", "--devices", nargs = "+", help = "Specify the used sensors as a list: \n" \
															"1 - Use BMP180\n" \
															"2 - Use BMP280\n" \
															"3 - Use BME280\n" \
															"4 - Use BME680", 
															required = True)
Parser.add_argument("-t", "--time", help = "Delay time in seconds for the gas baseline measurement. Only needed when 'BME680' is used (Default: 1)", default = 1, type = int)
Parser.add_argument("-s", "--samples", help = "Samples for the gas baseline measurement. Only needed when 'BME680' is used (Default: 50).", default = 50, type = int)
args = Parser.parse_args()

if(__name__ == "__main__"):
	HumidityBaseline = 40.0
	TemperatureBaseline = 21.0

	# Initialize different sensors
	if("1" in args.devices):
		Sensor1 = Sensors.BMP180(1)

	if("2" in args.devices):
		Sensor2 = Sensors.BMP280(1, False)

	if("3" in args.devices):
		Sensor3 = Sensors.BME280(1, False)

	if("4" in args.devices):
		Sensor4 = Sensors.BME680(1, True)
		Sensor4Heater = Sensors.BME680_Heater(0, 320, 150)

	# Wait some time after the reset
	time.sleep(1)

	if("2" in args.devices):
		print("Sensor 2 mode: {}".format(Sensor2.GetMode()))
		print("Disable filter for sensor 2...")
		Sensor2.SetFilter(Sensors.BMP280_Filter.OFF)

	if("3" in args.devices):
		print("Sensor 3 mode: {}".format(Sensor3.GetMode()))
		print("Disable filter for sensor 3...")
		Sensor3.SetFilter(Sensors.BME280_Filter.OFF)

	if("4" in args.devices):
		print("Sensor 4 mode: {}".format(Sensor4.GetMode()))
		print("Disable filter for sensor 4...")
		Sensor4.SetFilter(Sensors.BME680_Filter.COEF_0)

	if("4" in args.devices):
		# Get the gas baseline for IAQ calculation
		print("Getting gas baseline for IAQ index...")
		GasList = list()
		i = 0
		while(True):
			Sensor4.SetMode(Sensors.BME680_Mode.FORCED)
			Gas, Valid = Sensor4.MeasureGas(Sensor4Heater)

			if(Valid == True):
				print("	Sample: {}".format(i + 1))
				i += 1
				GasList.append(Gas)

				if(i == args.samples):
					break
			else:
				print("	Invalid sample...")

			time.sleep(args.time)

		GasBaseline = sum(GasList[-args.samples:]) / float(args.samples)

		print("	Temperature baseline {} Degree Celsius".format(TemperatureBaseline))
		print("	Humidity baseline: {} %RH".format(HumidityBaseline))
		print("	Gas baseline: {} Ohms".format(GasBaseline))

		time.sleep(5)

	while(True):
		if("1" in args.devices):
			print("BMP180:")
			#print("Temperature: {} Degree Celsius".format(Sensor1.MeasureTemperature()))
			#print("Pressure: {} hPa".format(Sensor1.MeasurePressure(Sensors.BMP180_OSS.X1)))
			print(Sensor1.Measure())

		if("2" in args.devices):
			print("BMP280:")
			Sensor2.SetMode(Sensors.BMP280_Mode.FORCED)
			#print("Temperature: {} Degree Celsius".format(Sensor2.MeasureTemperature()))
			#print("Pressure: {} hPa".format(Sensor2.MeasurePressure()))
			print(Sensor2.Measure())

		if("3" in args.devices):
			print("BME280:")
			Sensor3.SetMode(Sensors.BME280_Mode.FORCED)
			#print("Temperature: {} Degree Celsius".format(Sensor3.MeasureTemperature()))
			#print("Pressure: {} hPa".format(Sensor3.MeasurePressure()))
			#print("Humidity: {} %RH".format(Sensor3.MeasureHumidity()))
			print(Sensor3.Measure())

		if("4" in args.devices):
			print("BME680:")
			Sensor4.SetMode(Sensors.BME680_Mode.FORCED)
			#print("Temperature: {} Degree Celsius".format(Sensor4.MeasureTemperature()))
			#print("Pressure: {} hPa".format(Sensor4.MeasurePressure()))
			print("Humidity: {} %RH".format(Sensor4.MeasureHumidity()))
			#print("Gas: {} Ohms".format(Sensor4.MeasureGas(Sensor4Heater)))
			Data, Valid = Sensor4.Measure(Sensor4Heater)
			if(Valid == True):
				print(Data)

			# Calculate the IAQ index
			# (based on https://forum.iot-usergroup.de/t/indoor-air-quality-index/416/2)
			if(Data.temperature > 21.0):
				TempRel = (Data.temperature - TemperatureBaseline) / (79.0 / 10.0)
			else:
				TempRel = (Data.temperature - TemperatureBaseline)  / (-41.0 / 10.0)

			if(Data.humidity > 40.0):
				HumRel = (Data.humidity - HumidityBaseline) / (60.0 / 10.0)
			else:
				HumRel = (Data.humidity - HumidityBaseline)  / (-40.0 / 10.0)

			if(Valid == True):
				GasRel = (Data.gasResistance - GasBaseline) / (-GasBaseline / 80.0)

			IAQ = min(max(TempRel * HumRel * GasRel, 0.0), 500.0)

			print("IAQ index: {}".format(IAQ))

		time.sleep(1)