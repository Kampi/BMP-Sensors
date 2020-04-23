'''
 * ReadingSensors.py
 *
 *  Copyright (C) Daniel Kampert, 2020
 *	Website: www.kampis-elektroecke.de
 *  File info: Example application for the Python I2C pressure sensor module.

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
import bmp_sensors as Sensors

if(__name__ == "__main__"):
	Sensor1 = Sensors.BMP180(1)
	Sensor2 = Sensors.BMP280(1, False)
	Sensor3 = Sensors.BME280(1, False)

	Sensor2.SetFilter(Sensors.BMP280_Filter.OFF)
	Sensor2.SetMode(Sensors.BMP280_Mode.FORCED)
	Sensor2.SetFilter(Sensors.BMP280_Filter.OFF)

	while(True):
		print("BMP180:")
		print("Temperature: {} Degree Celsius".format(Sensor1.MeasureTemperature()))
		print("Pressure: {} hPa".format(Sensor1.MeasurePressure(Sensors.BMP180_OSS.x1)))

		print("BMP280:")
		print("Temperature: {} Degree Celsius".format(Sensor2.MeasureTemperature(Sensors.BMP280_OSS.x1)))
		print("Pressure: {} hPa".format(Sensor2.MeasurePressure(Sensors.BMP280_OSS.x1, Sensors.BMP280_OSS.x1)))

		time.sleep(1)