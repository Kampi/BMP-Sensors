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