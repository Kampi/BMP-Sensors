import bmp_sensors

if(__name__ == "__main__"):
	Sensor1 = BMP280(1, False)
	print("BMP280 temperature: {} degree celsius".format(Sensor1.MeasureTemperature(BMP280_OSS.x1)))
	print("BMP280 pressure: {} hPa".format(Sensor1.MeasurePressure(BMP280_OSS.x1, BMP280_OSS.x1)))

	Sensor2 = BMP180(1)
	print("BMP180 temperature: {} degree celsius".format(Sensor2.MeasureTemperature()))
	print("BMP180 pressure: {} hPa".format(Sensor2.MeasurePressure(BMP180_OSS.x1)))
