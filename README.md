# BMP-Sensors

## Table of Contents

- [BMP-Sensors](#bmp-sensors)
  - [Table of Contents](#table-of-contents)
  - [About](#about)
  - [Setup](#setup)
  - [Supported devices](#supported-devices)
  - [History](#history)
  - [Maintainer](#maintainer)

## About

This module contains I2C driver for digital pressure sensors from [Bosch Sensortec](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/).

## Setup

You can install this module via `pip`:

```
$ python3 -m pip install bmp-sensors
```

## Supported devices

| **Part Number** | **Description** |
|---|---|
| BMP180 | Digital pressure (1 Pa resolution) and temperature (0.1 Kelvin resolution) sensor. |
| BMP280 | Digital pressure (0.16 Pa resolution) and temperature (0.01 Kelvin resolution) sensor. |
| BME280 | Digital pressure (0.16 Pa resolution), humidity (0.008% resolution) and temperature (0.01 Kelvin resolution) sensor. |

## History

| **Version** | **Description** |  **Date**  |
|-------------|-----------------|------------|
| 1.0         | First release   | 22.04.2020 |
| 1.1         | Add BME280  |  |

## Maintainer

- [Daniel Kampert](mailto:DanielKampert@kampis-elektroecke.de)
