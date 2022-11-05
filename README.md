# MMC5958MA

Python implementation for the MMC5983MA 3-axis magnetometer

----------
## **Installation**

It's recommended to install the package's requirements using `pip install -r requirements.txt` before installing the package.

To install run: `python setup.py install`

----------
## **I2C Tools**

To install I2C Tools and the smbus Python library, you may need run:
```
$ sudo apt-get install python-smbus python3-smbus python-dev python3-dev i2c-tools
```

To test if i2ctools is working, run:
```
$ sudo i2cdetect -y 1
```

i2cdetect will display a grid with the I2C devices connected to your system.
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: 30 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

----------
## **Usage**

```python
>>> import MMC5983MA
>>> sensor = MMC5983MA.MMC5983MA()
>>> print(f"Setting: {str(MMC5983MA.BandwidthValues.BANDWIDTH_800HZ)}")
>>> sensor.bandwidth = MMC5983MA.BandwidthValues.BANDWIDTH_800HZ
>>> print(f"Setting: {str(MMC5983MA.MeasurementType.SINGLE)}")
>>> sensor.measurement_type = MMC5983MA.MeasurementType.SINGLE
>>>
>>> print(f"Temperature: {sensor.temperature:4.2f} Celcius")
>>>
>>> x, y, z = sensor.magnetic18b_Gauss
>>> print(f"Magnetic field: (x={x:1.7f}, y={y:1.7f}, z={z:1.7f}) Gauss")
```

----------
## **API documentation**

* MMC5983MA

  * temperature
  * magnetic16b_raw
  * magnetic16b_Gauss
  * magnetic16b_uTesla
  * magnetic18b_raw
  * magnetic18b_Gauss
  * magnetic18b_uTesla
  * bandwidth
  * measurement_type
  * set_continuous_measurement_mode

**Enums**

* MeasurementType:

  SINGLE, CONTINUOUS_1HZ, CONTINUOUS_10HZ, CONTINUOUS_20HZ, CONTINUOUS_50HZ, CONTINUOUS_100HZ, CONTINUOUS_200HZ, CONTINUOUS_1000HZ,
    
* BandwidthValues:

    BANDWIDTH_100HZ, BANDWIDTH_200HZ, BANDWIDTH_400HZ, BANDWIDTH_800HZ
    
* SetPeriodMode:
    
    PERFORM_SET_AFTER_1_MEASUREMENT, PERFORM_SET_AFTER_25_MEASUREMENTS, PERFORM_SET_AFTER_75_MEASUREMENTS, PERFORM_SET_AFTER_100_MEASUREMENTS, PERFORM_SET_AFTER_250_MEASUREMENTS, PERFORM_SET_AFTER_500_MEASUREMENTS, PERFORM_SET_AFTER_1000_MEASUREMENTS, PERFORM_SET_AFTER_2000_MEASUREMENTS,