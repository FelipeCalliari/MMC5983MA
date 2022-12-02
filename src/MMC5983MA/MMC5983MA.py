import abc
import os, sys, time
import enum

try:
    import smbus
except:
    import smbus2 as smbus

from typing import Union, Tuple


class CONSTANTS(enum.Enum):
    DEFAULT_ADDRESS = 0x30
    XOUT0 = 0x00
    XOUT1 = 0x01
    YOUT0 = 0x02
    YOUT1 = 0x03
    ZOUT0 = 0x04
    ZOUT1 = 0x05
    XYZOUT2 = 0x06
    TOUT = 0x07 # The range is -75~125°C, about 0.8°C/LSB, 00000000 stands for -75°C
    STATUS = 0x08
    INTERNAL_CONTROL_0 = 0x09
    INTERNAL_CONTROL_1 = 0x0A
    INTERNAL_CONTROL_2 = 0x0B
    INTERNAL_CONTROL_3 = 0x0C
    PRODUCT_ID_ADDR = 0x2F
    PRODUCT_ID_ANSWER = 0x30

class MeasurementType(enum.Enum):
    SINGLE = 0x00
    CONTINUOUS_1HZ = 0x01
    CONTINUOUS_10HZ = 0x02
    CONTINUOUS_20HZ = 0x03
    CONTINUOUS_50HZ = 0x04
    CONTINUOUS_100HZ = 0x05
    CONTINUOUS_200HZ = 0x06
    CONTINUOUS_1000HZ = 0x07
    
class BandwidthValues(enum.Enum):
    BANDWIDTH_100HZ = 0x00
    BANDWIDTH_200HZ = 0x01
    BANDWIDTH_400HZ = 0x02
    BANDWIDTH_800HZ = 0x03
    
class SetPeriodMode(enum.Enum):
    PERFORM_SET_AFTER_1_MEASUREMENT = 0x00
    PERFORM_SET_AFTER_25_MEASUREMENTS = 0x01
    PERFORM_SET_AFTER_75_MEASUREMENTS = 0x02
    PERFORM_SET_AFTER_100_MEASUREMENTS = 0x03
    PERFORM_SET_AFTER_250_MEASUREMENTS = 0x04
    PERFORM_SET_AFTER_500_MEASUREMENTS = 0x05
    PERFORM_SET_AFTER_1000_MEASUREMENTS = 0x06
    PERFORM_SET_AFTER_2000_MEASUREMENTS = 0x07


class MMC5983MA:
    """Driver for the MMC5983MA 3-axis magnetometer.

    :param ~bus: The I2C bus the MMC5983MA is connected to. Defaults to :const:`0x01`
    :param int address: The I2C device address. Defaults to :const:`0x30`

    **Quickstart: Importing and using the device**

        .. code-block:: python
            import MMC5983MA

            sensor = MMC5983MA.MMC5983MA()
            
            sensor.measurement_type = MeasurementType.SINGLE
            sensor.bandwidth = BandwidthValues.BANDWIDTH_800HZ

            mag_x, mag_y, mag_z = sensor.magnetic16b_raw
    """
    
    _TOUT_TO_CELCIUS = 0.8
    _TOUT_OFFSET = - 75
    _TO_GAUSS_16b = 4096
    _TO_GAUSS_18b = 16384
    _GAUSS_TO_UT = 100
    _HALF_16BITS = int(2**16 / 2)
    _HALF_18BITS = int(2**18 / 2)

    def __init__(self, i2c_bus: int = 0x01, address: int = CONSTANTS.DEFAULT_ADDRESS.value) -> None:
        self._DEVICE_ADDR = address
        self._DEVICE_BUS = i2c_bus
        
        self._MEASUREMENT_SLEEP_TIME = 0.008
        self._STATUS_BANDWIDTH = BandwidthValues.BANDWIDTH_800HZ
        self._STATUS_MEASUREMENT = MeasurementType.SINGLE
        self._STATUS_SET_PERIOD = SetPeriodMode.PERFORM_SET_AFTER_1_MEASUREMENT
        
        self._bus = smbus.SMBus(self._DEVICE_BUS)
        if self._PRODUCT_ID != CONSTANTS.DEFAULT_ADDRESS.value:
            raise RuntimeError("Failed to find MMC5983MA - check your wiring!")
        
        self.reset()
        
        self._BW = 3       # Set bandwidth = 800 Hz
        self._CM_FREQ = 0  # Single measurement

        self._CALIBRATION_X16b = 0
        self._CALIBRATION_Y16b = 0
        self._CALIBRATION_Z16b = 0
        self._CALIBRATION_X18b = 0
        self._CALIBRATION_Y18b = 0
        self._CALIBRATION_Z18b = 0

    @property
    def _PRODUCT_ID(self) -> int:
        return self._bus.read_byte_data(self._DEVICE_ADDR, CONSTANTS.PRODUCT_ID_ADDR.value)

    @property
    def _OTP_READ_DONE(self) -> bool:
        """Indicates the chip was able to successfully read its memory."""
        return bool( self._bus.read_byte_data(self._DEVICE_ADDR, CONSTANTS.STATUS.value) & 0x10 )
    
    @property
    def _MEAS_T_DONE(self) -> bool:
        """Indicates a measurement event of magnetic field is completed.  This bit should be checked before
        reading the output. When the new measurement command is occurred, this bit turns to `0`. When
        the measurement is finished, this bit will remain `1` till next measurement. Writing 1 into this bit
        will clear the corresponding interrupt."""
        return bool( self._bus.read_byte_data(self._DEVICE_ADDR, CONSTANTS.STATUS.value) & 0x02 )

    @_MEAS_T_DONE.setter
    def _MEAS_T_DONE(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.STATUS.value, bool(value) << 1)

    @property
    def _MEAS_M_DONE(self) -> bool:
        """Indicates a measurement event of temperature is completed. When the new measurement
        command is occurred, this bit turns to `0`. When the measurement is finished, this bit will remain
        `1` till next measurement. Writing 1 into this bit will clear the corresponding interrupt."""
        return bool( self._bus.read_byte_data(self._DEVICE_ADDR, CONSTANTS.STATUS.value) & 0x01 )

    @_MEAS_M_DONE.setter
    def _MEAS_M_DONE(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.STATUS.value, bool(value) << 0)

    ## WRITE_ONLY
    def __OTP_READ(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 6)

    _OTP_READ = property(None, __OTP_READ, None,
        """Writing `1` will let the device to read the OTP data again. This bit will be automatically reset to 0
        after the shadow registers for OTP are refreshed.""")

    def __AUTO_SR_EN(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 5)
    
    _AUTO_SR_EN = property(None, __AUTO_SR_EN, None,
        """Writing `1` will enable the feature of automatic set/reset.""")
    
    def __RESET(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 4)
        time.sleep(0.0005)

    _RESET = property(None, __RESET, None,
        """Writing `1` will cause the chip to do the Reset operation, which will allow large reset current to flow
        through the sensor coils for 500ns. This bit is self-cleared at the end of Reset operation.""")
    
    def __SET(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 3)
        time.sleep(0.0005)

    _SET = property(None, __SET, None,
        """Writing `1` will cause the chip to do the Set operation, which will allow large set current to flow
        through the sensor coils for 500ns. This bit is self-cleared at the end of Set operation.""")
    
    def __INT_MEAS_DONE_EN(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 2)

    _INT_MEAS_DONE_EN = property(None, __INT_MEAS_DONE_EN, None,
        """Writing `1` will enable the interrupt for completed measurements. Once a measurement is finished,
        either magnetic field or temperature, an interrupt will be sent to the host.""")
    
    def __TM_T(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 1)

    _TM_T = property(None, __TM_T, None,
        """Take Temperature measurement, set `1` will initiate measurement. This bit will be automatically
        reset to 0 at the end of each measurement. This bit and TM_M cannot be high at the same time.""")
    
    def __TM_M(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_0.value, bool(value) << 0)

    _TM_M = property(None, __TM_M, None,
        """Take magnetic field measurement, set `1` will initiate measurement. This bit will be automatically
        reset to 0 at the end of each measurement.""")
    
    def __SW_RST(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_1.value, bool(value) << 7)
        
    _SW_RST = property(None, __SW_RST, None,
        """Writing `1` will cause the part to reset, similar to power-up. It will clear all registers and also re-
        read OTP as part of its startup routine. The power on time is 10ms.""")

    def __YZ_INHIBIT(self, value) -> None:
        if bool(value):
            value = 0b00011000
        else:
            value = 0
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_1.value, value)

    _YZ_INHIBIT = property(None, __YZ_INHIBIT, None,
        """Writing `1` to the two bits will disable Y and Z channel.""")

    def __X_INHIBIT(self, value) -> None:
         self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_1.value, bool(value) << 2)

    _X_INHIBIT = property(None, __X_INHIBIT, None,
        """Writing `1` will disable X channel.""")

    def __BW(self, value) -> None:
        try:
            self._STATUS_BANDWIDTH = BandwidthValues(value)
        except ValueError:
            raise ValueError(f"Possible values are:\n\t{', '.join([str(i) for i in BandwidthValues])}")
        
        if self._STATUS_BANDWIDTH == BandwidthValues.BANDWIDTH_100HZ:
            self._MEASUREMENT_SLEEP_TIME = 0.008
        if self._STATUS_BANDWIDTH == BandwidthValues.BANDWIDTH_200HZ:
            self._MEASUREMENT_SLEEP_TIME = 0.004
        if self._STATUS_BANDWIDTH == BandwidthValues.BANDWIDTH_400HZ:
            self._MEASUREMENT_SLEEP_TIME = 0.002
        if self._STATUS_BANDWIDTH == BandwidthValues.BANDWIDTH_800HZ:
            self._MEASUREMENT_SLEEP_TIME = 0.001 # 0.0005
            
        value = self._STATUS_BANDWIDTH.value
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_1.value, value & 0x03)

    _BW = property(None, __BW, None,
        """These bandwidth selection bits adjust the length of the decimation filter. They control the duration
        of each measurement.
        ======== ================================================================================================
        VALUE    DESCRIPTION
        ======== ================================================================================================
        `00`     bandwidth = 100 Hz, measurement time = 8.0 ms, RMS noise = 0.4 mG, max output data rate = 50 Hz
        `01`     bandwidth = 200 Hz, measurement time = 4.0 ms, RMS noise = 0.6 mG, max output data rate = 100 Hz
        `10`     bandwidth = 400 Hz, measurement time = 2.0 ms, RMS noise = 0.8 mG, max output data rate = 225 Hz
        `11`     bandwidth = 800 Hz, measurement time = 0.5 ms, RMS noise = 1.2 mG, max output data rate = 580 Hz
        """)

    def __EN_PRD_SET(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_2.value, bool(value) << 7)

    _EN_PRD_SET = property(None, __EN_PRD_SET, None,
        """Writing `1` will enable the feature of periodic set. This feature needs to work with both
        Auto_SR_en and Cmm_en bits set to 1.""")


    def __PRD_SET(self, value) -> None:
        try:
            self._STATUS_SET_PERIOD = SetPeriodMode(value)
        except ValueError:
            raise ValueError(f"Possible values are:\n\t{', '.join([str(i) for i in SetPeriodMode])}")
        
        value = self._STATUS_SET_PERIOD.value
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_2.value, (value & 0x07) << 4)

    _PRD_SET = property(None, __PRD_SET, None,
        """These bits determine how often the chip will do a set operation. The device will perform a
        SET automatically per the setting in below table.
        ======== ================================================================================================
        VALUE    DESCRIPTION
        ======== ================================================================================================
        000         1 measurement
        001        25 measurements
        010        75 measurements
        011       100 measurements
        100       250 measurements
        101       500 measurements
        110      1000 measurements
        111      2000 measurements
        """)

    def __CMM_EN(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_2.value, bool(value) << 3)

    _CMM_EN = property(None, __CMM_EN, None,
        """Writing `1` will enable the continuous mode. In order to enter the continuous mode,
        CM_Freq[2:0] cannot be 000.""")

    def __CM_FREQ(self, value) -> None:
        try:
            self._STATUS_MEASUREMENT = MeasurementType(value)
        except ValueError:
            raise ValueError(f"Possible values are:\n\t{', '.join([str(i) for i in MeasurementType])}")
        
        value = self._STATUS_MEASUREMENT.value
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_2.value, value & 0x07)

    _CM_FREQ = property(None, __CM_FREQ, None,
        """These bits determine how often the chip will take measurements in Continuous
        Measurement Mode. The frequency is based on the assumption that BW[1:0] = 00.
        ============ ================================================================================================
        VALUE        DESCRIPTION
        ============ ================================================================================================
        000          Continuous Measurement Mode is off.
        001          Measurements are made continuously at 1 Hz.
        010          Measurements are made continuously at 10 Hz.
        011          Measurements are made continuously at 20 Hz.
        100          Measurements are made continuously at 50 Hz.
        101          Measurements are made continuously at 100 Hz.
        110 & BW=01  Measurements are made continuously at 200 Hz.
        111 & BW=11  Measurements are made continuously at 1000 Hz.
        """)

    def __SPI_3W(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_3.value, bool(value) << 6)

    _SPI_3W = property(None, __SPI_3W, None,
        """Writing a 1 into this location will put the device into 3-wire SPI mode.""")

    def __ST_ENM(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_3.value, bool(value) << 2)

    _ST_ENM = property(None, __ST_ENM, None,
        """Writing `1` will apply an extra current flowing from the negative end to the positive end of an
        internal coil and result in an extra magnetic field. This feature can be used to check whether the
        sensor has been saturated.""")

    def __ST_ENP(self, value) -> None:
        self._bus.write_byte_data(self._DEVICE_ADDR, CONSTANTS.INTERNAL_CONTROL_3.value, bool(value) << 1)

    _ST_ENP = property(None, __ST_ENP, None,
        """Writing `1` will apply an extra current flowing from the positive end to the negative end of an
        internal coil and result in an extra magnetic field. This feature can be used to check whether the
        sensor has been saturated. 
        """)

    @property
    def _raw_data_all(self) -> Tuple[int, int, int, int, int, int, int, int]:
        return self._bus.read_i2c_block_data(self._DEVICE_ADDR, 0x00, 8)

    @property
    def _raw_data_mag18(self) -> Tuple[int, int, int]:
        data = self._bus.read_i2c_block_data(self._DEVICE_ADDR, 0x00, 7)
        return ( (data[0] << 10) + (data[1] << 2) + ((data[6] >> 6) & 0x3) - self._CALIBRATION_X18b,
                 (data[2] << 10) + (data[3] << 2) + ((data[6] >> 4) & 0x3) - self._CALIBRATION_Y18b,
                 (data[4] << 10) + (data[5] << 2) + ((data[6] >> 2) & 0x3) - self._CALIBRATION_Z18b
                )

    @property
    def _raw_data_mag16(self) -> Tuple[int, int, int]:
        data = self._bus.read_i2c_block_data(self._DEVICE_ADDR, 0x00, 6)
        return ( (data[0] << 8) + data[1] - self._CALIBRATION_X16b,
                 (data[2] << 8) + data[3] - self._CALIBRATION_Y16b,
                 (data[4] << 8) + data[5] - self._CALIBRATION_Z16b
                )

    @property
    def _raw_data_temp(self) -> int:
        return self._bus.read_byte_data(self._DEVICE_ADDR, 0x07)
    
    def reset(self) -> None:
        """Writing `1` will cause the part to reset, similar to power-up. It will clear all registers and also re-
        read OTP as part of its startup routine. The power on time is 10ms."""
        self._SW_RST = True
        time.sleep(0.015)
        
    @property
    def bandwidth(self) -> BandwidthValues:
        return self._STATUS_BANDWIDTH
    
    @bandwidth.setter
    def bandwidth(self, value) -> None:
        self._BW = value
        
    @property
    def measurement_type(self) -> MeasurementType:
        return self._STATUS_MEASUREMENT
    
    @measurement_type.setter
    def measurement_type(self, value) -> None:
        try:
            mode = MeasurementType(value)
        except ValueError:
            raise ValueError(f"Possible values are:\n\t{', '.join([str(i) for i in MeasurementType])}")
        if mode.value == MeasurementType.CONTINUOUS_200HZ:
            if self._STATUS_BANDWIDTH.value < BandwidthValues.BANDWIDTH_200HZ.value:
                self._BW = BandwidthValues.BANDWIDTH_200HZ
        if mode.value == MeasurementType.CONTINUOUS_1000HZ:
            self._BW = BandwidthValues.BANDWIDTH_1000HZ
        self._CM_FREQ = mode
    
    def __set_continuous_measurement_mode(self, value) -> None:
        if bool(value):
            if self._STATUS_MEASUREMENT.value != 0:
                self._CMM_EN = 1
            else:
                raise AttributeError("You must change `measurement_type` before!")
        else:
            self._CM_FREQ = MeasurementType.SINGLE
            self._CMM_EN = 0

    set_continuous_measurement_mode = property(None, __set_continuous_measurement_mode, None,
        """Can be set to `True` or `False`.""")

    @property
    def temperature(self) -> float:
        """Return the temperature in CELCIUS."""
        self._TM_T = True
        
        ts = time.time()
        while not self._MEAS_T_DONE:
            if (time.time() - ts) > 0.05:
                raise TimeoutError("Measurement took too much time without answer, device hang?")
       
        return (self._raw_data_temp * self._TOUT_TO_CELCIUS) + self._TOUT_OFFSET

    def _RUN_SINGLE_MEASUREMENT(self):
        time2sleep = self._MEASUREMENT_SLEEP_TIME / 4
        self._TM_M = 1
        ts = time.time()
        while not self._MEAS_M_DONE:
            time.sleep(time2sleep)
            if (time.time() - ts) > (25 * self._MEASUREMENT_SLEEP_TIME):
                print(f"Time: {time.time()-ts:1.8f} @ start = {ts:1.8f} / stop = {time.time():1.8f}")
                print(f"Measurement time: {self._MEASUREMENT_SLEEP_TIME:1.8f}")
                raise TimeoutError("Measurement took 25x more time than needed, device hang?")

    @property
    def magnetic16b_raw(self) -> Tuple[int, int, int]:
        """The raw magnetometer sensor values."""
        if self._STATUS_MEASUREMENT == MeasurementType.SINGLE:
            self._RUN_SINGLE_MEASUREMENT()

        return self._raw_data_mag16

    @property
    def magnetic18b_raw(self) -> Tuple[int, int, int]:
        """The raw magnetometer sensor values."""
        if self._STATUS_MEASUREMENT == MeasurementType.SINGLE:
            self._RUN_SINGLE_MEASUREMENT()

        return self._raw_data_mag18

    @property
    def magnetic16b_Gauss(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in Gauss that are signed floats."""
        if self._STATUS_MEASUREMENT == MeasurementType.SINGLE:
            self._RUN_SINGLE_MEASUREMENT()

        raw_mag_data = self._raw_data_mag16
        x = ( (raw_mag_data[0] - self._HALF_16BITS) / self._TO_GAUSS_16b)
        y = ( (raw_mag_data[1] - self._HALF_16BITS) / self._TO_GAUSS_16b)
        z = ( (raw_mag_data[2] - self._HALF_16BITS) / self._TO_GAUSS_16b)
        return (x, y, z)

    @property
    def magnetic18b_Gauss(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in Gauss that are signed floats."""
        if self._STATUS_MEASUREMENT == MeasurementType.SINGLE:
            self._RUN_SINGLE_MEASUREMENT()

        raw_mag_data = self._raw_data_mag18
        x = ( (raw_mag_data[0] - self._HALF_18BITS) / self._TO_GAUSS_18b)
        y = ( (raw_mag_data[1] - self._HALF_18BITS) / self._TO_GAUSS_18b)
        z = ( (raw_mag_data[2] - self._HALF_18BITS) / self._TO_GAUSS_18b)
        return (x, y, z)

    @property
    def magnetic16b_uTesla(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats."""
        if self._STATUS_MEASUREMENT == MeasurementType.SINGLE:
            self._RUN_SINGLE_MEASUREMENT()

        raw_mag_data = self._raw_data_mag16
        x = ( (raw_mag_data[0] - self._HALF_16BITS) / self._TO_GAUSS_16b) * self._GAUSS_TO_UT
        y = ( (raw_mag_data[1] - self._HALF_16BITS) / self._TO_GAUSS_16b) * self._GAUSS_TO_UT
        z = ( (raw_mag_data[2] - self._HALF_16BITS) / self._TO_GAUSS_16b) * self._GAUSS_TO_UT
        return (x, y, z)

    @property
    def magnetic18b_uTesla(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats."""
        if self._STATUS_MEASUREMENT == MeasurementType.SINGLE:
            self._RUN_SINGLE_MEASUREMENT()

        raw_mag_data = self._raw_data_mag18
        x = ( (raw_mag_data[0] - self._HALF_18BITS) / self._TO_GAUSS_18b) * self._GAUSS_TO_UT
        y = ( (raw_mag_data[1] - self._HALF_18BITS) / self._TO_GAUSS_18b) * self._GAUSS_TO_UT
        z = ( (raw_mag_data[2] - self._HALF_18BITS) / self._TO_GAUSS_18b) * self._GAUSS_TO_UT
        return (x, y, z)

    @property
    def calibrate(self) -> bool:
        self.__SET(True)
        time.sleep(500e-9) # 500 ns
        x1, y1, z1 = self._raw_data_mag16
        x3, y3, z3 = self._raw_data_mag18
        self.__RESET(True)
        time.sleep(500e-9) # 500 ns
        x2, y2, z2 = self._raw_data_mag16
        x4, y4, z4 = self._raw_data_mag18

        self._CALIBRATION_X16b = (x1 + x2) / 2
        self._CALIBRATION_Y16b = (y1 + y2) / 2
        self._CALIBRATION_Z16b = (z1 + z2) / 2
        self._CALIBRATION_X18b = (x3 + x4) / 2
        self._CALIBRATION_Y18b = (y3 + y4) / 2
        self._CALIBRATION_Z18b = (z3 + z4) / 2

        return True

    

if __name__ == '__main__':
    sensor = MMC5983MA()
    
    sensor.calibrate

    print(f"Setting: {str(MeasurementType.SINGLE)}")
    sensor.measurement_type = MeasurementType.SINGLE
    print(f"Setting: {str(BandwidthValues.BANDWIDTH_800HZ)}")
    sensor.bandwidth = BandwidthValues.BANDWIDTH_800HZ
    
    print(f"Temperature: {sensor.temperature:4.2f} Celcius")
    
    x, y, z = sensor.magnetic18b_Gauss
    print(f"Magnetic field: (x={x:1.7f}, y={y:1.7f}, z={z:1.7f}) Gauss")
    
    print(f"\nUse {'sensor'!r} to DEBUG!")
