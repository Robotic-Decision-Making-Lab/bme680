# Copyright 2025, Evan Palmer & Chris Holm
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, RelativeHumidity, Temperature
from smbus2 import SMBus

from bme680_driver.bme680_parameters import bme680_driver


def _read_register(bus: SMBus, address, register, length=1):
    if length == 1:
        return bus.read_byte_data(address, register)
    else:
        return bus.read_i2c_block_data(address, register, length)


def _write_register(bus: SMBus, address, register, data):
    bus.write_byte_data(address, register, data)


def _get_sensor_calibration(bus: SMBus, address: int) -> dict[str, float]:
    calibration = {}

    def u16(register):
        """Read 16-bit unsigned integer (little-endian)"""
        data = _read_register(bus, address, register, 2)
        return data[0] | (data[1] << 8)

    def s16(register):
        """Read 16-bit signed integer"""
        val = u16(register)
        return val if val < 32768 else val - 65536

    def u8(register):
        """Read 8-bit unsigned integer"""
        return _read_register(bus, address, register, 1)

    # Temperature coefficients
    calibration["T1"] = u16(0xE9)
    calibration["T2"] = s16(0x8A)
    calibration["T3"] = s16(0x8C)

    # Pressure coefficients
    calibration["P1"] = u16(0x8E)
    calibration["P2"] = s16(0x90)
    calibration["P3"] = s16(0x92)
    calibration["P4"] = s16(0x94)
    calibration["P5"] = s16(0x96)
    calibration["P6"] = s16(0x99)
    calibration["P7"] = s16(0x98)
    calibration["P8"] = s16(0x9C)
    calibration["P9"] = s16(0x9E)
    calibration["P10"] = u8(0xA0)

    # Humidity coefficients
    calibration["H1"] = (u8(0xE2) << 4) | (u8(0xE3) & 0x0F)
    calibration["H2"] = (u8(0xE4) << 4) | (u8(0xE3) >> 4)
    calibration["H3"] = u8(0xE6)
    calibration["H4"] = u8(0xE7)
    calibration["H5"] = u8(0xE8)
    calibration["H6"] = u8(0xE9)

    return calibration


def _configure_sensor(bus: SMBus, address: int):
    # Set the humidity oversampling to x2
    _write_register(bus, address, 0x72, 0x01)

    # Set the temperature and pressure oversampling to x4
    _write_register(bus, address, 0x74, 0x24)

    # Enable sensor and start measuring
    _write_register(bus, address, 0x74, 0x25)

    # Disable gas sensor heating to reduce self-heating
    _write_register(bus, address, 0x71, 0x00)


def _temp_comp(temp_adc, calib: dict[str, float]) -> tuple[float, float]:
    var1 = (temp_adc / 16384.0 - calib["T1"] / 1024.0) * calib["T2"]
    var2 = ((temp_adc / 131072.0 - calib["T1"] / 8192.0) ** 2) * calib["T3"]
    var3 = 4.7 * 5120.0
    tfine = (var1 + var2) - var3
    temperature = tfine / 5120.0
    tfine = tfine
    return temperature, tfine


def _press_comp(pres_adc, tfine, calib: dict[str, float]) -> float:
    var1 = (tfine / 2.0) - 64000.0
    var2 = var1 * var1 * calib["P6"] / 32768.0
    var2 += var1 * calib["P5"] * 2.0
    var2 = (var2 / 4.0) + (calib["P4"] * 65536.0)
    var1 = (calib["P3"] * var1 * var1 / 524288.0 + calib["P2"] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * calib["P1"]

    if var1 == 0:
        return 0

    pressure = 1048576.0 - pres_adc
    pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
    var1 = calib["P9"] * pressure * pressure / 2147483648.0
    var2 = pressure * calib["P8"] / 32768.0
    pressure = pressure + (var1 + var2 + calib["P7"]) / 16.0

    return pressure


def _humidity_comp(hum_adc, tfine, calib: dict[str, float]) -> float:
    var1 = tfine - 76800.0
    var2 = (hum_adc - (calib["H4"] * 64.0 + calib["H5"] / 16384.0 * var1)) * (
        calib["H2"]
        / 65536.0
        * (
            1.0
            + calib["H6"] / 67108864.0 * var1 * (1.0 + calib["H3"] / 67108864.0 * var1)
        )
    )
    humidity = var2 * (1.0 - calib["H1"] * var2 / 524288.0)
    return max(min(humidity, 100.0), 0.0)


def _read_sensor(
    bus: SMBus, address: int, calib: dict[str, float]
) -> tuple[float, float, float]:
    data = _read_register(bus, address, 0x1D, 10)
    temp, tfine = _temp_comp((data[5] << 12) | (data[6] << 4) | (data[7] >> 4), calib)
    press = _press_comp((data[2] << 12) | (data[3] << 4) | (data[4] >> 4), tfine, calib)
    humidity = _humidity_comp((data[8] << 8) | data[9], tfine, calib)
    return temp, press, humidity


class BME680Driver(Node):
    def __init__(self):
        super().__init__("bme680")

        # Get the parameters
        self.param_listener = bme680_driver.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.bus = SMBus(self.params.i2c_bus)
        self.address = self.params.i2c_address

        # Initialize the sensor
        self.calibration = _get_sensor_calibration(self.bus, self.address)
        _configure_sensor(self.bus, self.address)

        self.pressure_pub = self.create_publisher(
            FluidPressure, "~/pressure", qos_profile_sensor_data
        )
        self.temp_pub = self.create_publisher(
            Temperature, "~/temperature", qos_profile_sensor_data
        )
        self.humidity_pub = self.create_publisher(
            RelativeHumidity, "~/humidity", qos_profile_sensor_data
        )

        self.timer = self.create_timer(self.params.update_rate, self.read_sensor)

    def read_sensor(self):
        temp, pressure, humidity = _read_sensor(
            self.bus, self.address, self.calibration
        )

        stamp = self.get_clock().now().to_msg()
        frame_id = "bme680"

        pres_msg = FluidPressure()
        pres_msg.header.stamp = stamp
        pres_msg.header.frame_id = frame_id
        pres_msg.fluid_pressure = pressure
        pres_msg.variance = 0.0
        self.pressure_pub.publish(pres_msg)

        temp_msg = Temperature()
        temp_msg.header.stamp = stamp
        temp_msg.header.frame_id = frame_id
        temp_msg.temperature = temp
        temp_msg.variance = 0.0
        self.temp_pub.publish(temp_msg)

        hum_msg = RelativeHumidity()
        hum_msg.header.stamp = stamp
        hum_msg.header.frame_id = frame_id
        hum_msg.relative_humidity = humidity
        hum_msg.variance = 0.0
        self.humidity_pub.publish(hum_msg)


def main(args=None):
    rclpy.init(args=args)
    driver = BME680Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()
