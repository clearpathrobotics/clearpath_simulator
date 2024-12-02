#!/usr/bin/env python3

# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.

from clearpath_config.common.types.platform import Platform
from clearpath_config.sensors.types.gps import Garmin18x
from clearpath_config.sensors.types.imu import BaseIMU
from clearpath_generator_common.param.generator import ParamGenerator
from clearpath_generator_common.param.manipulators import ManipulatorParam
from clearpath_generator_common.param.platform import PlatformParam
from clearpath_generator_gz.param.sensors import SensorParam

PLATFORMS = {
    Platform.A200: {'imu': False, 'gps': False},
    Platform.A300: {'imu': False, 'gps': False},
    Platform.J100: {'imu': True, 'gps': True},
    Platform.DD100: {'imu': True, 'gps': False},
    Platform.DD150: {'imu': True, 'gps': False},
    Platform.DO100: {'imu': True, 'gps': False},
    Platform.DO150: {'imu': True, 'gps': False},
    Platform.R100: {'imu': True, 'gps': False},
    Platform.W200: {'imu': True, 'gps': False},
}


class GzParamGenerator(ParamGenerator):

    def generate_sensors(self) -> None:
        for sensor in self.clearpath_config.sensors.get_all_sensors():
            sensor_param = SensorParam(
                sensor,
                self.clearpath_config.get_namespace(),
                self.sensors_params_path)
            sensor_param.generate_config()

    def generate_platform(self) -> None:
        for param in PlatformParam.PARAMETERS:
            platform_param = PlatformParam(
                param,
                self.clearpath_config,
                self.platform_params_path)
            platform_param.generate_parameters(use_sim_time=True)
            platform_param.generate_parameter_file()
        if PLATFORMS[self.clearpath_config.get_platform_model()]['imu']:
            sensor_param = SensorParam(
                BaseIMU(idx=0),
                self.clearpath_config.get_namespace(),
                self.sensors_params_path,
                'sensors'
            )
            sensor_param.generate_config()
        if PLATFORMS[self.clearpath_config.get_platform_model()]['gps']:
            sensor_param = SensorParam(
                Garmin18x(idx=0),
                self.clearpath_config.get_namespace(),
                self.sensors_params_path,
                'sensors',
            )
            sensor_param.generate_config()

    def generate_manipulators(self) -> None:
        # MoveIt
        moveit = ManipulatorParam(
            ManipulatorParam.MOVEIT,
            self.clearpath_config,
            self.manipulators_params_path)
        moveit.generate_parameters(use_sim_time=True)
        moveit.generate_parameter_file()
