# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
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
"""
Base class and registry for per-platform Gazebo launch composition.

`BasePlatformLaunch` defines the contract that each concrete platform launch subclass implements;
`PlatformLaunch` is the registry that maps platform `NAME` strings (matching
`BasePlatformConfig.NAME`) to their corresponding launch subclasses. Subclasses live under
`clearpath_generator_gz.launch.platforms` and self-register at import time.
"""
from clearpath_generator_gz.launch import nodes


class BasePlatformLaunch:
    """
    Base class for per-platform Gazebo launch composition.

    Subclasses set `NAME` to match the corresponding `BasePlatformConfig.NAME` (e.g. `'a200'`)
    and override `get_platform_components` to declare any platform-specific launch components
    (IMU bridge/filter, GPS bridge, etc.). Subclasses self-register against `PlatformLaunch`
    at import time.
    """

    NAME: str = ''

    def __init__(
        self,
        namespace: str,
        robot_name: str,
        platform_params_path: str,
        sensors_params_path: str,
    ) -> None:
        """Store the path arguments required to compose launch components."""
        self.namespace = namespace
        self.robot_name = robot_name
        self.platform_params_path = platform_params_path
        self.sensors_params_path = sensors_params_path

    def get_common_components(self) -> list:
        """Return launch components shared across all platforms (cmd_vel + odom TF bridges)."""
        return [
            nodes.make_cmd_vel_node(self.namespace, self.robot_name),
            nodes.make_odom_base_node(self.namespace, self.robot_name),
        ]

    def get_platform_components(self) -> list:
        """Return launch components specific to this platform. Default: no extras."""
        return []

    def get_components(self) -> list:
        """Return the full list of launch components for this platform."""
        return self.get_common_components() + self.get_platform_components()


class PlatformLaunch:
    """Registry mapping platform `NAME` strings to their concrete `BasePlatformLaunch` subclass."""

    _REGISTRY: dict = {}

    @classmethod
    def register(cls, platform_launch_cls) -> None:
        """Register a concrete BasePlatformLaunch subclass."""
        cls._REGISTRY[platform_launch_cls.NAME] = platform_launch_cls

    @classmethod
    def get(cls, name: str):
        """Return the registered BasePlatformLaunch subclass for the given name."""
        if name not in cls._REGISTRY:
            raise KeyError(
                f'No platform launch registered for "{name}". '
                f'Available: {list(cls._REGISTRY.keys())}'
            )
        return cls._REGISTRY[name]

    @classmethod
    def all_names(cls) -> list:
        """Return list of all registered platform launch names."""
        return list(cls._REGISTRY.keys())
