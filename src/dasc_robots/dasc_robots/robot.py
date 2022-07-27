import ctypes
import platform
import numpy as np
import numpy.typing as npt


def load_library(filename: str = 'libdasc_robot_lib.so',#'./lib/libdascBots.a',
                 mode: dict = None) -> ctypes.cdll:
    """Loads a C++ library specified by filename."""
    if mode is not None:
        return ctypes.CDLL(filename, **mode)  # Maybe this is incorrect
    else:
        return ctypes.cdll.LoadLibrary(filename)


class Robot:
    """Interface to C++ DASCRobot object."""

    def __init__(self,
                 name: str,
                 robot_id: int):
        """Initializes Robot in C++ and obtains corresponding object functions."""
        # From Python 3.8 onwards, there is a reported bug in CDLL.__init__()
        l_mode = dict(winmode=0) if platform.python_version() >= '3.8' else dict()
        self._lib = load_library('./lib/libdasc_robot_lib.so', l_mode)
        self.set_arg_res_types()

        # Initialize robot
        self._obj = self._lib.init(name, robot_id)

    def arm(self) -> bool:
        """Arms the robot.

        Arguments:
            None

        Returns:
            Success / error

        """
        return self._lib.arm(self._obj)

    def disarm(self) -> bool:
        """Disarms the robot.

        Arguments:
            None

        Returns:
            None

        """
        return self._lib.disarm(self._obj)

    def cmd_offboard_mode(self) -> bool:
        """Sets the robot control to offboard mode.

        Arguments
            None

        Returns
            Success / error
            """
        return self._lib.cmdOffboardMode(self._obj)

    def get_world_position(self) -> npt.NDArray:
        """Retrieves the world position from the robot.

        Arguments
            None

        Returns
            x, y, z position in world coordinates

        """
        return np.array(self._lib.getWorldPosition(self._obj))

    def get_world_velocity(self) -> npt.NDArray:
        """Retrieves the world velocity from the robot.

        Arguments
            None

        Returns
            vx, vy, vz velocity in world coordinates

        """
        return np.array(self._lib.getWorldVelocity(self._obj))

    def get_world_acceleration(self) -> npt.NDArray:
        """Retrieves the world acceleration from the robot.

        Arguments
            None

        Returns
            ax, ay, az acceleration in world coordinates

        """
        return np.array(self._lib.getWorldAcceleration(self._obj))

    def get_body_acceleration(self) -> npt.NDArray:
        """Retrieves the body acceleration from the robot.

        Arguments
            None

        Returns
            ax, ay, az acceleration in body coordinates

        """
        return np.array(self._lib.getBodyAcceleration(self._obj))

    def get_body_rate(self) -> npt.NDArray:
        """Retrieves the body angular rate from the robot.

        Arguments
            None

        Returns
            wx, wy, wz body angular rates

        """
        return np.array(self._lib.getBodyRate(self._obj))

    def emergency_stop(self) -> None:
        """Brings the robot to an emergency stop.

        Arguments
            None

        Returns
            None

        """
        return self._lib.emergencyStop(self._obj)

    def get_body_quaternion(self,
                            blocking: bool = False) -> npt.NDArray:
        """Gets the body quaternion state from the robot.

        Arguments
            blocking (optional) -- whether or not the call should be blockiing

        Returns
            array containing a, b, c, d quaternions

        """
        quaternion_array = (4 * ctypes.c_double)()
        self._lib.getBodyQuaternion(self._obj, quaternion_array, blocking)

        return np.array(quaternion_array)

    def set_command_mode(self,
                         mode: str = "position") -> bool:
        """Sets the control mode for the robot (position, velocity, acceleration).

        Arguments
            mode: control mode for robot

        Returns
            Success / error flag

        """
        if mode.lower() == 'position' or mode.lower() == 'pos':
            cmode = 0
        elif mode.lower() == 'velocity' or mode.lower() == 'vel':
            cmode = 1
        elif mode.lower() == 'acceleration' or mode.lower() == 'accel':
            cmode = 2
        else:
            raise ValueError("Unknown vehicle control mode!")

        return self._lib.setCmdMode(self._obj, cmode)

    def command_position(self,
                         pos: npt.NDArray) -> bool:
        """Commands the new position setpoint for the robot.

        Arguments
            pos: (x, y, z)

        Returns
            success / error

        """
        x = ctypes.c_double(pos[0])
        y = ctypes.c_double(pos[1])
        z = ctypes.c_double(pos[2])

        return self._lib.cmdWorldPosition(self._obj, x, y, z, 0, 0)

    def command_velocity(self,
                         vel: npt.NDArray) -> bool:
        """Commands the new velocity setpoint for the robot.

        Arguments
            vel: (vx, vy, vz)

        Returns
            success / error

        """
        vx = ctypes.c_double(vel[0])
        vy = ctypes.c_double(vel[1])
        vz = ctypes.c_double(vel[2])

        return self._lib.cmdWorldVelocity(self._obj, vx, vy, vz, 0, 0)

    def command_acceleration(self,
                             acc: npt.NDArray) -> bool:
        """Commands the new acceleration setpoint for the robot.

        Arguments
            acc: (ax, ay, az)

        Returns
            success / error

        """
        ax = ctypes.c_double(acc[0])
        ay = ctypes.c_double(acc[1])
        az = ctypes.c_double(acc[2])

        return self._lib.cmdWorldAcceleration(self._obj, ax, ay, az, 0, 0)

    def command_atttitude(self,
                          quaternions: npt.NDArray,
                          thrust: float) -> bool:
        """Commands the new attitude setpoint for the robot and body thrust.

        Arguments
            quaternions: (qw, qx, qy, qz)
            thrust: body thrust

        Returns
            success / error

        """
        qw = ctypes.c_double(quaternions[0])
        qx = ctypes.c_double(quaternions[1])
        qy = ctypes.c_double(quaternions[2])
        qz = ctypes.c_double(quaternions[2])

        return self._lib.cmdAttitude(self._obj, qw, qx, qy, qz, ctypes.c_double(thrust))

    def command_angular_rates(self,
                              rates: npt.NDArray,
                              thrust: float) -> bool:
        """Commands the new attitude setpoint for the robot and body thrust.

        Arguments
            rates: (roll, pitch, yaw) rates
            thrust: body thrust

        Returns
            success / error

        """
        roll = ctypes.c_double(rates[0])
        pitch = ctypes.c_double(rates[1])
        yaw = ctypes.c_double(rates[2])

        return self._lib.cmdAttitude(self._obj, roll, pitch, yaw, ctypes.c_double(thrust))

    def set_gps_origin(self,
                       origin: npt.NDArray) -> None:
        """Sets the GPS origin point for robot localization.

        Arguments
            origin: (lat, lon, alt)

        Returns
            None

        """
        lat = ctypes.c_double(origin[0])
        lon = ctypes.c_double(origin[1])
        alt = ctypes.c_double(origin[2])

        self._lib.setGPSGlobalOrigin(self._obj, lat, lon, alt)

    def set_arg_res_types(self) -> None:
        """Sets the argument and return types for the Robot library."""
        self._lib.init.argtypes = [ctypes.c_char, ctypes.c_uint8]
        self._lib.init.restype = ctypes.c_void_p

        self._lib.arm.argtypes = ctypes.c_void_p
        self._lib.arm.restype = ctypes.c_bool
        self._lib.disarm.argtypes = ctypes.c_void_p
        self._lib.disarm.restype = ctypes.c_bool
        self._lib.cmdOffboardMode.argtypes = ctypes.c_void_p
        self._lib.cmdOffboardMode.restype = ctypes.c_bool
        self._lib.getWorldPosition.argtypes = ctypes.c_void_p
        self._lib.getWorldPosition.restype = ctypes.POINTER(ctypes.c_double)
        self._lib.getWorldVelocity.argtypes = ctypes.c_void_p
        self._lib.getWorldVelocity.restype = ctypes.POINTER(ctypes.c_double)
        self._lib.getWorldAcceleration.argtypes = ctypes.c_void_p
        self._lib.getWorldAcceleration.restype = ctypes.POINTER(ctypes.c_double)
        self._lib.getBodyAcceleration.argtypes = ctypes.c_void_p
        self._lib.getBodyAcceleration.restype  = ctypes.POINTER(ctypes.c_double)
        self._lib.getBodyRate.argtypes = ctypes.c_void_p
        self._lib.getBodyRate.restype = ctypes.POINTER(ctypes.c_double)
        self._lib.emergencyStop.argtypes = ctypes.c_void_p
        self._lib.emergencyStop.restype = ctypes.c_void_p

        # Functions with arguments
        self._lib.getBodyQuaternion.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_bool]
        self._lib.getBodyQuaternion.restype = ctypes.c_bool
        self._lib.setCmdMode.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        self._lib.setCmdMode.restype = ctypes.c_bool
        self._lib.cmdWorldPosition.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                              ctypes.c_double, ctypes.c_double]
        self._lib.cmdWorldPosition.restype = ctypes.c_bool
        self._lib.cmdWorldVelocity.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                              ctypes.c_double, ctypes.c_double]
        self._lib.cmdWorldVelocity.restype = ctypes.c_bool
        self._lib.cmdLocalVelocity.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                              ctypes.c_double, ctypes.c_double]
        self._lib.cmdLocalVelocity.restype = ctypes.c_bool
        self._lib.cmdWorldAcceleration.argtypes = [ctypes.c_void_p,ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                                  ctypes.c_double, ctypes.c_double]
        self._lib.cmdWorldAcceleration.restype = ctypes.c_bool
        self._lib.cmdAttitude.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                         ctypes.c_double, ctypes.c_double]
        self._lib.cmdAttitude.restype = ctypes.c_bool
        self._lib.cmdRates.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
                                      ctypes.c_double]
        self._lib.cmdRates.restype = ctypes.c_bool
        self._lib.setGPSGlobalOrigin.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        self._lib.setGPSGlobalOrigin.restype = ctypes.c_void_p





