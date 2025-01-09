import datetime


class GyroData:
    """GyroData Class

    This is just a "frame" from the .csv IMU file, just data nothing more

    Don't know if it's a standard, might not work with different IMU.

    A lot of the data is not used so remove them if you have compatibility problem.
    """

    def __init__(self, line) -> None:
        self.timestamp: datetime.datetime= datetime.datetime.strptime(line["timestamp"], '%Y-%m-%d %H:%M:%S.%f')
        self.north_vel	= line["north_vel"]
        self.east_vel	= line["east_vel"]
        self.down_vel	= line["down_vel"]
        self.north_vel_dev	= line["north_vel_dev"]
        self.east_vel_dev	= line["east_vel_dev"]
        self.down_vel_dev	= line["down_vel_dev"]
        self.lat	= line["lat"]
        self.long	= line["long"]
        self.alt	= line["alt"]
        self.undulation	= line["undulation"]
        self.lat_dev	= line["lat_dev"]
        self.long_dev	= line["long_dev"]
        self.alt_dev	= line["alt_dev"]
        self.accel_x	= line["accel_x"]
        self.accel_y	= line["accel_y"]
        self.accel_z	= line["accel_z"]
        self.gyro_x	= line["gyro_x"]
        self.gyro_y	= line["gyro_y"]
        self.gyro_z	= line["gyro_z"]
        self.temp	= line["temp"]
        self.delta_vel_x	= line["delta_vel_x"]
        self.delta_vel_y	= line["delta_vel_y"]
        self.delta_vel_z	= line["delta_vel_z"]
        self.delta_angle_x	= line["delta_angle_x"]
        self.delta_angle_y	= line["delta_angle_y"]
        self.delta_angle_z	= line["delta_angle_z"]
        self.mag_x	= line["mag_x"]
        self.mag_y	= line["mag_y"]
        self.mag_z	= line["mag_z"]
        self.accel_x_1= line["accel_x.1"]
        self.accel_y_1= line["accel_y.1"]
        self.accel_z_1= line["accel_z.1"]
        self.roll	= line["roll"]
        self.pitch	= line["pitch"]
        self.yaw	= line["yaw"]
        self.roll_dev	= line["roll_dev"]
        self.pitch_dev	= line["pitch_dev"]
        self.yaw_dev	= line["yaw_dev"]
        self.w_quat	= line["w_quat"]
        self.x_quat	= line["x_quat"]
        self.y_quat	= line["y_quat"]
        self.z_quat	= line["z_quat"]
        self.roll_dev_1= line["roll_dev.1"]
        self.pitch_dev_1	= line["pitch_dev.1"]
        self.yaw_dev_1= line["yaw_dev.1"]
        self.heave_period	= line["heave_period"]
        self.surge_motion	= line["surge_motion"]
        self.sway_motion	= line["sway_motion"]
        self.heave_motion	= line["heave_motion"]
        self.surge_accel	= line["surge_accel"]
        self.sway_accel	= line["sway_accel"]
        self.heave_accel	= line["heave_accel"]
        self.surge_vel	= line["surge_vel"]
        self.sway_vel	= line["sway_vel"]
        self.heave_vel	= line["heave_vel"]
        self.wave_angle= line["wave_angle"]

    def __str__(self) -> str:
        return str(self.__dict__)