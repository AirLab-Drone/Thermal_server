import pytz
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

db = SQLAlchemy()

# 定義資料庫模型

class SensorGroup(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    group_name = db.Column(db.String(50), nullable=False)  # 感測器組名稱
    timestamp = db.Column(db.DateTime, default=datetime.now(pytz.timezone('Asia/Taipei')))
    sensors = db.relationship('SensorStatus', backref='group', lazy=True)



class SensorStatus(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    sensor_name = db.Column(db.String(50), nullable=False)  # 感測器名稱
    sensor_value = db.Column(db.Boolean, nullable=False)  # 感測器值
    group_id = db.Column(db.Integer, db.ForeignKey('sensor_group.id'), nullable=False)



class Drone_Status(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(
        db.DateTime, default=lambda: datetime.now(pytz.timezone("Asia/Taipei"))
    )
    # 無人機 onboard_control_sensors_present
    gyro = db.Column(db.Boolean, nullable=False)
    accel = db.Column(db.Boolean, nullable=False)
    mag = db.Column(db.Boolean, nullable=False)
    abs_pressure = db.Column(db.Boolean, nullable=False)
    diff_pressure = db.Column(db.Boolean, nullable=False)
    gps = db.Column(db.Boolean, nullable=False)
    optical_flow = db.Column(db.Boolean, nullable=False)
    vision_position = db.Column(db.Boolean, nullable=False)
    laser_position = db.Column(db.Boolean, nullable=False)
    external_ground_truth = db.Column(db.Boolean, nullable=False)
    angular_velocity_control = db.Column(db.Boolean, nullable=False)
    attitude_stabilization = db.Column(db.Boolean, nullable=False)
    yaw_position = db.Column(db.Boolean, nullable=False)
    z_axis_height_control = db.Column(db.Boolean, nullable=False)
    xy_position_control = db.Column(db.Boolean, nullable=False)
    motor_output = db.Column(db.Boolean, nullable=False)
    rc_receiver = db.Column(db.Boolean, nullable=False)
    second_gyro = db.Column(db.Boolean, nullable=False)
    second_accel = db.Column(db.Boolean, nullable=False)
    second_mag = db.Column(db.Boolean, nullable=False)
    geofence = db.Column(db.Boolean, nullable=False)
    ahrs = db.Column(db.Boolean, nullable=False)
    terrain = db.Column(db.Boolean, nullable=False)
    reverse_motor = db.Column(db.Boolean, nullable=False)
    logging = db.Column(db.Boolean, nullable=False)
    battery = db.Column(db.Boolean, nullable=False)
    proximity = db.Column(db.Boolean, nullable=False)
    satellite_communication = db.Column(db.Boolean, nullable=False)
    prearm_check = db.Column(db.Boolean, nullable=False)
    obstacle_avoidance = db.Column(db.Boolean, nullable=False)
    propulsion = db.Column(db.Boolean, nullable=False)
    extended_bit_field = db.Column(db.Boolean, nullable=False)


    def __repr__(self):
        return f"<Drone_Status {self.id}>"
