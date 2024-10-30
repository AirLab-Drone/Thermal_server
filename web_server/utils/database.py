import pytz
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

database = SQLAlchemy()


class Drone_Status(database.Model):
    id = database.Column(database.Integer, primary_key=True)
    date = database.Column(
        database.Date, default=datetime.now(pytz.timezone("Asia/Taipei")).date()
    )
    time = database.Column(
        database.Time,
        default=lambda: datetime.now(pytz.timezone("Asia/Taipei"))
        .time(),
    )

    sensor_health = database.Column(database.Integer, nullable=False)
    battery_voltage = database.Column(database.Float, nullable=False)
    battery_current = database.Column(database.Float, nullable=False)
    battery_remaining = database.Column(database.Integer, nullable=False)
    gps_hdop = database.Column(database.Float, nullable=False)
    gps_satellites_visible = database.Column(database.Integer, nullable=False)
    attitude_roll = database.Column(database.Float, nullable=False)
    attitude_pitch = database.Column(database.Float, nullable=False)
    attitude_yaw = database.Column(database.Float, nullable=False)
    servo_output_1 = database.Column(database.Integer, nullable=False)
    servo_output_2 = database.Column(database.Integer, nullable=False)
    servo_output_3 = database.Column(database.Integer, nullable=False)
    servo_output_4 = database.Column(database.Integer, nullable=False)
    servo_output_5 = database.Column(database.Integer, nullable=False)
    servo_output_6 = database.Column(database.Integer, nullable=False)

    def __repr__(self):
        return f"<Drone_Status {self.id}>"
