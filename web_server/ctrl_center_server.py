import os
import pytz
from flask import Flask, render_template, request, redirect, url_for, jsonify
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
from utils.database import database, Drone_Status


app = Flask(__name__)

basedir = os.path.abspath(os.path.dirname(__file__))
app.config["SQLALCHEMY_DATABASE_URI"] = f'sqlite:///{os.path.join(basedir, "drone_raw_data.db")}'
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False


database.init_app(app)



# 首頁路由，顯示所有的電池狀態
@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        if request.is_json:
            data = request.get_json()
            sensor_health = data.get("sensor_health")
            battery_voltage = data.get("battery_voltage")
            battery_current = data.get("battery_current")
            battery_remaining = data.get("battery_remaining")
            gps_hdop = data.get("gps_hdop")
            gps_satellites_visible = data.get("gps_satellites_visible")
            attitude_roll = data.get("attitude_roll")
            attitude_pitch = data.get("attitude_pitch")
            attitude_yaw = data.get("attitude_yaw")
            servo_output_1 = data.get("servo_output_1")
            servo_output_2 = data.get("servo_output_2")
            servo_output_3 = data.get("servo_output_3")
            servo_output_4 = data.get("servo_output_4")
            servo_output_5 = data.get("servo_output_5")
            servo_output_6 = data.get("servo_output_6")


        # else:
        #     sensor_health = request.form["sensor_health"]
        #     battery_voltage = request.form["battery_voltage"]
        #     battery_current = request.form["battery_current"]
        #     battery_remaining = request.form["battery_remaining"]
        #     gps_hdop = request.form["gps_hdop"]
        #     gps_satellites_visible = request.form["gps_satellites_visible"]
        #     attitude_roll = request.form["attitude_roll"]
        #     attitude_pitch = request.form["attitude_pitch"]
        #     attitude_yaw = request.form["attitude_yaw"]
        #     servo_output_1 = request.form["servo_output_1"]
        #     servo_output_2 = request.form["servo_output_2"]
        #     servo_output_3 = request.form["servo_output_3"]
        #     servo_output_4 = request.form["servo_output_4"]
        #     servo_output_5 = request.form["servo_output_5"]
        #     servo_output_6 = request.form["servo_output_6"]

        new_status = Drone_Status(
            sensor_health=sensor_health,
            battery_voltage=battery_voltage,
            battery_current=battery_current,
            battery_remaining=battery_remaining,
            gps_hdop=gps_hdop,
            gps_satellites_visible=gps_satellites_visible,
            attitude_roll=attitude_roll,
            attitude_pitch=attitude_pitch,
            attitude_yaw=attitude_yaw,
            servo_output_1=servo_output_1,
            servo_output_2=servo_output_2,
            servo_output_3=servo_output_3,
            servo_output_4=servo_output_4,
            servo_output_5=servo_output_5,
            servo_output_6=servo_output_6
        )

        try:
            database.session.add(new_status)
            database.session.commit()
            if request.is_json:
                return jsonify({"message": "Status added successfully"}), 200
            else:
                return redirect(url_for("index", updated="true"))
        except Exception as e:
            print(f"資料庫錯誤: {e}")
            return jsonify(
                {"message": "There was an issue adding the status"}
            ), (
                500
                if request.is_json
                else "There was an issue adding the status"
            )

    else:
        updated = request.args.get("updated", "false")

        statuses = Drone_Status.query.order_by(Drone_Status.id.desc()).all()
        return render_template("index.html", statuses=statuses, updated=updated)




# 刪除電池狀態的路由
@app.route("/delete/<int:id>")
def delete(id):
    status_to_delete = Drone_Status.query.get_or_404(id)

    try:
        database.session.delete(status_to_delete)
        database.session.commit()
        return redirect("/")
    except Exception as e:
        print(f"刪除錯誤: {e}")
        return "There was a problem deleting that status"

# 更新電池狀態的路由
@app.route("/update/<int:id>", methods=["GET", "POST"])
def update(id):
    status = Drone_Status.query.get_or_404(id)

    if request.method == "POST":
        status.sensor_health = request.form["sensor_health"]
        status.battery_voltage = request.form["battery_voltage"]
        status.battery_current = request.form["battery_current"]
        status.battery_remaining = request.form["battery_remaining"]
        status.gps_hdop = request.form["gps_hdop"]
        status.gps_satellites_visible = request.form["gps_satellites_visible"]
        status.attitude_roll = request.form["attitude_roll"]
        status.attitude_pitch = request.form["attitude_pitch"]
        status.attitude_yaw = request.form["attitude_yaw"]
        status.servo_output_1 = request.form["servo_output_1"]
        status.servo_output_2 = request.form["servo_output_2"]
        status.servo_output_3 = request.form["servo_output_3"]
        status.servo_output_4 = request.form["servo_output_4"]
        status.servo_output_5 = request.form["servo_output_5"]
        status.servo_output_6 = request.form["servo_output_6"]
        try:
            database.session.commit()
            return redirect(url_for("index", updated="true"))
        except Exception as e:
            print(f"更新錯誤: {e}")
            return "There was an issue updating the status"
    else:
        return render_template("update.html", status=status)

# 取得所有狀態資料 (JSON 格式)
@app.route("/api/statuses", methods=["GET"])
def get_statuses():
    statuses = Drone_Status.query.order_by(Drone_Status.id).all()
    return jsonify(
        [
            {
                "id": status.id,
                "timestamp": status.timestamp.strftime("%Y-%m-%d %H:%M:%S"),
                "sensor_health": status.sensor_health,
                "battery_voltage": status.battery_voltage,
                "battery_current": status.battery_current,
                "battery_remaining": status.battery_remaining,
                "gps_hdop": status.gps_hdop,
                "gps_satellites_visible": status.gps_satellites_visible,
                "attitude_roll": status.attitude_roll,
                "attitude_pitch": status.attitude_pitch,
                "attitude_yaw": status.attitude_yaw,
                "servo_output_1": status.servo_output_1,
                "servo_output_2": status.servo_output_2,
                "servo_output_3": status.servo_output_3,
                "servo_output_4": status.servo_output_4,
                "servo_output_5": status.servo_output_5,
                "servo_output_6": status.servo_output_6,
            }
            for status in statuses
        ]
    )

if __name__ == "__main__":
    with app.app_context():
        database.create_all()  # 創建資料庫和表格
    app.run(debug=True, host='0.0.0.0', port=5000)
