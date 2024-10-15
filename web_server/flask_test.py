import os

from flask import Flask, render_template, request, redirect, url_for, jsonify
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

app = Flask(__name__)

basedir = os.path.abspath(os.path.dirname(__file__))
app.config['SQLALCHEMY_DATABASE_URI'] = f'sqlite:///{os.path.join(basedir, "drone.db")}'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

db = SQLAlchemy(app)

# 定義資料庫模型
class Drone_Status(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)
    battery_remaining = db.Column(db.Float, nullable=False)
    voltage = db.Column(db.Float, nullable=False)
    current = db.Column(db.Float, nullable=False)
    # imu_status = db.Column(db.Boolean, nullable=False)
    # imu_data = db.Column(db.Float, nullable=False)
    # compass_status = db.Column(db.Boolean, nullable=False)




    def __repr__(self):
        return f'<Drone_Status {self.id}>'

# 首頁路由，顯示所有的電池狀態
@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        if request.is_json:
            data = request.get_json()
            battery_remaining = data.get('battery_remaining')
            voltage = data.get('voltage')
            current = data.get('current')
        else:
            battery_remaining = request.form['battery_remaining']
            voltage = request.form['voltage']
            current = request.form['current']
        
        new_status = Drone_Status(battery_remaining=battery_remaining, voltage=voltage, current=current)
        
        try:
            db.session.add(new_status)
            db.session.commit()
            if request.is_json:
                return jsonify({'message': 'Battery status added successfully'}), 200
            else:
                return redirect(url_for('index', updated='true'))
        except:
            return jsonify({'message': 'There was an issue adding the battery status'}), 500 if request.is_json else 'There was an issue adding the battery status'
    
    else:
        updated = request.args.get('updated', 'false')

        statuses = Drone_Status.query.order_by(Drone_Status.id.desc()).all()
        return render_template('index.html', statuses=statuses, updated=updated)

# 刪除電池狀態的路由
@app.route('/delete/<int:id>')
def delete(id):
    status_to_delete = Drone_Status.query.get_or_404(id)

    try:
        db.session.delete(status_to_delete)
        db.session.commit()
        return redirect('/')
    except:
        return 'There was a problem deleting that battery status'

# 更新電池狀態的路由
@app.route('/update/<int:id>', methods=['GET', 'POST'])
def update(id):
    status = Drone_Status.query.get_or_404(id)

    if request.method == 'POST':
        status.battery_remaining = request.form['battery_remaining']
        status.voltage = request.form['voltage']
        status.current = request.form['current']
        try:
            db.session.commit()
            return redirect(url_for('index', updated='true'))
        except:
            return 'There was an issue updating the battery status'
    else:
        return render_template('update.html', status=status)

# 取得所有狀態資料 (JSON 格式)
@app.route('/api/statuses', methods=['GET'])
def get_statuses():
    statuses = Drone_Status.query.order_by(Drone_Status.id).all()
    return jsonify([{
        'id': status.id,
        'timestamp': status.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        'battery_remaining': status.battery_remaining,
        'voltage': status.voltage,
        'current': status.current
    } for status in statuses])

if __name__ == '__main__':
    with app.app_context():
        db.create_all()  # 創建資料庫和表格
    app.run(debug=True)