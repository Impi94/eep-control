#!/usr/bin/env python3
"""
EEP Control — Веб-интерфейс станка электроэрозионной полировки
Raspberry Pi 5
"""

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from config import Config
from modules.machine import Machine
from modules.process import ProcessManager
from modules.system_info import get_system_info
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.config.from_object(Config)
socketio = SocketIO(app, cors_allowed_origins="*")

# Инициализация
machine = Machine()
process_mgr = ProcessManager()


#СТРАНИЦЫ

@app.route('/')
def index():
    return render_template('index.html', machine_name=Config.MACHINE_NAME)


#API

@app.route('/api/state')
def api_state():
    return jsonify(machine.get_full_state())

@app.route('/api/system')
def api_system():
    return jsonify(get_system_info())

@app.route('/api/log')
def api_log():
    limit = request.args.get('limit', 50, type=int)
    return jsonify(machine.get_log(limit))

@app.route('/api/recipes')
def api_recipes():
    return jsonify(process_mgr.get_recipes())

@app.route('/api/control', methods=['POST'])
def api_control():
    data = request.json
    action = data.get('action')
    
    actions = {
        'start': machine.start,
        'pause': machine.pause,
        'resume': machine.resume,
        'stop': machine.stop,
        'emergency_stop': machine.emergency_stop,
        'reset_errors': machine.reset_errors,
    }
    
    if action in actions:
        result = actions[action]()
        socketio.emit('state_update', machine.get_full_state())
        return jsonify(result)
    
    return jsonify({'success': False, 'error': 'Unknown action'}), 400

@app.route('/api/params/pulse', methods=['POST'])
def api_set_pulse():
    result = machine.set_pulse_params(request.json)
    return jsonify(result)

@app.route('/api/params/process', methods=['POST'])
def api_set_process():
    result = machine.set_process_params(request.json)
    return jsonify(result)

@app.route('/api/preset/<name>', methods=['POST'])
def api_load_preset(name):
    result = machine.load_preset(name)
    return jsonify(result)


#WEBSOCKET

@socketio.on('connect')
def on_connect():
    logger.info('Клиент подключён')
    emit('state_update', machine.get_full_state())

@socketio.on('request_state')
def on_request_state():
    emit('state_update', machine.get_full_state())


# ФОНОВОЕ ОБНОВЛЕНИЕ ТЕЛЕМЕТРИИ

def telemetry_loop():
    """Отправка телеметрии 5 раз в секунду"""
    while True:
        socketio.sleep(0.2)
        data = machine.get_full_state()
        data['system'] = get_system_info()
        socketio.emit('telemetry', data)


# **** ЗАПУСК ****

if __name__ == '__main__':
    logger.info(f"EEP Control | {Config.MACHINE_NAME}")
    logger.info(f"http://{Config.HOST}:{Config.PORT}")
    socketio.start_background_task(telemetry_loop)
    socketio.run(app, host=Config.HOST, port=Config.PORT, debug=Config.DEBUG)
