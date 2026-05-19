"""Системная информация Raspberry Pi"""
import psutil
import platform
import os
from datetime import datetime


def get_cpu_temperature():
    try:
        temp_file = '/sys/class/thermal/thermal_zone0/temp'
        if os.path.exists(temp_file):
            with open(temp_file, 'r') as f:
                return round(float(f.read().strip()) / 1000, 1)
        temps = psutil.sensors_temperatures()
        if temps:
            for entries in temps.values():
                if entries:
                    return round(entries[0].current, 1)
    except Exception:
        pass
    return None


def get_system_info():
    mem = psutil.virtual_memory()
    disk = psutil.disk_usage('/')
    return {
        'hostname': platform.node(),
        'cpu_percent': psutil.cpu_percent(interval=0.1),
        'cpu_temp': get_cpu_temperature(),
        'mem_percent': mem.percent,
        'mem_used': round(mem.used / (1024**2)),
        'mem_total': round(mem.total / (1024**2)),
        'disk_percent': disk.percent,
        'uptime_hours': round((datetime.now().timestamp() - psutil.boot_time()) / 3600, 1),
        'cpu_freq': round(psutil.cpu_freq().current) if psutil.cpu_freq() else None,
    }
