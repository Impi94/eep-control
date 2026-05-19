import os
from dotenv import load_dotenv

load_dotenv()

class Config:
    SECRET_KEY = os.getenv('SECRET_KEY', 'eep-machine-secret')
    HOST = os.getenv('HOST', '0.0.0.0')
    PORT = int(os.getenv('PORT', 5000))
    DEBUG = os.getenv('DEBUG', 'True').lower() == 'true'
    
    # затычки параметров, поотм поменяем по факту
    MACHINE_NAME = os.getenv('MACHINE_NAME', 'EEP-01')
    MAX_VOLTAGE = float(os.getenv('MAX_VOLTAGE', 300))
    MAX_CURRENT = float(os.getenv('MAX_CURRENT', 50))
    MAX_PULSE_FREQ = int(os.getenv('MAX_PULSE_FREQ', 100000))
