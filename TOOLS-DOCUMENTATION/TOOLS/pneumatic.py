import asyncio
import logging
import serial
import numpy as np


SERIAL_PORT = "COM3"
BAUDRATE = 115200
RESET = 0x00
CANNON = 0x40
COMPRESSOR = 0x80
VALVES = 0xC0
VALVES_PULSE = 0xD0
TURBINE = 0xE0
LCD_LED = 0xF0

LOGGER = logging.getLogger(__name__)

class PneumaticCommands:
    def __init__(self):
        self._pressure_command = 0
        self._serial = serial.Serial(SERIAL_PORT, BAUDRATE)
        LOGGER.info("initialized serial port for pneumatic board on " + SERIAL_PORT)
    
    # testé et validé
    def reset(self):
        LOGGER.info("Reset nucleo")
        self._serial.write(b'\x01')
        
    def reset_motors(self):
        LOGGER.info("Reset all 5 motors")
        self._serial.write(b'\x02')

    """ inutilisé pour l'instant car la régulation est gérée par la carte pneuma
    def stop_compressor(self):
        LOGGER.info("Stop compressor")
        if self._pressure_command > 0x3F:
            self._pressure_command = 0x3F
        val = RESET | self._pressure_command
        self._serial.write(bytes[val])
    """

    # testé et validé (pas de test des signaux sur la carte)
    def shoot_cannon(self, speed_left, speed_right, speed_top):
        if speed_left > 3:
            speed_left = 3
        if speed_right > 3:
            speed_right = 3
        if speed_top > 3:
            speed_top = 3
        LOGGER.info("Cannon L = " + str(speed_left) + " | R = " + str(speed_right) + " | T = " + str(speed_top))
        _val = CANNON | speed_left << 4 | speed_right << 2 | speed_top
        self._serial.write(bytes([_val]))

    # testé et validé (pas de test des signaux sur la carte)
    def stop_cannon(self):
        LOGGER.info("Stop cannon")
        self._serial.write(b'\x40')

    # testé et validé (pas de test des signaux sur la carte)
    def start_compressor(self, pressure):
        self._pressure_command = pressure
        if self._pressure_command > 0x3F:
            self._pressure_command = 0x3F
        LOGGER.info("Start compressor : pressure command = " + str(0.1 * self._pressure_command) + " bar")
        val = COMPRESSOR | pressure
        self._serial.write(bytes([val]))

    # testé et validé (pas de test des signaux sur la carte)
    def purge_compressor(self):
        LOGGER.info("Purge compressor")
        self._serial.write(b'\x80')

    # testé et validé
    def set_valves(self, a, b, c, e):
        LOGGER.info("Valves : a = " + str(a) + " | b = " + str(b) + " | c = " + str(c) + " | e = " + str(e))
        _val = VALVES | a*8 & 0x08 | b*4 & 0x04 | c*2 & 0x02 | e & 0x01
        self._serial.write(bytes([_val]))
        
    def pulse_valve(self, a, b, c, e):
        LOGGER.info("Pulse on Valves : a = " + str(a) + " | b = " + str(b) + " | c = " + str(c) + " | e = " + str(e))
        val_ = VALVES_PULSE | a*8 & 0x08 | b*4 & 0x04 | c*2 & 0x02 | e & 0x01
        self._serial.write(bytes([val_]))                               

    # testé et validé (pas de tes des signaux sur la carte)
    def start_turbine(self, speed):
        if speed > 3:
            speed = 3
        LOGGER.info("Start turbine, speed = " + str(speed))
        val = TURBINE | speed
        self._serial.write(bytes([val]))

    # testé et validé (pas de tes des signaux sur la carte)
    def stop_turbine(self):
        LOGGER.info("Stop turbine")
        self._serial.write(b'\xE0')
        
    # testé et validé (pas de tes des signaux sur la carte)
    def led_on(self):
        LOGGER.info("LED ON")
        self._serial.write(b'\xF1')
        
    # testé et validé (pas de tes des signaux sur la carte)
    def led_off(self):
        LOGGER.info("LED OFF")
        self._serial.write(b'\xF0')

    # testé et validé (pas de tes des signaux sur la carte)
    def lcd_on(self):
        LOGGER.info("LCD ON")
        self._serial.write(b'\xF2')
        
    # testé et validé (pas de tes des signaux sur la carte) 
    def lcd_off(self):
        LOGGER.info("LCD OFF")
        self._serial.write(b'\xF0')