#!/usr/env python3
# -*- encoding: utf-8 -*-

# ======================================================================
# LSST-VRO / StarDICE
#
# Low level control for the IR Blackbody
# Python3 minimal driver based on official documentation
# https://www.infraredsystems.com/Products/blackbody2100.html
# ======================================================================

# Authors: K. Sommer, J. Cohen-Tanugi 
# Email: <kelian.sommer@umontpellier.fr>, <johann.cohen-tanugi@umontpellier.fr>
# ======================================================================

import serial
from datetime import datetime

MASK = 0xA001 # POLYNOMIAL MASK for CRC
INSTR_ADDR = 0x01 # (DEFAULT IR-301 ADDRESS IS 01) 
READ_CMD = 0x03 # Read Single Register
WRITE_CMD = 0x06 # Write Single Register

# IR-301 REGISTERS
registers = {'model': 0,
             'software_revision': 3,
             'output1a_type': 16,
             'output1b_type': 17,
             'save_to_eeprom': 25,
             'blackbody_temperature': 100,
             'error_status': 101,
             'alarm1_status': 102,
             'power': 103,
             'cold_junction_sensor_value': 104,
             'cold_junction_error_status': 105,
             'alarm2_status': 106,
             'operation_mode': 200,
             'system_error_status': 209,
             'blackbody_set_point': 300,
             'alarm1_low_setpoint': 302,
             'alarm1_high_setpoint': 303,
             'alarm2_low_setpoint': 321,
             'alarm2_high_setpoint': 322}

# CRC computation functions
def cycle(crc):
    bit_count=0
    while bit_count<8:
        if crc&0x0001:
            crc>>=1
            crc ^= MASK
        else:
            crc>>=1
        bit_count +=1
    return crc

def compute_crc(packets):
    crc=0xffff
    for packet in packets:
        b = packet&0xff
        crc ^= b
        crc = cycle(crc)
        packet>>=8
        packet +=1
    crc=hex(crc)  
    if len(crc)==6:
        crc = crc[4::]+crc[2:4]
    else:
        crc = crc[3:]+'0'+crc[2:3]
    return crc

class IR_Blackbody(object):
    """
    Class to control the IR-2101 blackbody with the IR-301 controller
    """
    
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 19200, timeout = 1, debug = False):
        """
        Parameters
        ----------
        port : str
            Port to connect to the focuser (default = '/dev/ttyUSB0')
        baudrate : int
            Baudrate speed for communication in serial (default = 19200)
        timeout : int
            Timeout for response in seconds (default = 1)
        debug : bool
            If True, print additional information (get and set commands in raw format)
        """
                
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.debug = debug

        for r in registers.items():

            def dynamic_method(self, name=r[0], register=r[1], return_length=2):
                res, res_f = self.send_read_cmd(register=register, return_length=return_length)
                print('{} : {}'.format(name, res_f))
                return res, res_f

            setattr(self.__class__, 'read_'+r[0], dynamic_method)
        
        if not self.ser.is_open:
            raise Exception("Device not open, check port!")
        else:
            if self.ping_device() == True:
                print('{} : ==========CONNECTED TO IR BLACKBODY CONTROLLER=========='.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S')))
            else:
                print('{} : ==========ERROR WHILE CONNECTING TO IR BLACKBODY CONTROLLER=========='.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S')))
            
    def ping_device(self):
        """Read device model number to check if connection is established properly

        Returns
        -------
        True : if model is 5280
        False : if operation failed

        """        
        res, res_f = self.send_read_cmd(register=registers['model'], return_length=2)
        if res_f == 5280:
            return True
        else:
            return False

    def send_read_cmd(self, register, return_length):
        """Low-level method to write 'send' commands to the IR-301 controller

        Parameters
        ----------
        register : int
            Modbus register address
        return_length : int
            Bytes to return from the read method
        
        Returns
        -------
        res, res_f : tuple
            Raw response and formatted response

        """
        packets = [INSTR_ADDR, READ_CMD, 0x00, register, 0x00, return_length]

        while max(packets)>256:
            print('Warning : input byte value is above size limit of 256')
            max_value = max(packets)
            max_index = packets.index(max_value)
            c_value = max_value.to_bytes(2, "big")
            packets[max_index - 1] = c_value[0]
            packets[max_index] = c_value[1]
        
        crc = compute_crc(packets=packets)
        packets.append(int(crc[0:2], 16))
        packets.append(int(crc[2:4], 16))
        
        if self.debug:
            print('{} : Send packets to IR-301 : ASCII = {}, INT = {}, HEX = {}'.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S'), bytearray(packets), packets, bytearray(packets).hex()))
            
        self.ser.write(packets)
        res = self.ser.read(32)
        
        if self.debug:
            print('{} : Received packets from IR-301 : {}'.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S'), res))
            
        return res, int.from_bytes(res[3:5], "big")

    def write_blackbody_set_point(self, temperature):
        """Write the controller’s current set point (Register 300 = 0x012C) to provided temperature

        Parameters
        ----------
        temperature : int or float
            Desired temperature of BB in Celsius degrees
        Returns
        -------
        res, res_f : tuple
            Raw response and formatted response
        """
        
        print('{} : Set blackbody temperature to {} °C'.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S'), temperature))
        packets = [INSTR_ADDR, WRITE_CMD, 0x00, registers['blackbody_set_point'], 0x00, temperature*10]

        while max(packets)>256:
            print('Warning : input byte value is above size limit of 256')
            max_value = max(packets)
            max_index = packets.index(max_value)
            c_value = max_value.to_bytes(2, "big")
            packets[max_index - 1] = c_value[0]
            packets[max_index] = c_value[1]

        while min(packets)<0:
            print('Warning : input byte value is below zero')
            min_value = min(packets)
            min_index = packets.index(min_value)
            c_value = min_value.to_bytes(2, "big", signed=True)
            packets[min_index - 1] = c_value[0]
            packets[min_index] = c_value[1]
        
        crc = compute_crc(packets=packets)
        packets.append(int(crc[0:2], 16))
        packets.append(int(crc[2:4], 16))
        
        if self.debug:
            print('{} : Send packets to IR-301 : ASCII = {}, INT = {}, HEX = {}'.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S'), bytearray(packets), packets, bytearray(packets).hex()))
            
        self.ser.write(packets)
        res = self.ser.read(32)
        
        if self.debug:
            print('{} : Received packets from IR-301 : {}'.format(datetime.now().strftime('%Y-%m-%dT%H:%M:%S'), res))
            
        return res, int.from_bytes(res[3:5], "big")
