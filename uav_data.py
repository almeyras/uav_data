#!/usr/bin/python3
# sshfs cuatrocmssd:/home/daniel/mavlink ~/sshfsmavlink/
# sudo DiskUtil unmount force ~/mavlink/

import math
from pymavlink import mavutil
import serial
import csv
import os
import time 

serial_mavlink = mavutil.mavserial(device='/dev/ttyACM0', baud=57600)
serial_simcom = serial.Serial(port = '/dev/ttyUSB2', baudrate = 115200, timeout=1)

serial_mavlink.wait_heartbeat() # wait for a heartbeat
print("Connected to system:", serial_mavlink.target_system, ", component:", serial_mavlink.target_component)
serial_mavlink.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (serial_mavlink.target_system, serial_mavlink.target_component))

wait_time = 1
time.sleep(wait_time)

# Arm the system
serial_mavlink.mav.command_long_send(
    serial_mavlink.target_system,
    serial_mavlink.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # Parámetro 1: 1 para armar, 0 para desarmar
    0, 0, 0, 0, 0, 0
)
print("Sistema armado\n")

def request_message_interval(serial_mavlink, message_input: str, frequency_hz: float):
    message_name =  "MAVLINK_MSG_ID_" + message_input  
    message_id = getattr(mavutil.mavlink, message_name)
    serial_mavlink.mav.command_long_send(serial_mavlink.target_system, serial_mavlink.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message_id, 1e6/ frequency_hz, 0, 0, 0, 0, 0)
    print(f"Solicitada mensaxe {message_input} (ID: {message_id}) con éxito.")

def request_simcom_medicion_campo():
    serial_simcom.write(b'AT+CPSI?\r\n') # Medición de campo
    simcom_medicion_campo = serial_simcom.read_until(b'+CPSI: ').decode('utf-8')
    simcom_medicion_campo = serial_simcom.readline().decode('utf-8').strip()
    
    simcom_medicion_campo_valores = simcom_medicion_campo.split(',')
    simcom_medicion_campo_claves = [
        'system_mode', 'operation_mode', 'plmn', 'tac', 'cid', 'pci', 'band', 
        'earfcn', 'dlbw', 'ulbw', 'rssq', 'rssp', 'rssi', 'sinr'
    ]
    simcom_medicion_campo_dict = dict(zip(simcom_medicion_campo_claves, simcom_medicion_campo_valores))    
    return simcom_medicion_campo_dict
    

def request_simcom_gnss():
    serial_simcom.write(b'AT+CGPS=1\r\n') # Activa GPS
    time.sleep(0.3)
    serial_simcom.write(b'AT+CGNSSINFO\r\n') # mide GNSS con datos de número de satélites.

    try:
        buffer = b""
        buffercgnssinfo = None  # Variable para guardar los datos después de "+CGNSSINFO:"

        while True:
            # Ler ata encontrar salto de liña
            line = serial_simcom.read_until(b'\n')
            buffer += line  # Acumular en buffer
            if b"+CGNSSINFO:" in line:
                buffercgnssinfo = line.split(b"+CGNSSINFO:")[1].strip()
                buffercgnssinfo = buffercgnssinfo.decode('utf-8')
                simcom_gnss_claves = [
                    'mode', 'GPS-SVs', 'GLONASS-SVs', 'BEIDOU-SVs', 'lat', 'N/S', 'lon', 
                    'E/W', 'date', 'UTC-time', 'mslalt', 'speed', 'course', 'pdop', 'hdop', 'vdop'
                ]
                simcom_gnss_valores = buffercgnssinfo.split(",")
                simcom_gnss_valores_dict = dict(zip(simcom_gnss_claves, simcom_gnss_valores))
                break 
    
        return simcom_gnss_valores_dict
        
    
    except Exception as e:
        print(f"Error al leer del puerto serie: {e}")
    

# Petición de datos a frecuencia fixa
request_message_interval(serial_mavlink, "GLOBAL_POSITION_INT", 2) # GLOBAL_POSITION_INT (ID: 33)
request_message_interval(serial_mavlink, "SYSTEM_TIME", 2)
request_message_interval(serial_mavlink, "SYS_STATUS", 2)
request_message_interval(serial_mavlink, "GPS_RAW_INT", 2)
request_message_interval(serial_mavlink, "ATTITUDE", 2)
request_message_interval(serial_mavlink, "LOCAL_POSITION_NED", 2)
request_message_interval(serial_mavlink, "GPS_GLOBAL_ORIGIN", 2)
request_message_interval(serial_mavlink, "GLOBAL_POSITION_INT_COV", 2)


print("\n")

try:
    while True:
        msg_02 = serial_mavlink.recv_match(type='SYSTEM_TIME', blocking=True)
        msg_01 = serial_mavlink.recv_match(type='SYS_STATUS', blocking=True)
        msg_24 = serial_mavlink.recv_match(type='GPS_RAW_INT', blocking=True)
        msg_30 = serial_mavlink.recv_match(type='ATTITUDE', blocking=True)
        print("Se non hai GPS FIX, o programa queda bloqueado na liña seguinte. Chamar a Dani para que asome o equipo á ventá")
        msg_32 = serial_mavlink.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        msg_33 = serial_mavlink.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        msg_49 = serial_mavlink.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True)
        msg_63 = serial_mavlink.recv_match(type='GLOBAL_POSITION_INT_COV', blocking=True, timeout=0.5)
        if not msg_63:
            print("Timeout al esperar GLOBAL_POSITION_INT_COV")

        msg_simcom_gnss = request_simcom_gnss()
        msg_simcom_medicion_campo_dict = request_simcom_medicion_campo()

        print(msg_simcom_gnss)
        print(msg_simcom_medicion_campo_dict)

        fieldnames = ['SYSTEM_TIME', 'time_boot_ms', 'time_unix_usec', 'SYS_STATUS', 'load', 'voltage_battery', 'current_battery', 'battery_remaining', 'GPS_RAW_INT', 'fix_type', 'lat', 'lon', 'alt', 'eph', 'epv', 'vel', 'cog', 'satellites_visible', 'alt_ellipsoid', 'h_acc', 'v_acc', 'vel_acc', 'hdg_acc', 'yaw',  'ATTITUDE', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed', 'LOCAL_POSITION_NED', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'GLOBAL_POSITION_INT', 'lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz', 'hdg', 'GPS_GLOBAL_ORIGIN', 'latitude', 'longitude', 'altitude', 'GLOBAL_POSITION_INT_COV', 'lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz', 'covariance',  'SIMCOM_MEDICION_CAMPO', 'system_mode', 'operation_mode', 'plmn', 'tac', 'cid', 'pci', 'band', 'earfcn', 'dlbw', 'ulbw', 'rssq', 'rssp', 'rssi', 'sinr', 'SIMCOM_GNSS', 'mode', 'GPS-SVs', 'GLONASS-SVs', 'BEIDOU-SVs', 'lat', 'N/S', 'lon', 'E/W', 'date', 'UTC-time', 'mslalt', 'speed', 'course', 'pdop', 'hdop', 'vdop'] # Definir  encabezados ficheiro CSV 

        msg_02_dict = msg_02.to_dict() if msg_02 else None # Convertimos o mensaxe a un diccionario
        msg_01_dict = msg_01.to_dict() if msg_01 else None
        msg_24_dict = msg_24.to_dict() if msg_24 else None
        msg_30_dict = msg_30.to_dict() if msg_30 else None
        msg_32_dict = msg_32.to_dict() if msg_32 else None
        msg_33_dict = msg_33.to_dict() if msg_33 else None
        msg_49_dict = msg_49.to_dict() if msg_49 else None
        msg_63_dict = msg_63.to_dict() if msg_63 else None

        csv_filename = 'uav_data.csv'

        if not os.path.exists(csv_filename):
            with open(csv_filename, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()  # Escribe los encabezados si el fichero no existe
          
        row_data = {
                'SYSTEM_TIME' : 'SYSTEM_TIME',
                'time_unix_usec': msg_02_dict.get('time_unix_usec', None),
                'time_boot_ms': msg_02_dict.get('time_boot_ms', None),

                'SYS_STATUS' : 'SYS_STATUS',
                'load' : msg_01_dict.get('load', None),
                'voltage_battery' : msg_01_dict.get('voltage_battery', None),
                'current_battery' : msg_01_dict.get('current_battery', None),
                'battery_remaining' : msg_01_dict.get('battery_remaining', None),

                'GPS_RAW_INT' : 'GPS_RAW_INT',
                'fix_type' : msg_24_dict.get('fix_type', None),
                'lat' : msg_24_dict.get('lat', None),
                'lon' : msg_24_dict.get('lon', None),
                'alt' : msg_24_dict.get('alt', None),
                'eph' : msg_24_dict.get('eph', None),
                'epv' : msg_24_dict.get('epv', None),
                'vel' : msg_24_dict.get('vel', None),
                'cog' : msg_24_dict.get('cog', None),
                'satellites_visible' : msg_24_dict.get('satellites_visible', None),
                'alt_ellipsoid' : msg_24_dict.get('alt_ellipsoid', None),
                'h_acc' : msg_24_dict.get('h_acc', None),
                'v_acc' : msg_24_dict.get('v_acc', None),
                'vel_acc' : msg_24_dict.get('vel_acc', None),
                'hdg_acc' : msg_24_dict.get('hdg_acc', None),
                'yaw' : msg_24_dict.get('yaw', None),
  
                'ATTITUDE' : 'ATTITUDE',
                'roll' : msg_30_dict.get('roll', None), 
                'pitch' : msg_30_dict.get('pitch', None), 
                'yaw' : msg_30_dict.get('yaw', None), 
                'rollspeed' : msg_30_dict.get('rollspeed', None), 
                'pitchspeed' : msg_30_dict.get('pitchspeed', None), 
                'yawspeed' : msg_30_dict.get('yawspeed', None), 

                'LOCAL_POSITION_NED' : 'LOCAL_POSITION_NED',               
                'x' : msg_32_dict.get('x', None), 
                'y' : msg_32_dict.get('y', None),
                'z' : msg_32_dict.get('z', None),
                'vx' : msg_32_dict.get('vx', None),
                'vy' : msg_32_dict.get('vy', None),
                'vz' : msg_32_dict.get('vz', None),  
                    
                'GLOBAL_POSITION_INT' : 'GLOBAL_POSITION_INT',
                'lat': msg_33_dict.get('lat', None),
                'lon': msg_33_dict.get('lon', None),
                'alt': msg_33_dict.get('alt', None),
                'relative_alt': msg_33_dict.get('relative_alt', None),
                'vx': msg_33_dict.get('vx', None),
                'vy': msg_33_dict.get('vy', None),
                'vz': msg_33_dict.get('vz', None),
                'hdg': msg_33_dict.get('hdg', None),
    
                'GPS_GLOBAL_ORIGIN' : 'GPS_GLOBAL_ORIGIN',
                'latitude': msg_49_dict.get('latitude', None),
                'longitude': msg_49_dict.get('longitude', None),
                'altitude': msg_49_dict.get('altitude', None),

                'GLOBAL_POSITION_INT_COV' : 'GLOBAL_POSITION_INT_COV',
                'lat': msg_63_dict['lat'] if msg_63_dict else None,
                'lon': msg_63_dict['lon'] if msg_63_dict else None,
                'alt': msg_63_dict['alt'] if msg_63_dict else None,
                'relative_alt': msg_63_dict['relative_alt'] if msg_63_dict else None,
                'vx': msg_63_dict['vx'] if msg_63_dict else None,
                'vy': msg_63_dict['vy'] if msg_63_dict else None,
                'vz': msg_63_dict['vz'] if msg_63_dict else None,
                'covariance': msg_63_dict['covariance'] if msg_63_dict else None,

                
                'SIMCOM_MEDICION_CAMPO' : 'SIMCOM_MEDICION_CAMPO',  
                'system_mode': msg_simcom_medicion_campo_dict.get('system_mode', None) if msg_simcom_medicion_campo_dict else None,
                'operation_mode': msg_simcom_medicion_campo_dict.get('operation_mode', None) if msg_simcom_medicion_campo_dict else None,
                'plmn': msg_simcom_medicion_campo_dict.get('plmn', None) if msg_simcom_medicion_campo_dict else None,
                'tac': msg_simcom_medicion_campo_dict.get('tac', None) if msg_simcom_medicion_campo_dict else None,
                'cid': msg_simcom_medicion_campo_dict.get('cid', None) if msg_simcom_medicion_campo_dict else None,
                'pci': msg_simcom_medicion_campo_dict.get('pci', None) if msg_simcom_medicion_campo_dict else None,
                'band': msg_simcom_medicion_campo_dict.get('band', None) if msg_simcom_medicion_campo_dict else None,
                'earfcn': msg_simcom_medicion_campo_dict.get('earfcn', None) if msg_simcom_medicion_campo_dict else None,
                'dlbw': msg_simcom_medicion_campo_dict.get('dlbw', None) if msg_simcom_medicion_campo_dict else None,
                'ulbw': msg_simcom_medicion_campo_dict.get('ulbw', None) if msg_simcom_medicion_campo_dict else None,
                'rssq': msg_simcom_medicion_campo_dict.get('rssq', None) if msg_simcom_medicion_campo_dict else None,
                'rssp': msg_simcom_medicion_campo_dict.get('rssp', None) if msg_simcom_medicion_campo_dict else None,
                'rssi': msg_simcom_medicion_campo_dict.get('rssi', None) if msg_simcom_medicion_campo_dict else None,                 
                'sinr': msg_simcom_medicion_campo_dict.get('sinr', None) if msg_simcom_medicion_campo_dict else None,

                'SIMCOM_GNSS' : 'SIMCOM_GNSS',  
                'mode': msg_simcom_gnss.get('mode', None) if msg_simcom_gnss else None,
                'GPS-SVs': msg_simcom_gnss.get('GPS-SVs', None) if msg_simcom_gnss else None,
                'GLONASS-SVs': msg_simcom_gnss.get('GLONASS-SVs', None) if msg_simcom_gnss else None,
                'BEIDOU-SVs': msg_simcom_gnss.get('BEIDOU-SVs', None) if msg_simcom_gnss else None,
                'lat': msg_simcom_gnss.get('lat', None) if msg_simcom_gnss else None,
                'N/S': msg_simcom_gnss.get('N/S', None) if msg_simcom_gnss else None,
                'lon': msg_simcom_gnss.get('lon', None) if msg_simcom_gnss else None,
                'E/W': msg_simcom_gnss.get('E/W', None) if msg_simcom_gnss else None,
                'date': msg_simcom_gnss.get('date', None) if msg_simcom_gnss else None,
                'UTC-time': msg_simcom_gnss.get('UTC-time', None) if msg_simcom_gnss else None,
                'mslalt': msg_simcom_gnss.get('mslalt', None) if msg_simcom_gnss else None,
                'speed': msg_simcom_gnss.get('speed', None) if msg_simcom_gnss else None,
                'course': msg_simcom_gnss.get('course', None) if msg_simcom_gnss else None,
                'pdop': msg_simcom_gnss.get('pdop', None) if msg_simcom_gnss else None,
                'hdop': msg_simcom_gnss.get('hdop', None) if msg_simcom_gnss else None,
                'vdop' : msg_simcom_gnss.get('vdop', None) if msg_simcom_gnss else None,               
        }
  
        with open(csv_filename, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writerow(row_data)  # Escribimos la fila de datos en el archivo
    
        time.sleep(1)
        
except KeyboardInterrupt:
    serial_simcom.close()
    print("\n\nBucle detido por usuario.")
    os.system(f"cat {csv_filename} | column -s ',' -t")
    os.system(f"head -n1 {csv_filename} | column -s ',' -t")    
