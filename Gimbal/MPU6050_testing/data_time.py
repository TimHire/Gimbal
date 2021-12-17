import serial
import time

ser = serial.Serial("COM3", baudrate=9600, timeout=1)           #Need to make sure there is a timeout otherwise infinite loop
time.sleep(2)               #Pause while the Arduino initialises

def get_data_time():
    ser.write(b'c')     #Writing 'c' as a byte of data
    roll, pitch, yaw, timePoint = ser.readline().decode().replace('\r\n', '').split(',')
    return roll, pitch, yaw, timePoint

def get_temp():
    ser.write(b't')
    temp, timePoint = ser.readline().decode().replace('\r\n', '').split(',')
    return temp, timePoint

def get_acc():
    ser.write(b'a')
    ax, ay, az, timePoint = ser.readline().decode().replace('\r\n', '').split(',')
    return ax, ay, az, timePoint

def get_gyr():
    ser.write(b'g')
    gx, gy, gz, timePoint = ser.readline().decode().replace('\r\n', '').split(',')
    return gx, gy, gz, timePoint

#print(get_data_time())