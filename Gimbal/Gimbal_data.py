import serial
import time
import string

ser = serial.Serial(port="COM3", baudrate=9600, timeout=1)
time.sleep(2)           #Wait while the arduino initialises
#print("Connection initialised. Starting calibration ...")
ready_for_data = False

while ready_for_data == False:
    data = str(ser.readline())
    if data[2] == '-' or data[2] in string.digits:#May need to change this to be looking for numbers
        ready_for_data = True
        #print("Ready for data transfer")
    else:
        time.sleep(1)  # Waiting for the calibration process to end and the data to start streaming

def get_data():
    roll, pitch, yaw, timePoint = ser.readline().decode().replace('\r\n', '').split(",")
    data_list = [roll, pitch, yaw, timePoint]
    return [float(i) for i in data_list]

#while True:
    #print(get_data())