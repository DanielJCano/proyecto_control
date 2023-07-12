import serial

arduino = serial.Serial('COM4', 9600, timeout=1)

while True:
    arduino_serial = arduino.readline()
    print(arduino_serial)