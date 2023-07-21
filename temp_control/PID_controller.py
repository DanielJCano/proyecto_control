
import serial
import time
import os
import django
os.environ['DJANGO_SETTINGS_MODULE'] = 'temp_control.settings'
django.setup()
from controler.models import Arduino_data
# Raspberrypi serial port configuration
# arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# Mac serial port configuration
arduino = serial.Serial('/dev/tty.usbserial-110', 9600, timeout=1)

# PID configuration
Kp = 1.0  # Proportional gain
Ki = 0.25  # Integral gain
Kd = 0.52  # Derivative gain
setpoint = 35.0  # Target setpoint

integral = 0.0  # Integral term
prev_error = 0.0  # Previous error term
fan_speed = 0  # Fan speed (0-255)
Arduino_data.objects.all().delete()
while True:
    # Read temperature data from Arduino
    arduino_serial = arduino.readline().decode('utf-8').strip()
    print(arduino_serial)
    try:
        temperature = float(arduino_serial[27:32])
        print(temperature)
        # voltage = float(arduino_serial[0:3])
        # print(voltage)
    except ValueError:
        # voltage = 0.0
        temperature = 0.0
        # print('Error reading data from Arduino')
    # temperature = -((voltage - 0.5) * 100)
    print(f'Temperature: {temperature} degrees C')
    # Send fan speed command to Arduino
    # fan_speed = int(input("Enter fan speed (0-255): "))  # Get fan speed from user

    arduino.write(f"{fan_speed}".encode())  # Send fan speed command to Arduino
    # Implement PID control here (omitted for simplicity)
    # ================> PID <================
    error = setpoint - temperature  # Calculate the error term

    # Proportional term
    proportional = Kp * error

    # Integral term
    integral += Ki * error

    # Derivative term
    derivative = Kd * (error - prev_error)

    # Calculate the PID output
    error = proportional + integral + derivative
    fan_speed = error

    print(f'fan_speed: {fan_speed}\n')
    if fan_speed > 255:
        fan_speed = 255
    elif fan_speed < 0:
        fan_speed = 0
    else:
        pass
    # Update the previous error for the next iteration
    prev_error = error
    Arduino_data.objects.create(temperature=int(temperature), fanspeed=int(fan_speed), pid=int(error))

    time.sleep(5)
