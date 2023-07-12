import subprocess
import time
import serial
# from gpiozero import OutputDevice

arduino = serial.Serial('/dev/cu.usbserial-10', 9600, timeout=.1)
time.sleep(1)  # give the connection a second to settle

# PID configuration
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.2  # Derivative gain
setpoint = 35.0  # Target setpoint

integral = 0.0  # Integral term
prev_error = 0.0  # Previous error term


while True:
    arduino_serial = arduino.readline()
    arduino_serial = temp.decode('utf-8')
    print(arduino_serial)
    temperature = int(temp[11:15])
    print(f'temperatura: {temperature}\n')

    # ================> PID <================
    error = setpoint - temperature  # Calculate the error term

    # Proportional term
    proportional = Kp * error

    # Integral term
    integral += Ki * error

    # Derivative term
    derivative = Kd * (error - prev_error)

    # Calculate the PID output
    output = proportional + integral + derivative

    # Update the previous error for the next iteration
    prev_error = error
