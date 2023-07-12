import serial
import time

arduino = serial.Serial('COM4', 9600, timeout=1)

# PID configuration
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.2  # Derivative gain
setpoint = 35.0  # Target setpoint

integral = 0.0  # Integral term
prev_error = 0.0  # Previous error term

while True:

    # Read temperature data from Arduino
    arduino_serial = arduino.readline().decode('utf-8').strip()
    print(arduino_serial)
    try:
        temperature = float(arduino_serial.split("|")[1].split("degrees C")[0].strip())
    except:
        temperature = 0.0
    print(f'Temperature: {temperature} degrees C')
    # Send fan speed command to Arduino
    fan_speed = int(input("Enter fan speed (0-255): "))  # Get fan speed from user
    arduino.write(f"{fan_speed}\n".encode())  # Send fan speed command to Arduino
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
    output = proportional + integral + derivative

    # Update the previous error for the next iteration
    prev_error = error
    time.sleep(5)
