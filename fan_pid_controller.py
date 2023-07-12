import serial
import time

# Setpoint temperature
setpoint = 30.0  # Set the desired setpoint temperature

# PID Parameters
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.2  # Derivative gain
integral = 0.0  # Integral term
# Serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with the correct serial port and baud rate

def compute_pid(temperature):
    # Compute PID control
    error = setpoint - temperature
    
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
    
    return output

while True:
    # Read temperature from LM35 sensor
    # Implement your code to read the temperature from the LM35 sensor on the Raspberry Pi
    # Read temperature data from Arduino
    arduino_serial = arduino.readline().decode('utf-8').strip()
    print(arduino_serial)
    try:
        temperature = float(arduino_serial[12:15])
        print(f"temperatura: {temperature}")
    except ValueError:
        temperature = 0.0

    print(f'Temperature: {temperature} degrees C')    
    # Compute fan speed control
    fan_speed = compute_pid(temperature)
    
    # Send fan speed command to Arduino
    arduino.write(f"{fan_speed}\n".encode())  # Send fan speed command to Arduino
    
    # Delay for a period of time
    time.sleep(1)  # Adjust the delay time as per your requirement
