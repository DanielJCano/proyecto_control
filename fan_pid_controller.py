import serial
import time

# Setpoint temperature
setpoint = 30.0  # Set the desired setpoint temperature

# PID Parameters
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.2  # Derivative gain

# Serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with the correct serial port and baud rate

integral = 0.0  # Initialize the integral term
prev_error = 0.0  # Initialize the previous error term

def compute_pid(temperature):
    global integral, prev_error  # Declare the variables as global
    
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
