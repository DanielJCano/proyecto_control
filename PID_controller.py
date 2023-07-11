import subprocess
import time
import serial
# from gpiozero import OutputDevice

arduino = serial.Serial('/dev/cu.usbserial-10', 9600, timeout=.1)
time.sleep(1)  # give the connection a second to settle

# PID configuration
setpoint = 25.0
Kp = 2.0
Ki = 5.0
Kd = 1.0
pid = PID(Kp, Ki, Kd, setpoint)

while True:
    # Read temperature data from Arduino
    line = ser.readline().decode().strip()
    if line.startswith("T:"):
        temperature = float(line[2:])

        # Compute fan speed using PID control
        fan_speed = pid(temperature)

        # Send fan speed command to Arduino
        ser.write(f"{int(fan_speed)}\n".encode())

        # Print temperature and fan speed
        print(f"Temperature: {temperature} °C\tFan Speed: {fan_speed}")
