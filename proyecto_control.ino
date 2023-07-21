// Sensor de temperatura
#define sensorPin A0
// Control de ventilador
const int fanPin = 3; // PWM pin for fan control
const int reading_speed = 4;
// Tachometro para medir la velocidad

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(fanPin, OUTPUT);
  pinMode(reading_speed, INPUT);
}
void loop()
{
  // put your main code here, to run repeatedly:
  int reading_sensor = analogRead(sensorPin);
  float voltage = (reading_sensor)* 4.8 / 1024;
  float temperatureC = (voltage * 100);

  // Receive fan speed command from Python
  if (Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n'); // Read the input string until a newline character is received
    input.trim();                                // Remove leading/trailing whitespace

    if (input.length() > 0)
    {
      int fanSpeed = input.toInt(); // Convert the input string to an integer

      // Ensure the fan speed value is within the valid range
      if (fanSpeed >= 0 && fanSpeed <= 255)
      {
        analogWrite(fanPin, fanSpeed); // Set the fan speed
      }
    }
  }
  Serial.print("Sensor: ");
  Serial.print(reading_sensor);
  Serial.print(" | ");
  Serial.print(voltage);
  Serial.print(" volts | ");
  Serial.print(temperatureC);
  Serial.print(" degrees C | ");
  Serial.print(reading_speed);
  Serial.print(" rpm | ");
  Serial.print(analogRead(fanPin));
  Serial.print(" speed \n");

  delay(5000);
}
