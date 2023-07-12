// Sensor de temperatura
#define sensorPin A7
// Control de ventilador
const int fanPin = 3; // PWM pin for fan control
// Tachometro para medir la velocidad
const int tachPin = 4;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(tachPin, INPUT_PULLUP);
  pinMode(fanPin, OUTPUT);
}

void loop()
{
  // put your main code here, to run repeatedly:
  int reading_sensor = analogRead(sensorPin);
  int reading_speed = digitalRead(tachPin);

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

  float voltage = (reading_sensor) * 5.0 / 1023.0;
  delay(500);
  float temperatureC = voltage * 100;

  Serial.print(voltage);
  Serial.print(" volts | ");
  Serial.print(temperatureC);
  Serial.print(" degrees C | ");
  Serial.print(reading_speed);
  Serial.print(" rpm | ");
  Serial.print(analogRead(fanPin));
  Serial.print(" speed \n");

  delay(4500);
}
