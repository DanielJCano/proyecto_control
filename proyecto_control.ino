// Sensor de temperatura
#define sensorPin A7
// Control de ventilador
const int fanPin = 3;             // PWM pin for fan control
// Tachometro para medir la velocidad
const int tachPin = 4;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int reading_sensor = analogRead(sensorPin);
  // Receive fan speed command from Python
  if (Serial.available() > 0) {
    int fanSpeed = Serial.parseInt();
    analogWrite(fanPin, fanSpeed);
  }
  float voltage = (reading_sensor * 5.0) / 1023.0;
  float temperatureC = -((voltage - 0.5)* 100) ;
  Serial.print(voltage); Serial.print(" volts ");
  Serial.print(temperatureC); Serial.print(" degrees C ");
  Serial.print(tachPin); Serial.print(" rpm");
  
  delay(5000);
}
