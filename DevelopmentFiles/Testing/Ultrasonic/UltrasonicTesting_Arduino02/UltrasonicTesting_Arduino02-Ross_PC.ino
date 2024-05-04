/*
  Ultrasonic Sensor HC-SR04 and Arduino Tutorial

  by Dejan Nedelkovski,
  www.HowToMechatronics.com

*/
// defines pins numbers
const int trigPinA = 9;
const int trigPinA = 9;

const int echoPinA = 10;
const int echoPinB = 10;

const int ledPin = 13;

// defines variables
long durationA;
int distanceA;S

long durationB;
int distanceB;

void setup() {
  pinMode(trigPinA, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinA, INPUT); // Sets the echoPin as an Input

  pinMode(trigPinB, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinB, INPUT); // Sets the echoPin as an Input
  
  


}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  // Serial.print("Distance: ");
  // Serial.println(distance);

  if (distance < 80)
  {
    digitalWrite(ledPin, HIGH);
  }
  else{
    digitalWrite(ledPin, LOW);
  }
  
  delay(10);
}