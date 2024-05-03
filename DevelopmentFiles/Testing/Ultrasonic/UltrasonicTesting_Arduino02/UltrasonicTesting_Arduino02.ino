// defines pins numbers
const int trigPinA = 8;
const int trigPinB = 9;

const int echoPinA = 2;
const int echoPinB = 3;

const int ledPin = 13;

const char A = 'A';
const char B = 'B';

volatile unsigned long a_start = 0;
volatile unsigned long a_end = 0;
volatile unsigned long b_start = 0;
volatile unsigned long b_end = 0;

volatile bool a_available = false;
volatile bool b_available = false;

double distA;
double distB;

#define pulse2dist(x) x*0.017


void trigger(char sensor)
{
  if (sensor=='A')
  {
      digitalWrite(trigPinA, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinA, LOW);
      Serial.println("Trigger A");
  }
  else if (sensor=='B')
  {
      digitalWrite(trigPinB, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinB, LOW);
      Serial.println("Trigger B");
  }
}

void dist2Serial(char sensor,double dist)
{
  if (sensor=='A')
  {
      Serial.print("Sensor A Distance:");
      Serial.println(dist);
  }
  else if (sensor=='B')
  {
      Serial.print("Sensor B Distance:");
      Serial.println(dist);
  }
}

void ISR_ultra()
{
    if (digitalRead(echoPinA) == HIGH) {
    // start measuring
    a_start = micros();
  }
  else {
    // stop measuring
    a_end = micros();
    a_available = true;
  }

  if (digitalRead(echoPinB) == HIGH) {
    // start measuring
    b_start = micros();
  }
  else {
    // stop measuring
    b_end = micros();
    b_available = true;
  }
}


void setup() {
  pinMode(trigPinA, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinA, INPUT); // Sets the echoPin as an Input

  pinMode(trigPinB, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinB, INPUT); // Sets the echoPin as an Input

  digitalWrite(trigPinA, LOW);
  digitalWrite(trigPinB, LOW);

  delay(100);
  attachInterrupt(digitalPinToInterrupt(echoPinA),ISR_ultra,CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinB),ISR_ultra,CHANGE);

  
  Serial.begin(9600);
  delay(300);
  Serial.println("");

  trigger(A);

  // distA = pulse2dist(pulseIn(echoPinA, HIGH));
  delay(2);
  trigger(B);
  // distB = pulse2dist(pulseIn(echoPinB, HIGH));
  
  
}
void loop() {

  if(a_available)
  {
    distA = pulse2dist(double(a_end-a_start));
    dist2Serial(A,distA);
    a_available = false;
  }
  if (b_available)
  {
    distB = pulse2dist(double(b_end-b_start));
    dist2Serial(B,distB);
    b_available = false;
  }
}