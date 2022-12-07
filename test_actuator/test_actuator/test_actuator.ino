// Motor connections
int speed = 9;
int in1 = 16;
int in2 = 17;

void setup() {
  pinMode(speed, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  Serial.begin(9600);
  // while (!Serial);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(speed, 100);
  Serial.println("Motor on");
}

void loop() {
  // retract
  Serial.println("Retract");
  digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
  delay(150);

  // extend
  Serial.println("Extend");
  digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
  delay(300);

  digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
  delay(1000);
}
