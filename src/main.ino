const int LEFT_FWD  = 32;
const int LEFT_REV  = 33;
const int RIGHT_FWD = 25;
const int RIGHT_REV = 26;

void stopAll() {
  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(LEFT_REV, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_REV, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_REV, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_REV, OUTPUT);

  stopAll();

  Serial.println("Motor test starting...");
}

void loop() {
  // Test LEFT wheel forward
  Serial.println("LEFT wheel forward");
  stopAll();
  digitalWrite(LEFT_FWD, HIGH);
  delay(3000);

  // Test LEFT wheel reverse
  Serial.println("LEFT wheel reverse");
  stopAll();
  digitalWrite(LEFT_REV, HIGH);
  delay(3000);

  // Test RIGHT wheel forward
  Serial.println("RIGHT wheel forward");
  stopAll();
  digitalWrite(RIGHT_FWD, HIGH);
  delay(3000);

  // Test RIGHT wheel reverse
  Serial.println("RIGHT wheel reverse");
  stopAll();
  digitalWrite(RIGHT_REV, HIGH);
  delay(3000);
  // Stop
  Serial.println("STOP");
  stopAll();
  delay(5000);
}
