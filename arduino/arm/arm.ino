#include <Servo.h>

Servo servo1, servo2, servo3, servo4;
int LED = 12;

int prev_angles[4] = {90, 90, 90, 90}; // Store previous angles

void setup() {
  Serial.begin(115200);
  servo1.attach(8);
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);

  servo1.write(prev_angles[0]);
  servo2.write(prev_angles[1]);
  servo3.write(prev_angles[2]);
  servo4.write(prev_angles[3]);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') {
      input = "";
    } else if (c == '>') {
      processInput(input);
    } else {
      input += c;
    }
  }
}

void processInput(String data) {
  float radians[4] = {0.0, 0.0, 0.0, 0.0};
  int angles[4] = {90, 90, 90, 90};
  int index = 0;

  char *token = strtok(const_cast<char*>(data.c_str()), ",");
  while (token != NULL && index < 4) {
    radians[index] = atof(token);  // convert to float
    angles[index] = int(radians[index] * (180.0 / PI)) + 90;  // rad → deg + offset
    token = strtok(NULL, ",");
    index++;
  }

  bool moved = false;

  if (angles[0] != prev_angles[0]) {
    servo1.write(angles[0]);
    prev_angles[0] = angles[0];
    moved = true;
  }
  if (angles[1] != prev_angles[1]) {
    servo2.write(angles[1]);
    prev_angles[1] = angles[1];
    moved = true;
  }
  if (angles[2] != prev_angles[2]) {
    servo3.write(angles[2]);
    prev_angles[2] = angles[2];
    moved = true;
  }
  if (angles[3] != prev_angles[3]) {
    servo4.write(angles[3]);
    prev_angles[3] = angles[3];
    moved = true;
  }

  if (moved) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
  }
}
