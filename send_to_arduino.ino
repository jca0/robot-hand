#include <Servo.h>

const int NUM_SERVOS = 6;
Servo servos[NUM_SERVOS];
// thumb rotation, index, middle, ring, pinky, thumb
int servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11}; // adjust to wiring

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(0); // Start neutral
  }
}

void loop() {
  static String inputString = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleInput(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

void handleInput(String data) {
  int angles[NUM_SERVOS];
  int idx = 0;
  int lastIndex = 0;

  while (idx < NUM_SERVOS) {
    int commaIndex = data.indexOf(',', lastIndex);
    String part;
    if (commaIndex == -1) {
      part = data.substring(lastIndex);
    } else {
      part = data.substring(lastIndex, commaIndex);
    }
    angles[idx] = constrain(part.toInt(), 0, 180);
    lastIndex = commaIndex + 1;
    idx++;
    if (commaIndex == -1) break;
  }

  if (idx == NUM_SERVOS) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i].write(angles[i]);
    }
  } else {
    Serial.println("Invalid input received.");
  }
}
