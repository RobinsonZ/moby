#include "base64.hpp"

byte buf[15]; 
byte msg[21];

void setup() {
  // initialize all pins we plan on using 
  // (2 through 12, 0+1 are reserved for serial comms and 13 has weird electrical characteristics because of the LED)
  for (int i = 2; i <= 12; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  Serial.begin(115200); 
}

void loop() {
  byte byte0 = B00000000; // 5 padding bits plus DIOs 2-4
  for (int i = 2; i >=0; i--) { // because bitSet and bitClear start at 0 for the rightmost bit
    if (digitalRead(4-i) == HIGH) {
      bitSet(byte0, i);
    } else {
      bitClear(byte0, i);
    }
  }
  buf[0] = byte0;

  byte byte1 = B00000000; // DIOs 5-12
  for (int i = 7; i >=0; i--) {
    if (digitalRead(12-i) == HIGH) {
      bitSet(byte1, i);
    } else {
      bitClear(byte1, i);
    }
  }
  
  buf[1] = byte1;

  // AIOs
  for (int i = 0; i < 6; i++) {
    int analog = analogRead(i);

    buf[(2 * i) + 2] = highByte(analog);
    buf[(2 * i) + 3] = lowByte(analog);
  }

  // checksum
  byte cksum = 0;
  for (int i = 0; i < 14; i++) {
    cksum += buf[i];
  }
  buf[14] = cksum;

  // base64 encode it all
  encode_base64(buf, 15, msg);
  msg[20] = 10; // ASCII newline char

  Serial.flush();
  Serial.write(msg, 21);
  delay(10);
}
