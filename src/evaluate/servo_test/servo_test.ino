#include <Servo.h>

Servo testServo;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  testServo.attach(3);
  testServo.writeMicroseconds(1500);
  //testServo.write(90);

/*
  for(int i=0;i<90;i++) {
    delay(50);
    testServo.write(90+i);
  }
  for(int i=180;i>0;i--) {
    delay(50);
    testServo.write(i);
  }
  for(int i=0;i<=90;i++) {
    delay(50);
    testServo.write(i);
  }
  */
 Serial.println("End of arming");

  delay(5000);
}

void loop() {
    testServo.writeMicroseconds(1600);
    //testServo.write(105);
    delay(3000);
    
    testServo.writeMicroseconds(1500);
    //testServo.write(90);
    delay(2000);
    testServo.writeMicroseconds(1300);
    //testServo.writeMicroseconds(1400);
    //testServo.write(95);
    delay(2000);
    testServo.writeMicroseconds(1500);
    delay(2000);
    testServo.writeMicroseconds(1300);
    delay(3000);
    testServo.writeMicroseconds(1500);
    delay(2000);
    
}
