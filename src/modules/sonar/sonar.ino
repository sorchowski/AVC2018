#include <arduino.h>

  #define MAX_SENSORS 8

  // kind of backwards the sensors on the board are meant to be used left to right.
  #define PIN_TRIGGER_1 9
  #define PIN_TRIGGER_2 8
  #define PIN_TRIGGER_3 7
  #define PIN_TRIGGER_4 6
  #define PIN_TRIGGER_5 5
  #define PIN_TRIGGER_6 4
  #define PIN_TRIGGER_7 3
  #define PIN_TRIGGER_8 2

  #define PIN_ECHO1 10
  #define PIN_ECHO2 11

  #define TIMEOUT 500000   //in microseconds = 1/2 second

  class SonarSensor {

    public:
      SonarSensor() {}
      void setPins(int tgrPin, int ePin);
      long scan();
      long getDistance() { return distance; }

    private:
      int triggerPin;
      int echoPin;
      long distance;
  };


  class SonarArray {
    public:
      SonarArray(int numberSensors);
      void scan();
      long * getDistances();

    private:
      SonarSensor sensors[MAX_SENSORS];
      int numberSensors;
      long distances[MAX_SENSORS];
  };

void SonarSensor::setPins(int tgrPin, int ePin) {
  triggerPin = tgrPin;
  echoPin = ePin;

  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);

  pinMode(echoPin, INPUT);
}

long SonarSensor::scan() {
  long duration;
  // Pulse the trigger pin
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);  // per spec, pulse should be 10us
  digitalWrite(triggerPin, LOW);

  // TODO: add timeout value for max distance
  // timeout value is in microseconds, default is 1 second
  duration = pulseIn(echoPin, HIGH, TIMEOUT);

  // TODO: what do these values mean?
  distance = (duration/2) / 29.1;
  return distance;
}

SonarArray::SonarArray(int numSensors) {
  numberSensors = numSensors;
  int triggerAssignments[MAX_SENSORS] = {PIN_TRIGGER_1, PIN_TRIGGER_2, PIN_TRIGGER_3, PIN_TRIGGER_4, PIN_TRIGGER_5, PIN_TRIGGER_6, PIN_TRIGGER_7, PIN_TRIGGER_8};
  int echoAssignments[MAX_SENSORS] = {PIN_ECHO1, PIN_ECHO1, PIN_ECHO1, PIN_ECHO1, PIN_ECHO2, PIN_ECHO2, PIN_ECHO2, PIN_ECHO2};
  for (int i=0;i<MAX_SENSORS;i++) {
    sensors[i].setPins(triggerAssignments[i], echoAssignments[i]);
  }
}

void SonarArray::scan() {
  for (int i=0;i<numberSensors;i++) {
    distances[i] = sensors[i].scan();
  }
}

long * SonarArray::getDistances() {
  return distances;
}

//SonarSensor sonarSensor;

SonarArray sonarArray(5);

void setup() {
  Serial.begin(115200);
  //int triggerPin = 9;
  //pinMode(triggerPin, OUTPUT);
  //digitalWrite(triggerPin, LOW);

  //sonarSensor.setPins(PIN_TRIGGER_1, PIN_ECHO);
}

void loop() {
  //int triggerPin = 9;
  //long distance = *distances;
  //String distStr = String(distance);
  //String output = "Distance: "+distStr;
  //Serial.println(output);
  //delay(2000);
  //digitalWrite(triggerPin, HIGH);
  //delay(2000);
  //digitalWrite(triggerPin, LOW);

  sonarArray.scan();
  long * distances = sonarArray.getDistances();
  long d1_distance = distances[0];
  long d2_distance = distances[1];
  long d5_distance = distances[4];

  String d1ValueStr = String(d1_distance);
  String d2ValueStr = String(d2_distance);
  String d5ValueStr = String(d5_distance);
  String distanceStr = "D1: "+d1ValueStr+", D2: "+d2ValueStr+", D5: "+d5ValueStr;

  Serial.println(distanceStr);
  delay(1000);
}
