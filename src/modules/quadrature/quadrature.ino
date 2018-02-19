#include <LS7366R.h>
#include <SPI.h>

#define ENCODER_SS 10

using namespace LS7366R;

byte MDR0_CONFIG = MDR0_X1QUAD|MDR0_FREE_CNT|MDR0_IDX_DISABLE|MDR0_FILTR_CLK_DIV1;
byte MDR1_CONFIG = MDR1_CNT_4BYTE|MDR1_FLAG_NONE;

QuadratureEncoder encoder(ENCODER_SS, MDR0_CONFIG, MDR1_CONFIG);

void setup(){
  Serial.begin(115200);
  pinMode(ENCODER_SS, OUTPUT);
  SPI.begin();
  encoder.init();
}

void loop() {
  delay(1000);
  long count = encoder.count();
  byte status = encoder.status();
  Serial.print("count: ");
  Serial.print(count);
  Serial.print(", status: ");
  Serial.println(status);
}

