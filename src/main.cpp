
#include <Arduino.h>
#include <sio_core_dma2.h>

  using namespace sio::core;

void setup() {

  Serial.begin(1);
  while(!Serial);
  Serial.println("-- CONNECTED --");


}

void loop() {
  // put your main code here, to run repeatedly:
}
