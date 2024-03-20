
#include <Arduino.h>
#include <sio_core_dma3.h>

using namespace sio::core;



void setup() {

  Serial.begin(1);
  while(!Serial);
  Serial.println("-- CONNECTED --");

    DmaPeripheral dma = DmaPeripheral(0);

    dma.sys.initialized(true);
    dma.sys.enabled(true);

    Serial.println("SUCCESS");
    Serial.println(dma.sys.initialized());
    Serial.println(dma.sys.enabled());

    Serial.println("----");

}

void loop() {
  // put your main code here, to run repeatedly:
  

}
