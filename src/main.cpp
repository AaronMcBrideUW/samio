
#include <Arduino.h>
#include <sioc_dma.h>


template<typename T, size_t N>
void print_array(T (&arr)[N]) {
  for (int i = 0; i < N; i++) {
    Serial.print(arr[i]);
    Serial.print(", ");
  }
  Serial.println();
}


void setup() {

  using namespace sioc::dma;

  uint32_t buff_in1[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  uint32_t buff_in2[10] = {0, 11, 22, 33, 44, 55, 66, 77, 88, 99};
  uint32_t buff_in3[10] = {0, 2, 4, 8, 16, 32, 64, 128, 256, 512};
  uint32_t buff_out[10] = {0};


  Serial.begin(1);
  while(!Serial);
  Serial.println("-- CONNECTED --");

  int chi = allocate_channel();

  DmacChannel *ch = &DMAC->Channel[chi];
  Serial.print("initialized: ");
  Serial.println(DMAC->CTRL.bit.DMAENABLE);

  set_channel_config(chi, {.mode = transfer_mode_e::all, .pri_lvl = 2});
  Serial.print("channel config set: ");
  Serial.println(ch->CHPRILVL.bit.PRILVL);

  Serial.println();

  TransferDescriptor td1{};
  TransferDescriptor td2{};
  TransferDescriptor td3{};

  td1.set_config({.src = &buff_in1, .dst = &buff_out, .src_inc = 1, 
    .dst_inc = 1, .beat_size = 4});
  td2.set_config({.src = &buff_in2, .dst = &buff_out, .src_inc = 1,
    .dst_inc = 1, .beat_size = 4});
  td3.set_config({.src = &buff_in3, .dst = &buff_out, .src_inc = 1,
    .dst_inc = 1, .beat_size = 4});

  Serial.println("transfer desc config set: ");
  Serial.println("source");
  Serial.print((uintptr_t)&buff_in1);
  Serial.print(" to -> ");
  Serial.println(td1._descPtr_->SRCADDR.bit.SRCADDR);
  Serial.println("destination");
  Serial.print((uintptr_t)&buff_out);
  Serial.print(" to -> ");
  Serial.println(td1._descPtr_->DSTADDR.bit.DSTADDR);
  Serial.println("stepsize:");
  Serial.println(td1._descPtr_->BTCTRL.bit.STEPSIZE);

  Serial.println();
  Serial.println();

  td1.set_length(3, false);
  td2.set_length(4, false);
  td3.set_length(5, false);

  Serial.println("Length set (check addr): ");
  Serial.println("source");
  Serial.print((uintptr_t)&buff_in1);
  Serial.print(" to -> ");
  Serial.println(td1._descPtr_->SRCADDR.bit.SRCADDR);
  Serial.println("destination");
  Serial.print((uintptr_t)&buff_out);
  Serial.print(" to -> ");
  Serial.println(td1._descPtr_->DSTADDR.bit.DSTADDR);

  Serial.println();

  Serial.println("Length values");
  Serial.println(td1.length(true));
  Serial.println(td2.length(true));
  Serial.println(td3.length(true));

  Serial.println();
  Serial.println();

  Serial.println("Setting transfer list:");

  TransferDescriptor *desclist[] = {&td1, &td2, &td3};
  set_transfer(chi, desclist);

  Serial.println("Transfer list set!!!");

  Serial.println(td1._next_ == &td2);
  Serial.println(td2._next_ == &td3);
  Serial.println(td3._next_ == nullptr);
  Serial.println(td1._descPtr_ != &td1._desc_);
  Serial.println(td1._assig_ + td2._assig_);

  Serial.println();
  Serial.println();

  Serial.println("Setting to idle:");
  set_channel_state(chi, channel_state_e::idle);
  Serial.println(channel_state(chi) == channel_state_e::idle);

  Serial.println("before:");
  print_array(buff_out);

  Serial.println("Setting to active:");
  set_channel_state(chi, channel_state_e::active);
  Serial.println(channel_state(chi) == channel_state_e::active);

  while(channel_state(chi) == channel_state_e::active);
  Serial.println("after:");
  print_array(buff_out);

  Serial.println();
  Serial.println();

  Serial.println("Testing unlinking:");
  td2.unlink();
  
  memset(buff_out, 0, sizeof(buff_out));
  Serial.println("before");
  print_array(buff_out);
  while(channel_state(chi) == channel_state_e::active);
  Serial.println("after");
  print_array(buff_out);
 

  
}

void loop() {
  // put your main code here, to run repeatedly:
  

}
