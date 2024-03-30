
#include <Arduino.h>
#include <sioc_dma.h>

template<typename T>
void print_reg(T reg, const char *name = nullptr) {
  if (name) {
    Serial.print(name);
    Serial.print(": ");
  }
  Serial.print("{");
  for (int i = sizeof(T) - 1; i >= 0; i--) {
    for (int j = 7; j >= 0; j--) {
      bool bit = (reg & (1 << (j + (i * 8))));
      Serial.print(bit);
    }
    if (i == 0) {
      Serial.println("}");
    } else {
      Serial.println();
    }
  }
}

template<typename T, size_t N>
void print_array(T (&arr)[N]) {
  for (int i = 0; i < N; i++) {
    Serial.print(arr[i]);
    Serial.print(", ");
  }
}

void setup() {

  using namespace sioc::dma;

  uint32_t buff_in1[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  uint32_t buff_in2[10] = {11, 22, 33, 44, 55, 66, 77, 88, 99, 111};
  uint32_t buff_in3[10] = {2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
  uint32_t buff_out[15] = {0};

  TransferDescriptor td1{}, td2{}, td3{};

  Serial.begin(1);
  while(!Serial);
  Serial.println("-- CONNECTED --");

  Channel channel{};

  DmacChannel *ch = &DMAC->Channel[channel.index];
  channel.set_config({.mode = transfer_mode_e::single, .burst_len = 1, .pri_lvl = 2});

  td1.set_config({.src = &buff_in1, .dst = &buff_out, .src_inc = 1, 
    .dst_inc = 1, .beat_size = 4, .susp_ch = false});
  td2.set_config({.src = &buff_in2, .dst = &buff_out[4], .src_inc = 1,
    .dst_inc = 1, .beat_size = 4, .susp_ch = true});
  td3.set_config({.src = &buff_in3, .dst = &buff_out[9], .src_inc = 1,
    .dst_inc = 1, .beat_size = 4, .susp_ch = true});

  td1.set_length(4, false);
  td2.set_length(5, false);
  td3.set_length(6, false);

  TransferDescriptor *desclist[] = {&td1, &td2};
  TransferDescriptor *desclist_inv[] = {&td3, &td2, &td1};

  auto print_info = [&]() {
    Serial.println();
    Serial.print("Enabled: ");
    // Serial.println(ch->CHCTRLA.bit.ENABLE);
    print_reg(ch->CHSTATUS.reg, "Channel Status");
    print_reg(ch->CHINTFLAG.reg, "Interrupt Flags");
    Serial.println("here");
  };

  auto print_out = [&]() {
    while(channel.transfer_busy());
    Serial.println();
    Serial.print("Output: ");
    print_array(buff_out);
    memset(&buff_out, 0U, sizeof(buff_out)); 
    Serial.println();
  };

  //////////////////////////////////////////////////

  using enum channel_state_e;
  channel.set_transfer(desclist);

  Serial.println("transfer set");

  channel.set_state(active);

  

  DMAC->SWTRIGCTRL.bit.SWTRIG0 = 1U;
  while(!buff_out[0]);
  ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
  print_info();
  print_out();

  ch->CHCTRLA.bit.ENABLE = 0;
  for (int i = 0; i < 1000; i++);
  channel.set_active_transfer_length(1, false);
  ch->CHCTRLA.bit.ENABLE = 1;

  channel.set_state(active);
  DMAC->SWTRIGCTRL.bit.SWTRIG0 = 1U;
  for (int i = 0; i < 1000; i++);
  print_info();
  print_out();


  channel.set_state(active);
  channel.trigger();
  for (int i = 0; i < 1000; i++);

  print_info();
  print_out();



}

void loop() {
  // put your main code here, to run repeatedly:
  

}



  // Serial.println((uintptr_t)&buff_in1);
  // Serial.println(td1._descPtr_->SRCADDR.reg);
  // Serial.println();
  // Serial.println((uintptr_t)&buff_in2);
  // Serial.println(td2._descPtr_->SRCADDR.reg);
  // Serial.println();
  // Serial.println((uintptr_t)&buff_in3);
  // Serial.println(td3._descPtr_->SRCADDR.reg);

  // print_reg(DMAC->Channel[chi].CHCTRLA.reg);
  // print_reg(DMAC->Channel[chi].CHPRILVL.reg);
  // print_reg(DMAC->Channel[chi].CHINTENSET.reg);
  // print_reg(td1._descPtr_->BTCTRL.reg);
  // Serial.println(td1._descPtr_->BTCNT.reg);
  // Serial.println(td1._descPtr_->DESCADDR.reg);
  // Serial.println(td1._descPtr_->SRCADDR.reg);
  // Serial.println(td1._descPtr_->DSTADDR.reg);