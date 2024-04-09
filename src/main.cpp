
#include <Arduino.h>
#include <sioc_dma.h>

template<typename T>
void print_reg(const T &reg, const char *name) {
  Serial.println(name);
  for (int i = sizeof(T) - 1; i >= 0; i--) {
    for (int j = 7; j >= 0; j--) {
      bool bit = (reg & (1 << (j + (i * 8))));
      Serial.print(static_cast<unsigned int>(bit));
    }
    Serial.println();
  }
  Serial.println();
}

void setup() {

  Serial.begin(1);
  while(!Serial);
  Serial.println("CONNECTED");

  using namespace sioc;

  static constexpr uint32_t buff_in1[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  static constexpr uint32_t buff_in2[10] = {11, 22, 33, 44, 55, 66, 77, 88, 99};
  static constexpr uint32_t buff_in3[10] = {111, 222, 333, 444, 555, 666, 777, 888, 999};
  static uint32_t buff_out[20] = {0};

  auto print_out = []() {
    Serial.print("Output: ");
    for (uint32_t i = 0; i < std::size(buff_out); i++) {
      Serial.print(buff_out[i]);
      if (i < std::size(buff_out) - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
    Serial.println();
  };

  dma::Channel chan = dma::Channel();
  DmacChannel *ch = &DMAC->Channel[0];
  static constexpr dma::Channel::ChannelConfig cfg{.trigact = dma::trigact_t::block};
  chan.set_channel_config<cfg>();
  
  dma::TransferDescriptor td1{};
  dma::TransferDescriptor td2{};
  dma::TransferDescriptor td3{};

  constexpr dma::TransferDescriptor::Config td1_cfg{
      .beatsize = 4, 
      .src = buff_in1, .srcinc = 1, 
      .dst = buff_out, .dstinc = 1,
      .blockact = dma::blockact_t::noact
    };
  constexpr dma::TransferDescriptor::Config td2_cfg{
      .beatsize = 4, 
      .src = buff_in2, .srcinc = 1, 
      .dst = &buff_out[3], .dstinc = 1, 
      .blockact = dma::blockact_t::suspend
    };
  constexpr dma::TransferDescriptor::Config td3_cfg{
      .beatsize = 4,
      .src = buff_in3, .srcinc = 1,
      .dst = &buff_out[6], .dstinc = 1,
      .blockact = dma::blockact_t::suspend
    };
  td1.set_config<td1_cfg>();
  td2.set_config<td2_cfg>();
  td3.set_config<td3_cfg>();

  print_reg(td1._descPtr_->BTCTRL.reg, "td1 ctrl");
  print_reg(td2._descPtr_->BTCTRL.reg, "td2 ctrl");
  print_reg(td3._descPtr_->BTCTRL.reg, "td3 ctrl");

  Serial.println((uintptr_t)buff_in1);
  Serial.println(td1._descPtr_->SRCADDR.reg);
  Serial.println((uintptr_t)buff_out);
  Serial.println(td1._descPtr_->DSTADDR.reg);
  Serial.println();

  td1.set_length(12, true);
  td2.set_length(3, false);
  td3.set_length(4, false);

  Serial.println((uintptr_t)buff_in1);
  Serial.println(td1._descPtr_->SRCADDR.reg);
  Serial.println((uintptr_t)buff_out);
  Serial.println(td1._descPtr_->DSTADDR.reg);
  Serial.println();

  chan.set_transfer<3>({&td1, &td2, &td3}, false);
  chan.set_state(dma::channel_state_t::enabled);

  Serial.println(td1._descPtr_->DESCADDR.reg);
  Serial.println((uintptr_t)&td2._descPtr_);
  print_reg(td2._descPtr_->BTCTRL.reg, "td2 info");

  /// TRIGGER 1
  chan.trigger();
  while(chan.transfer_busy());
  print_out();

  Serial.println((int)chan.state());
  print_reg(ch->CHINTFLAG.reg, "intflags");
  chan.set_state(dma::channel_state_t::enabled);

  chan.set_state(dma::channel_state_t::suspended);

  /// TRIGGER 2
  chan.trigger();
  Serial.println(chan.trigger_pending());
  chan.set_state(dma::channel_state_t::enabled);

  while(chan.transfer_busy());
  print_out();

  Serial.println((int)chan.state());
  print_reg(ch->CHINTFLAG.reg, "intflags");
  chan.set_state(dma::channel_state_t::enabled);

  /// TRIGGER 3
  chan.trigger();
  while(chan.transfer_busy());
  print_out();

  memset(buff_out, 0U, sizeof(buff_out));
  Serial.println((int)chan.state());
  print_reg(ch->CHINTFLAG.reg, "intflags");
  chan.set_state(dma::channel_state_t::enabled);

  /// TRIGGER 4
  chan.trigger();
  while(chan.transfer_busy());
  print_out();

  Serial.println((int)chan.state());
  print_reg(ch->CHINTFLAG.reg, "intflags");
  chan.set_state(dma::channel_state_t::enabled);


  //// @b RESET ////
  memset(buff_out, 0U, sizeof(buff_out));
  chan.set_transfer<2>({&td1, &td2}, true);

  /// TRIGGER 5
  chan.trigger();
  while(chan.transfer_busy());
  print_out();

  /// COPY ///
  uint32_t test;
  Serial.println("here");
  chan.set_active_transfer(&td3);

  /// TRIGGER 6
  chan.trigger();
  while(chan.transfer_busy());
  print_out();

  memset(buff_out, 0U, sizeof(buff_out));

  /// TRIGGER 7
  chan.set_state(dma::channel_state_t::enabled);
  chan.trigger();
  while(chan.transfer_busy());
  print_out();


}

void loop() {

}