
#include <Arduino.h>
#include <sam.h>
#include <inttypes.h>
#include <algorithm>
#include <iterator>

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

volatile int intpend{0};

void DMAC_0_Handler(void) {
  intpend = DMAC->INTPEND.reg;
  DMAC->Channel[0].CHINTENCLR.bit.SUSP = 1;
};

void setup() {

  Serial.begin(1);
  while(!Serial);
  Serial.println("CONNECTED");

  static DmacDescriptor bdesc[32]{};
  static volatile DmacDescriptor wbdesc[32]{};

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

  DMAC->CTRL.vec.LVLEN = 1;
  MCLK->AHBMASK.bit.DMAC_ = 1;
  DMAC->BASEADDR.reg = (uintptr_t)&bdesc;
  DMAC->WRBADDR.reg = (uintptr_t)&wbdesc;
  NVIC_EnableIRQ(DMAC_0_IRQn);
  NVIC_EnableIRQ(DMAC_1_IRQn);
  NVIC_EnableIRQ(DMAC_2_IRQn);
  NVIC_EnableIRQ(DMAC_3_IRQn);
  NVIC_EnableIRQ(DMAC_4_IRQn);
  DMAC->CTRL.bit.DMAENABLE = 1;

  DMAC->Channel[0].CHCTRLA.bit.TRIGACT = DMAC_CHCTRLA_TRIGACT_BLOCK_Val;
  bdesc[0].SRCADDR.reg = (uintptr_t)&buff_in1 + (4 * 3);
  bdesc[0].DSTADDR.reg = (uintptr_t)&buff_out + (4 * 3);
  bdesc[0].BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val;
  bdesc[0].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_WORD_Val;
  bdesc[0].BTCTRL.bit.VALID = 1;
  bdesc[0].BTCTRL.bit.SRCINC = 1;
  bdesc[0].BTCTRL.bit.DSTINC = 1;
  bdesc[0].BTCNT.reg = 3;

  DmacDescriptor desc1;
  desc1.SRCADDR.reg = (uintptr_t)&buff_in2 + (4 * 3);
  desc1.DSTADDR.reg = (uintptr_t)&buff_out[3] + (4 * 3);
  desc1.BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
  desc1.BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_WORD_Val;
  desc1.BTCTRL.bit.VALID = 1;
  desc1.BTCTRL.bit.SRCINC = 1;
  desc1.BTCTRL.bit.DSTINC = 1;
  desc1.BTCNT.reg = 3;

  DmacDescriptor desc2;
  desc2.SRCADDR.reg = (uintptr_t)&buff_in3 + (4 * 4);
  desc2.DSTADDR.reg = (uintptr_t)&buff_out[6] + (4 * 4);
  desc2.BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
  desc2.BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_WORD_Val;
  desc2.BTCTRL.bit.VALID = 1;
  desc2.BTCTRL.bit.SRCINC = 1;
  desc2.BTCTRL.bit.DSTINC = 1;
  desc2.BTCNT.reg = 4;

  // bdesc[0].DESCADDR.reg = (uintptr_t)&desc1;

  DMAC->Channel[0].CHINTENSET.bit.SUSP = 1;
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;

  DMAC->SWTRIGCTRL.bit.SWTRIG0 = 1;
  while(DMAC->Channel[0].CHSTATUS.bit.BUSY);
  print_reg(wbdesc[0].BTCTRL.reg, "btctrl");
  print_reg(wbdesc[0].BTCNT.reg, "btcnt");
  print_reg(wbdesc[0].DESCADDR.reg, "descaddr");
  print_out();
  print_reg(DMAC->Channel[0].CHSTATUS.reg, "status");
  print_reg(DMAC->Channel[0].CHINTFLAG.reg, "intflag");
  


}

void loop() {

}

