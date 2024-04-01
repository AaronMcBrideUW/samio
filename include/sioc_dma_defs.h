// This file is part of the SAM-IO C++ library.
// Copyright (c) 2024 Aaron McBride.
 
// This program is free software: you can redistribute it and/or modify  
// it under the terms of the GNU General Public License as published by  
// the Free Software Foundation, version 3.

// This program is distributed in the hope that it will be useful, but 
// WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
// General Public License for more details.

// You should have received a copy of the GNU General Public License 
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#pragma once
#include <sam.h>
#include <inttypes.h>
#include <array>

namespace sioc::dma {
  
  enum class channel_state_t {
    disabled,
    suspended,
    enabled,
  };

  /// @brief Denotes the reason that an interrupt was triggered.
  enum class callback_flag_e {
    null,
    tcmpl,
    terr,
    ferr,
  };

  /// @brief Denotes the type of transfer executed by a DMA channel
  ///   each time it recieves a trigger (i.e. is set active).
  enum class blockact_t : int32_t {
    null    = -1,
    noact   = 0,
    noint   = 1,
    suspend = 2,
    both    = 3,
  };

  enum class trigact_t : int32_t {
    null  = -1,
    block = 0,
    burst = 2,
    transaction = 3,
  };

  /// @brief Denotes peripheral trigger sources that can be
  ///   linked to a DMA channel.
  enum class trigsrc_t : int32_t {
    null               = -1,
    none               = 0,
    rtc_timestamp      = 1,
    dsu_dcc0           = 2,
    dsu_dcc1           = 3,
    sercom0_rx         = 4,
    sercom0_tx         = 5,
    sercom1_rx         = 6,
    sercom1_tx         = 7,
    sercom2_rx         = 8,
    sercom2_tx         = 9,
    sercom3_rx         = 10,
    sercom3_tx         = 11,
    sercom4_rx         = 12,
    sercom4_tx         = 13,
    sercom5_rx         = 14,
    sercom5_tx         = 15,
    sercom6_rx         = 16,
    sercom6_tx         = 17,
    sercom7_rx         = 18,
    sercom7_tx         = 19,
    can0_debug_req     = 20,
    can1_debug_req     = 21,
    tcc0_OOB           = 22,
    tcc0_compare_0     = 23,
    tcc0_compare_1     = 24,
    tcc0_compare_2     = 25,
    tcc0_compare_3     = 26,
    tcc0_compare_4     = 27,
    tcc0_compare_5     = 28,
    tcc1_oob           = 29,
    tcc1_compare_0     = 30,
    tcc1_compare_1     = 31,
    tcc1_compare_2     = 32,
    tcc1_compare_3     = 33,
    tcc2_oob           = 34,
    tcc2_compare_0     = 35,
    tcc2_compare_1     = 36,
    tcc2_compare_2     = 37,
    tcc3_oob           = 38,
    tcc3_compare_0     = 39,
    tcc3_compare_1     = 40,
    tcc4_OOB           = 41,
    tcc4_compare_0     = 42,
    tcc4_compare_1     = 43,
    tc0_OOB            = 44,
    tc0_compare_0      = 45,
    tc0_compare_1      = 46,
    tc1_OOB            = 47,
    tc1_compare_0      = 48,
    tc1_compare_1      = 49,
    tc2_OOB            = 50,
    tc2_compare_0      = 51,
    tc2_compare_1      = 52,
    tc3_OOB            = 53,
    tc3_compare_0      = 54,
    tc3_compare_1      = 55,
    tc4_OOB            = 56,
    tc4_compare_0      = 57,
    tc4_compare_1      = 58,
    tc5_OOB            = 59,
    tc5_compare_0      = 60,
    tc5_compare_1      = 61,
    tc6_OOB            = 62,
    tc6_compare_0      = 63,
    tc6_compare_1      = 64,
    tc7_OOB            = 65,
    tc7_compare_0      = 66,
    tc7_compare_1      = 67,
    adc0_resrdy        = 68,
    adc0_seq           = 69,
    adc1_resrdy        = 70,
    adc1_seq           = 71,
    dac_empty0         = 72,
    dac_empty1         = 73,
    dac_result_ready_0 = 74,
    dac_result_ready_1 = 75,
    i2s_rx0            = 76,
    i2s_rx1            = 77,
    i2s_tx0            = 78,
    i2s_tx1            = 79,
    pcc_rx             = 80,
    aes_write          = 81,
    aes_read           = 82,
    qspi_rx            = 83,
    qspi_tx            = 84,
  };


  using callback_t = void (*)(const uint32_t&, const callback_flag_e&);
  using length_t = decltype(std::declval<DmacDescriptor>().BTCNT.reg);


} /// End of sioc::dma namespace
