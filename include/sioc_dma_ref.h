
#pragma once

#include <sam.h>
#include <array>
#include <inttypes.h>

namespace sioc::dma::ref
{

  //// MISC INFO ////
  static inline constexpr uint32_t ch_num = 16;
  static inline constexpr uint32_t inst_num = DMAC_INST_NUM;
  static inline constexpr uint32_t max_td_inst = 128;

  //// DMA SYSTEM ////
  static inline constexpr uint32_t def_irq_prio = 0; // DEFAULT
  static inline constexpr std::array<uint32_t, 5>                       irq_array = {31, 32, 33, 34, 35};
  static inline constexpr std::array<uint32_t, (1 << __NVIC_PRIO_BITS)> irqpri_map = {1, 2, 3, 4, 5, 6, 7, 8};
  static inline constexpr std::array<uint32_t, DMAC_LVL_NUM>            prilvl_map = {1, 2, 3, 4};
  static inline constexpr std::array<uint32_t, DMAC_LVL_NUM>            squal_map  = {1, 2, 3, 4};

  //// DMA CHANNEL ////
  static inline constexpr std::array<uint32_t, 16> burstlen_map = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  static inline constexpr std::array<uint32_t, 4> threshold_map = {1, 2, 4, 8};

  //// TRANSFER DESCRIPTOR ////
  static inline constexpr std::array<uint32_t, 3> beatsize_map  = {1, 2, 4};
  static inline constexpr std::array<uint32_t, 8> stepsize_map  = {1, 2, 4, 8, 16, 32, 64, 128};
  
}

