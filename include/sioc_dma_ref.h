
#pragma once
#include <sam.h>
#include <array>
#include <inttypes.h>

namespace sioc::dma::ref {

  //// MISC INFO ////
  inline constexpr uint32_t max_ch = 16;
  inline constexpr uint32_t inst_num = DMAC_INST_NUM;

  //// DMA SYSTEM ////
  inline constexpr std::array<uint32_t, 5>                       irq_array = {31, 32, 33, 34, 35};
  inline constexpr std::array<uint32_t, (1 << __NVIC_PRIO_BITS)> irqpri_map = {0, 1, 2, 3, 4, 5, 6, 7};
  inline constexpr std::array<uint32_t, DMAC_LVL_NUM>            squal_map  = {0, 1, 2, 3}; 

  //// DMA CHANNEL ////
  inline constexpr std::array<uint32_t, 16> burstlen_map = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  inline constexpr std::array<uint32_t, 4> threshold_map = {1, 2, 4, 8};

  //// TRANSFER DESCRIPTOR ////
  inline constexpr std::array<uint32_t, 3> beatsize_map  = {1, 2, 4};
  inline constexpr std::array<uint32_t, 8> stepsize_map  = {1, 2, 4, 8, 16, 32, 64, 128};
  
}