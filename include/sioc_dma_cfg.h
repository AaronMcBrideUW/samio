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
#include <sioc_dma_defs.h>
#include <sam.h>

namespace sioc::dma {


  //// CONSTANT CONFIG ////
  inline constexpr uint32_t max_channels = 16;
  inline constexpr bool prilvl_enabled[DMAC_LVL_NUM] = {true, true, true, true};
  inline constexpr bool prilvl_round_robin_mode[DMAC_LVL_NUM] = {false, false, false, false};
  inline constexpr uint32_t prilvl_service_quality[DMAC_LVL_NUM] = {1, 1, 2, 3}; 
  inline constexpr uint32_t irq_priority[hwref::_irqcnt_] = {1, 1, 1, 1, 1};
  inline constexpr bool ch_run_in_standby[max_channels] = {false, false, false, false};

  //// CHANNEL CONFIG DEFAULTS ////
  inline constexpr linked_peripheral_e ch_def_periph = linked_peripheral_e::none;
  inline constexpr transfer_mode_e ch_def_mode = transfer_mode_e::single;
  inline constexpr uint32_t ch_def_burst_len = 1;
  inline constexpr uint32_t ch_def_pri_lvl = 1;

  //// TRANSFER DESCRIPTOR CONFIG DEFAULTS ////
  inline constexpr uint32_t td_def_beat_size = 1;
  inline constexpr uint32_t td_def_src_inc = 1;
  inline constexpr uint32_t td_def_dst_inc = 1;
  inline constexpr bool td_def_susp_ch = true;

}

