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
#include <sioc_dma_cfg.h>
#include <inttypes.h>
#include <sam.h>
#include <String.h>
#include <math.h>
#include <utility>
#include <bit>
#include <type_traits>
#include <array>
#include <iterator>
#include <algorithm>

namespace sioc::dma {

  //// FORWARD DECLS ////
  struct SysConfig;
  struct Channelconfig;
  struct TransferDescriptor;

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: SYSTEM FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////
    
  int32_t allocate_channel();

  bool free_channel(const uint32_t &ch_index);

  uint32_t free_channel_count();

  int32_t active_channel_index();

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHANNEL FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  typedef struct ChannelConfig {
    linked_peripheral_e periph = linked_peripheral_e::null;
    transfer_mode_e mode       = transfer_mode_e::null;
    int32_t burst_len          = -1;
    int32_t pri_lvl            = -1;
    callback_t callback        = detail::dummy_callback;
  };

  bool set_channel_state(const uint32_t &ch_index, const channel_state_e &state);

  channel_state_e channel_state(const uint32_t &ch_index);

  bool set_channel_config(const uint32_t &ch_index, const ChannelConfig &cfg);

  ChannelConfig channel_config(const uint32_t &ch_index);

  void reset_channel(const uint32_t &ch_index);

  bool skip_suspend(const uint32_t &ch_index);

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: BLOCK DESCRIPTOR OBJ 
///////////////////////////////////////////////////////////////////////////////////////////////////

  typedef struct TransferDescriptor {
    
    TransferDescriptor() = default;

    TransferDescriptor(const TransferDescriptor &other);
    TransferDescriptor(TransferDescriptor &&other);

    TransferDescriptor &operator = (const TransferDescriptor &other);       
    TransferDescriptor &operator = (TransferDescriptor &&other);

    typedef struct Config {
      void *src        = &detail::dummy_loc;
      void *dst        = &detail::dummy_loc;
      int src_inc      = -1;
      int dst_inc      = -1;
      int beat_size    = -1;
      int susp_ch      = -1;
    };

    bool set_config(const Config &cfg);

    Config config() const;
    
    bool set_length(const uint32_t &len, const bool &in_bytes);
    
    uint32_t length(const bool &in_bytes) const;

    bool valid() const;
  
    void unlink();

    ~TransferDescriptor();

    //// INTERNAL ////
    DmacDescriptor _desc_{}; 
    DmacDescriptor *_descPtr_ = &_desc_;
    TransferDescriptor *_next_;
    int _assig_ = -1;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DESCRIPTOR FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  namespace detail {  
    void set_transfer(const uint32_t&, TransferDescriptor** = nullptr, 
      const uint32_t& = 0, const bool& = false);
  }

  template<uint32_t N>
  void set_transfer(const uint32_t ch_index, TransferDescriptor 
    *(&desc_array)[N], const bool &looped = false) {
    detail::set_transfer(ch_index, desc_array, N, looped);
  }

  void set_transfer(const uint32_t ch_index, TransferDescriptor *tdesc,
    const bool &looped = false);

  bool set_active_transfer(const uint32_t &ch_index, const uint32_t &td_index);

  TransferDescriptor *active_transfer(const uint32_t &ch_index);

  bool set_active_transfer_length(const uint32_t &ch_index,
    const uint32_t &len, const bool &in_bytes);

  uint32_t active_transfer_length(const uint32_t &ch_index, 
    const bool &in_bytes);

















    


  





  







}





