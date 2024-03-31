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
 
/*
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
#include <Arduino.h>
#include <sio_temp.h>

namespace sioc::dma {

  //// FORWARD DECLS ////
  struct SysConfig;
  struct Channelconfig;
  struct TransferDescriptor;

  //// @e TO_REMOVE!!!!
  extern TransferDescriptor *_btd_[DMAC_CH_NUM];
  extern volatile SECTION_DMAC_DESCRIPTOR __ALIGNED(16) DmacDescriptor _wbdesc_[DMAC_CH_NUM];
  extern SECTION_DMAC_DESCRIPTOR __ALIGNED(16) DmacDescriptor _bdesc_[DMAC_CH_NUM];

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: SYSTEM FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////
    
  uint32_t free_channel_count();

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHANNEL FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  struct Channel {

    typedef struct ChannelConfig {
      linked_peripheral_e periph = linked_peripheral_e::null;
      transfer_mode_e mode       = transfer_mode_e::null;
      int32_t burst_len          = -1;
      int32_t pri_lvl            = -1;
      callback_t callback        = detail::dummy_callback;
    };  

    const uint32_t &index = _index_;

    Channel();
    Channel(const int32_t &index);

    Channel(const Channel &other);
    Channel(Channel &&other);

    Channel &operator = (const Channel &other);
    Channel &operator = (Channel &&other); 

    bool set_config(const ChannelConfig &cfg);
    bool set_config(const Channel &other);

    ChannelConfig config();

    bool reset(const bool &clear_transfer);

    bool set_state(const channel_state_e &state);
    channel_state_e state() const;

    void trigger();
    bool trigger_pending() const;
    bool transfer_busy() const;

    template<uint32_t N>
    bool set_transfer(TransferDescriptor *(&desc_array)[N], 
      const bool &looped = false) {
      return set_transfer_impl(desc_array, N, looped);
    }

    bool set_transfer(TransferDescriptor *tdesc, const bool &looped = false);

    bool set_transfer(std::initializer_list<TransferDescriptor*> desclist,  
      const bool &looped);

    bool set_active_transfer(const uint32_t &td_index);
    
    bool set_active_transfer(TransferDescriptor *td, const int32_t &link_index);

    TransferDescriptor *active_transfer() const;

    bool set_active_transfer_length(const uint32_t &len, const bool &in_bytes);

    int32_t active_transfer_length(const bool &in_bytes);

    ~Channel();

    protected:
      int32_t _index_ = -1;
      DmacChannel *ch = &DMAC->Channel[_index_];

      bool set_transfer_impl(TransferDescriptor** = nullptr, const uint32_t& = 0, 
        const bool& = false);
  };


///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: BLOCK DESCRIPTOR OBJ 
///////////////////////////////////////////////////////////////////////////////////////////////////

  typedef struct TransferDescriptor {
    
    typedef struct Config {
      void *src        = &detail::dummy_loc;
      void *dst        = &detail::dummy_loc;
      int src_inc      = -1;
      int dst_inc      = -1;
      int beat_size    = -1;
      int susp_ch      = -1;
    };

    TransferDescriptor();
    TransferDescriptor(const TransferDescriptor &other);
    TransferDescriptor(TransferDescriptor &&other);

    TransferDescriptor &operator = (const TransferDescriptor &other);       
    TransferDescriptor &operator = (TransferDescriptor &&other);

    bool set_config(const Config &cfg);

    Config config() const;

    void reset();
    
    bool set_length(const uint32_t &len, const bool &in_bytes);
    
    uint32_t length(const bool &in_bytes) const;

    bool valid() const;
  
    void unlink();

    ~TransferDescriptor();

    DmacDescriptor _desc_{}; 
    DmacDescriptor *_descPtr_ = &_desc_;
    TransferDescriptor *_next_;
    int _assig_ = -1;
  };


}


*/


