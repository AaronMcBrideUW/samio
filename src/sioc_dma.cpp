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


#include <sioc_dma.h> 

namespace sioc::dma { // START OF SIOC::DMA NAMESPACE

  //// USING STATEMENTS ////
  using namespace sioc::dma::hwref;

  //// MISC REF VARIABLES ////
  inline constexpr size_t _dsize_ = sizeof(DmacDescriptor);

  //// PRE-ALLOCATED STORAGE ////
  uint32_t ch_msk{};
  callback_t _cbarr_[DMAC_CH_NUM]{nullptr};
  TransferDescriptor *_btd_[DMAC_CH_NUM]{nullptr};
  DmacDescriptor _wbdesc_[DMAC_CH_NUM]{};
  DmacDescriptor _bdesc_[DMAC_CH_NUM]{};

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: UTILITY FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  void clear_channel(const uint32_t &ch_index) {
    DmacChannel *ch = &DMAC->Channel[ch_index];
    ch->CHCTRLA.bit.ENABLE = 0U;
      while(ch->CHCTRLA.bit.ENABLE);
    ch->CHCTRLA.bit.SWRST = 1U;
      while(ch->CHCTRLA.bit.SWRST);

    set_transfer(ch_index, nullptr);
    memset(&_bdesc_[ch_index], 0U, _dsize_);
    memset(&_wbdesc_[ch_index], 0U, _dsize_);
    _btd_[ch_index] = nullptr;
  }

  bool update_valid(DmacDescriptor *desc_ptr) {
    if (!desc_ptr) [[unlikely]] return false;
    uint8_t desc_valid = desc_ptr->SRCADDR.reg 
      && desc_ptr->DSTADDR.reg && desc_ptr->BTCNT.reg;
    desc_ptr->BTCTRL.bit.VALID = desc_valid;
    return desc_valid;
  }

  uintptr_t addr_mod(DmacDescriptor *desc, const bool &is_src) {
    if (!desc || !(is_src ? desc->BTCTRL.bit.SRCINC 
      : desc->BTCTRL.bit.DSTINC)) return 0;

    bool sel = (desc->BTCTRL.bit.STEPSEL == is_src 
      ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val);
    return desc->BTCNT.reg * (_beat_size_map_[desc->BTCTRL.bit.BEATSIZE] + 1)
      * (sel ? (1 << desc->BTCTRL.bit.STEPSIZE) : 1);    
  }

  void revalidate_descriptors(const uint32_t &ch_index) {
    TransferDescriptor *curr = _btd_[ch_index];
    do { 
      update_valid(curr->_descPtr_);
      curr = curr->_next_;
    } while(curr && curr != _btd_[ch_index]);
  }

  struct CriticalSection {

    CriticalSection(const int32_t &ch_index) : _index_(ch_index) {
      if (ch_index < 0 || ch_index >= DMAC_CH_NUM) return;
      DmacChannel *ch = &DMAC->Channel[ch_index];
      if (!ch->CHINTFLAG.bit.SUSP && ch->CHCTRLA.bit.ENABLE) {
        _flag_ = true;
        ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
        while(!ch->CHINTFLAG.bit.SUSP);
      }
    }

    ~CriticalSection() {
      if (!_flag_) return;
      DmacChannel *ch = &DMAC->Channel[_index_];
      ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
      ch->CHINTFLAG.bit.SUSP = 1U;
    }
    
    bool _flag_ = false;
    const uint32_t _index_;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: INTERRUPT HANDLER
///////////////////////////////////////////////////////////////////////////////////////////////////

  void dma_irq_handler() {
    using namespace sioc::dma;
    using enum callback_flag_e;
    uint32_t index = DMAC->INTPEND.bit.ID;
    callback_flag_e flag = null;
    
    if (DMAC->INTPEND.bit.TCMPL) {
      DMAC->Channel[index].CHINTFLAG.bit.TCMPL = 1U;
      if (_wbdesc_[index].DESCADDR.reg == DMAC_SRCADDR_RESETVALUE) {
        revalidate_descriptors(index);
      }
    } else if (DMAC->INTPEND.bit.TERR) { 
      DMAC->Channel[index].CHINTFLAG.bit.TERR; 
      flag = (DMAC->INTPEND.bit.FERR) ? descriptor_error : transfer_error;
    }
    if (flag != null && _cbarr_[index]) {
      _cbarr_[index](index, flag);
    }
  }

  void DMAC_0_Handler() [[weak]];		
  void DMAC_0_Handler() { dma_irq_handler(); }
  
  void DMAC_1_Handler(void) [[irq, weak, alias("sioc::dma::DMAC_0_Handler")]];
  void DMAC_2_Handler(void) [[irq, weak, alias("sioc::dma::DMAC_0_Handler")]];
  void DMAC_3_Handler(void) [[irq, weak, alias("sioc::dma::DMAC_0_Handler")]];
  void DMAC_4_Handler(void) [[irq, weak, alias("sioc::dma::DMAC_0_Handler")]];

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: SYSTEM CONTROL FUNCTIONS @
///////////////////////////////////////////////////////////////////////////////////////////////////

  /// @b FINAL_V1
  int32_t allocate_channel() {
    if (ch_msk == 0x0) [[unlikely]] {
      /// Initialize DMA
      DMAC->Channel[0].CHCTRLA.bit.ENABLE = 0U;
        while(DMAC->Channel[0].CHCTRLA.bit.ENABLE);
      DMAC->Channel[0].CHCTRLA.bit.SWRST = 1U;
        while(DMAC->Channel[0].CHCTRLA.bit.SWRST);

      memset(&_bdesc_, 0, sizeof(_bdesc_));
      memset(&_wbdesc_, 0, sizeof(_wbdesc_));

      for (uint32_t i = 0; i < DMAC_LVL_NUM; i++) {
        NVIC_ClearPendingIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_EnableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
      }
      DMAC->BASEADDR.reg = (uintptr_t)&_bdesc_;
      DMAC->WRBADDR.reg = (uintptr_t)&_wbdesc_;
      MCLK->AHBMASK.bit.DMAC_ = 1U;
      DMAC->CTRL.bit.DMAENABLE = 1U;

      /// Set DMA config
      DMAC->PRICTRL0.reg |= []() consteval {
        uint32_t msk = 0x0;
        for (uint32_t i = 0; i < DMAC_LVL_NUM; i++) {
          uint32_t rr_shift = (i * (DMAC_PRICTRL0_RRLVLEN1_Pos
            - DMAC_PRICTRL0_RRLVLEN0_Pos)) + DMAC_PRICTRL0_RRLVLEN0_Pos;
          msk |= (prilvl_round_robin_mode[i] << rr_shift);

          uint32_t sq_shift = (i * (DMAC_PRICTRL0_QOS1_Pos 
            - DMAC_PRICTRL0_QOS0_Pos)) + DMAC_PRICTRL0_QOS0_Pos;
          msk |= (prilvl_service_quality[i] << sq_shift);
        }
        return msk;
      }();
      for (uint32_t i = 0; i < _irqcnt_; i++) {
        NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), irq_priority[i]);
      }
    } /// Update channel mask
    for (uint32_t i = 0; i < DMAC_CH_NUM; i++) {
      if ((ch_msk & (1U << i)) == 0) {    
        ch_msk |= (1U << i);
        reset_channel(i);
        return i;
      }
    }
    return -1;
  }
  
  /// @b FINAL_V1
  bool free_channel(const uint32_t &ch_index) {
    if ((ch_msk & (1U << ch_index)) == 0x0) [[unlikely]] return false;
    ch_msk &= ~(1U << ch_index);
    clear_channel(ch_index);

    if (ch_msk == 0x0) { // Un-initialize DMA 
      DMAC->CTRL.bit.DMAENABLE = 0U;
        while(DMAC->CTRL.bit.DMAENABLE);
      DMAC->CTRL.bit.SWRST = 1U;
        while(DMAC->CTRL.bit.SWRST);
      MCLK->AHBMASK.bit.DMAC_ = 0U;

      for (uint32_t i = 0; i < DMAC_LVL_NUM; i++) {
        NVIC_DisableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_ClearPendingIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), 
          (1 << __NVIC_PRIO_BITS) - 1);
      }
    }
    return true;
  }

  /// @b FINAL_V1
  uint32_t free_channel_count() {
    return DMAC_CH_NUM - std::popcount(ch_msk);
  }
 
  /// @b FINAL_V1
  int32_t active_channel_index() {
    return std::countr_zero(DMAC->BUSYCH.reg) - 1;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHANNEL CONTROL FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  /// @b FINAL_V1
  bool set_channel_state(const uint32_t &ch_index, const channel_state_e &state) {
    if (DMAC->CTRL.bit.DMAENABLE == 0U) [[unlikely]] return false;
    if (channel_state(ch_index) == state) return true;

    DmacChannel *ch = &DMAC->Channel[ch_index];
    switch(state) {
      case channel_state_e::disabled: {
        ch->CHCTRLA.bit.ENABLE = 0U;
          while(ch->CHCTRLA.bit.ENABLE);
        ch->CHINTFLAG.reg &= ~DMAC_CHINTFLAG_MASK;

      } case channel_state_e::idle: {
        ch->CHCTRLA.bit.ENABLE = 0U;
          while(ch->CHCTRLA.bit.ENABLE);
        ch->CHCTRLA.bit.ENABLE = 1U;
        ch->CHINTFLAG.bit.SUSP = 1U;

      } case channel_state_e::suspended: {
        ch->CHCTRLA.bit.ENABLE = 1U;
        ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
          while(!ch->CHINTFLAG.bit.SUSP);

      } case channel_state_e::active: {
        ch->CHCTRLA.bit.ENABLE = 1U;
        if (ch->CHINTFLAG.bit.SUSP) {
          ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
          ch->CHINTFLAG.bit.SUSP = 1U;
        }
        DMAC->SWTRIGCTRL.reg |= (1 <<   
          (DMAC_SWTRIGCTRL_SWTRIG0_Pos + ch_index));
      } 
    }
    return true;
  }

  /// @b FINAL_V1
  channel_state_e channel_state(const uint32_t &ch_index) {
    DmacChannel *ch = &DMAC->Channel[ch_index];
    if (ch->CHSTATUS.bit.BUSY) {
      return channel_state_e::active;
    }
    if (!ch->CHCTRLA.bit.ENABLE) {
      return channel_state_e::disabled;

    } else if (ch->CHINTFLAG.bit.SUSP) {
      return channel_state_e::suspended;

    } else if (ch->CHSTATUS.bit.PEND) {
      return channel_state_e::active;
    }
    return channel_state_e::idle; 
  }

  /// @b FINAL_V1
  bool set_channel_config(const uint32_t &ch_index, const ChannelConfig &cfg) {
    uint32_t burst_len_r = 0, pri_lvl_r = 0;

    if (cfg.burst_len >= 0) {
      burst_len_r = std::distance(_burst_len_map_.begin(), std::find
      (_burst_len_map_.begin(), _burst_len_map_.end(), cfg.burst_len));
      if (burst_len_r == _burst_len_map_.size()) [[unlikely]] return false;
    }
    if (cfg.pri_lvl >= 0) {
      pri_lvl_r = std::distance(_pri_lvl_map_.begin(), std::find
      (_pri_lvl_map_.begin(), _pri_lvl_map_.end(), cfg.pri_lvl));
      if (pri_lvl_r == _pri_lvl_map_.size()) [[unlikely]] return false;
    }
    DmacChannel *ch = &DMAC->Channel[ch_index];
    if (cfg.burst_len >= 0) ch->CHCTRLA.bit.BURSTLEN = burst_len_r;
    if (cfg.pri_lvl >= 0) ch->CHPRILVL.bit.PRILVL = pri_lvl_r;
    if (cfg.callback) _cbarr_[ch_index] = cfg.callback;

    if (cfg.periph != linked_peripheral_e::null) {
      bool en_flag =false;
      if (DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE) {
        en_flag = true;
        DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE = 0U;
        while(DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE);
      }
      ch->CHCTRLA.bit.TRIGSRC = (uint32_t)cfg.mode;
      if (en_flag) {
        DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE = 1U;
      }
    }
    if (cfg.mode != transfer_mode_e::null) {
      ch->CHCTRLA.bit.TRIGACT = (uint32_t)cfg.periph;
    }
    return true;
  }

  /// @b FINAL_V1
  ChannelConfig channel_config(const uint32_t &ch_index) {
    DmacChannel *ch = &DMAC->Channel[ch_index];
    return ChannelConfig{
      .periph    = (linked_peripheral_e)ch->CHCTRLA.bit.TRIGSRC,
      .mode      = (transfer_mode_e)ch->CHCTRLA.bit.TRIGACT,
      .burst_len = (int32_t)_burst_len_map_[ch->CHCTRLA.bit.BURSTLEN],
      .pri_lvl   = (int32_t)_pri_lvl_map_[ch->CHPRILVL.bit.PRILVL],
      .callback = _cbarr_[ch_index],
    };
  }

  /// @b FINAL_V1
  void reset_channel(const uint32_t &ch_index) {
    DmacChannel *ch = &DMAC->Channel[ch_index];
    clear_channel(ch_index);

    ch->CHCTRLA.bit.TRIGSRC = (uint32_t)ch_def_periph;
    ch->CHCTRLA.bit.TRIGACT = (uint32_t)ch_def_mode;
    ch->CHCTRLA.bit.BURSTLEN = (uint32_t)ch_def_burst_len;
    ch->CHPRILVL.bit.PRILVL = (uint32_t)ch_def_pri_lvl;

    ch->CHINTENSET.bit.TCMPL = 1U;
    // ch->CHINTENSET.bit.TERR = 1U; -> CURRNETLY NOT WORKING
  }

  /// @b FINAL_V1
  bool skip_suspend(const uint32_t &ch_index) {
    DmacChannel *ch = &DMAC->Channel[ch_index];
    if (!ch->CHCTRLA.bit.ENABLE || ch->CHINTFLAG.bit.SUSP) return false;
    ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
    return true;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER DESCRIPTOR OBJ
///////////////////////////////////////////////////////////////////////////////////////////////////

  /// @b FINAL_V1
  TransferDescriptor::TransferDescriptor(const TransferDescriptor &other) {
    this->operator = (other);
  } 

  /// @b FINAL_V1
  TransferDescriptor::TransferDescriptor(TransferDescriptor &&other) {
    this->operator = (std::move(other));
  }

  /// @b FINAL_V1
  TransferDescriptor &TransferDescriptor::operator = (const TransferDescriptor &other) {
    if (this == &other) return *this;
    _descPtr_->BTCTRL.bit.VALID = 0U;
    
    auto prev_link = _descPtr_->DESCADDR.reg;
    memcpy(_descPtr_, other._descPtr_, _dsize_);
    _descPtr_->DESCADDR.reg = prev_link;

    update_valid(_descPtr_);
    return *this;
  }
   
  /// @b FINAL_V1
  TransferDescriptor &TransferDescriptor::operator = (TransferDescriptor &&other) {
    if (this == &other) return *this;
    if (_assig_ != -1) unlink();
    other._descPtr_->BTCTRL.bit.VALID = 0U;
   
    if (other._assig_ != -1 && _btd_[other._assig_] == &other) {
      _descPtr_ = std::exchange(other._descPtr_, &other._desc_);
      _btd_[other._assig_] = this;
    } else {
      memcpy(&_desc_, &other._desc_, _dsize_);

      if (other._assig_ != -1) {
        TransferDescriptor *prev = _btd_[other._assig_];
        while (prev->_next_ != &other) prev = prev->_next_; 

        prev->_descPtr_->BTCTRL.bit.VALID = 0U;
        prev->_descPtr_->DESCADDR.reg = (uintptr_t)&_desc_; 
        prev->_next_ = this; 

        if (_wbdesc_[other._assig_].SRCADDR.reg) {
          _wbdesc_->DESCADDR.reg = (uintptr_t)&_desc_;
        }
        update_valid(prev->_descPtr_);
      }
    }
    _assig_ = std::exchange(other._assig_, -1);
    _next_ = std::exchange(other._next_, nullptr);
    update_valid(_descPtr_);
    return *this;
  }

  /// @b FINAL_V1
  bool TransferDescriptor::set_config(const Config &cfg) {
    if (cfg.src_inc > 1 && cfg.dst_inc > 1) return false;
    
    uint32_t beatsize_r = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
    if (cfg.beat_size >= 0) {
      beatsize_r = std::distance(_beat_size_map_.begin(), std::find
        (_beat_size_map_.begin(), _beat_size_map_.end(), cfg.beat_size));
      if (beatsize_r == _beat_size_map_.size()) return false;
    }
    int32_t max_inc = std::max(cfg.src_inc, cfg.dst_inc); 
    uint32_t stepsize_r = DMAC_BTCTRL_STEPSIZE_X1_Val;
    if (max_inc > 1) {
      stepsize_r = std::distance(_step_size_map_.begin(), std::find
        (_step_size_map_.begin(), _step_size_map_.end(), max_inc));
      if (stepsize_r >= _step_size_map_.size()) return false;
    }
    _descPtr_->BTCTRL.bit.VALID = 0U;
    if (cfg.src_inc >= 0) _descPtr_->BTCTRL.bit.SRCINC   = cfg.src_inc;
    if (cfg.dst_inc >= 0) _descPtr_->BTCTRL.bit.DSTINC   = cfg.dst_inc;
    if (max_inc >= 0) _descPtr_->BTCTRL.bit.STEPSIZE = stepsize_r;
    if (max_inc >= 0) _descPtr_->BTCTRL.bit.STEPSEL  = cfg.src_inc > cfg.dst_inc
      ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;

    if (cfg.beat_size >= 0) _descPtr_->BTCTRL.bit.BEATSIZE = beatsize_r;
    if (cfg.susp_ch >= 0) _descPtr_->BTCTRL.bit.BLOCKACT = cfg.susp_ch 
      ? DMAC_BTCTRL_BLOCKACT_SUSPEND_Val : DMAC_BTCTRL_BLOCKACT_NOACT_Val;

    if (cfg.src != &detail::dummy_loc) [[likely]] {
      _descPtr_->SRCADDR.reg = cfg.src ? (uintptr_t)cfg.src 
        + addr_mod(_descPtr_, true) : DMAC_SRCADDR_RESETVALUE;
    }
    if (cfg.dst != &detail::dummy_loc) [[likely]] {
      _descPtr_->DSTADDR.reg  = cfg.dst ? (uintptr_t)cfg.dst
        + addr_mod(_descPtr_, false) : DMAC_SRCADDR_RESETVALUE;
    }
    update_valid(_descPtr_);
    return true;
  }

  /// @b FINAL_V1
  TransferDescriptor::Config TransferDescriptor::config() const {
    uint32_t stepsize_v = _step_size_map_[_descPtr_->BTCTRL.bit.STEPSIZE];
    return Config{
      .src = _descPtr_->SRCADDR.reg == DMAC_SRCADDR_RESETVALUE ? nullptr
        : (void*)(_descPtr_->SRCADDR.reg - addr_mod(_descPtr_, true)),
      .dst = _descPtr_->DSTADDR.reg == DMAC_SRCADDR_RESETVALUE ? nullptr
        : (void*)(_descPtr_->DSTADDR.reg - addr_mod(_descPtr_, false)),

      .src_inc = (int)(_descPtr_->BTCTRL.bit.SRCINC * _descPtr_
        ->BTCTRL.bit.STEPSEL == DMAC_BTCTRL_STEPSEL_SRC_Val ? stepsize_v : 1),
      .dst_inc = (int)(_descPtr_->BTCTRL.bit.DSTINC * _descPtr_
        ->BTCTRL.bit.STEPSEL == DMAC_BTCTRL_STEPSEL_DST_Val ? stepsize_v : 1),

      .beat_size = (int)_beat_size_map_[_descPtr_->BTCTRL.bit.BEATSIZE],
      .susp_ch = _descPtr_->BTCTRL.bit.BLOCKACT == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val,
    };
  }

  /// @b FINAL_V1
  bool TransferDescriptor::set_length(const uint32_t &len, const bool &in_bytes) {
    _descPtr_->BTCTRL.bit.VALID = 0U;
    uint32_t new_cnt = len * (in_bytes ? _beat_size_map_
      [_descPtr_->BTCTRL.bit.BEATSIZE] : 1);

    if (new_cnt < max_td_length) {
      uint32_t s_mod = addr_mod(_descPtr_, true);
      uintptr_t d_mod = addr_mod(_descPtr_, false);

      _descPtr_->BTCNT.reg = new_cnt;
      _descPtr_->SRCADDR.reg += (addr_mod(_descPtr_, true) - s_mod);
      _descPtr_->DSTADDR.reg += (addr_mod(_descPtr_, false) - d_mod);
    }
    update_valid(_descPtr_);
    return new_cnt < max_td_length;
  }

  /// @b FINAL_V1
  uint32_t TransferDescriptor::length(const bool &in_bytes) const {
    return _descPtr_->BTCNT.reg * (in_bytes ? _beat_size_map_
      [_descPtr_->BTCTRL.bit.BEATSIZE] : 1);
  }

  bool TransferDescriptor::valid() const {
    update_valid(_descPtr_);
    return _descPtr_->BTCTRL.bit.VALID;
  }

  /// @b UPDATED
  void TransferDescriptor::unlink() {
    if (_assig_ == -1) return; 

    TransferDescriptor *prev = _btd_[_assig_];
    while(prev->_next_ && prev->_next_ != this) {
      prev = prev->_next_;
    }
    _descPtr_->BTCTRL.bit.VALID = 0U;
    if (prev && prev != this) {
      prev->_descPtr_->BTCTRL.bit.VALID = 0U;
    } 
    if (this == _btd_[_assig_]) {    
      memcpy(&_desc_, _descPtr_, _dsize_);
      _descPtr_ = &_desc_;
      
      if (_next_ && _next_ != this) {
        memcpy(&_bdesc_[_assig_], &_next_->_desc_, _dsize_);
        _next_->_descPtr_ = &_next_->_desc_;
        _btd_[_assig_] = _next_;
      } else {
        memset(&_bdesc_[_assig_], 0, _dsize_);
        memset(&_wbdesc_[_assig_], 0, _dsize_);
        _btd_[_assig_] = nullptr;
      }
    } else { /// ASSERT(prev && prev != this);
      prev->_next_ = _next_;  
      prev->_descPtr_->DESCADDR.reg = _descPtr_->DESCADDR.reg;

      if (_wbdesc_[_assig_].DESCADDR.reg == (uintptr_t)&_descPtr_) {
        _wbdesc_[_assig_].DESCADDR.reg = (uintptr_t)_next_->_descPtr_;
      }
    }
    _descPtr_->DESCADDR.reg = 0U;
    _next_ = nullptr;
    _assig_ = -1;

    update_valid(_descPtr_);
    if (prev && prev != this) {
      update_valid(prev->_descPtr_);
    }
  }

  TransferDescriptor::~TransferDescriptor() {
    if (_assig_ != -1) unlink();
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DESCRIPTOR CONTROL FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  /// @b UPDATED
  void detail::set_transfer(const uint32_t &ch_index, TransferDescriptor
    **desc_list, const uint32_t &length, const bool &looped) {
    DmacChannel *ch = &DMAC->Channel[ch_index];

    // if (ch->CHSTATUS.bit.BUSY || ch->CHSTATUS.bit.PEND)


    if (_btd_[ch_index]) {
      TransferDescriptor *curr = _btd_[ch_index];
      memcpy(&curr->_desc_, &_bdesc_[ch_index], _dsize_);
      curr->_descPtr_ = &curr->_desc_;
      do {
        curr->_assig_ = -1;
        curr->_descPtr_->DESCADDR.reg = 0U;
        curr = std::exchange(curr->_next_, nullptr);
      } while(curr && curr != _btd_[ch_index]);

      memset(&_bdesc_[ch_index], 0U, _dsize_);
      memset(&_wbdesc_[ch_index], 0U, _dsize_);
      _btd_[ch_index] = nullptr;
    }
    if (desc_list && length) {
      TransferDescriptor *prev = nullptr;

      for (uint32_t i = 0; i < length; i++) {
        TransferDescriptor *curr = desc_list[i];
        if (!curr) [[unlikely]] continue;
        if (curr->_assig_ != -1) [[unlikely]] curr->unlink();

        if (prev) {
          prev->_next_ = curr;
          prev->_descPtr_->DESCADDR.reg = (uintptr_t)curr->_descPtr_;
        } else {  
          memcpy(&_bdesc_[ch_index], curr->_descPtr_, _dsize_);
          curr->_descPtr_ = &_bdesc_[ch_index];
          _btd_[ch_index] = curr;
        }
        curr->_assig_ = ch_index;
        update_valid(curr->_descPtr_);
        prev = curr;
      }
      if (looped && prev) {
        prev->_next_ = _btd_[ch_index];
        prev->_descPtr_->DESCADDR.reg 
          = (uintptr_t)_btd_[ch_index]->_descPtr_;
      }
    }
  }

  void set_transfer(const uint32_t ch_index, TransferDescriptor *tdesc,
    const bool &looped) {
    TransferDescriptor *desc_array = tdesc;
    detail::set_transfer(ch_index, &desc_array, 1, looped);
  }

  bool set_active_transfer(const uint32_t &ch_index, const uint32_t &td_index) {
    if (!_btd_[ch_index]) return false;
    auto crit = CriticalSection(ch_index);
    TransferDescriptor *curr = _btd_[ch_index];

    for (uint32_t i = 0; i < td_index; i++) {
      if (!curr->_next_ || curr->_next_ == _btd_[ch_index]) return false;
      curr = curr->_next_;
    }
    memcpy(&_wbdesc_[ch_index], curr->_descPtr_, _dsize_);
    return true;
  }

  bool set_active_transfer_length(const uint32_t &ch_index,
    const uint32_t &len, const bool &in_bytes) {
    if (!_btd_[ch_index] || !_wbdesc_[ch_index].SRCADDR.reg) return false;

    auto crit = CriticalSection(ch_index);
    _wbdesc_[ch_index].BTCNT.reg = len * (in_bytes ? _beat_size_map_
      [active_transfer(ch_index)->_descPtr_->BTCTRL.bit.BEATSIZE] : 1);
    return true;
  }

  uint32_t active_transfer_length(const uint32_t &ch_index, 
    const bool &in_bytes) {
    if (!_btd_[ch_index] || _wbdesc_[ch_index].SRCADDR.reg) return 0;

    auto crit = CriticalSection(ch_index);
    uint32_t bs = _beat_size_map_[_wbdesc_[ch_index].BTCTRL.bit.BEATSIZE];
    return std::div(_wbdesc_[ch_index].BTCNT.reg, (in_bytes ? bs : 1)).quot;
  }

  TransferDescriptor *active_transfer(const uint32_t &ch_index) {
    if (_btd_[ch_index] && _wbdesc_[ch_index].SRCADDR.reg) {
      TransferDescriptor *curr = _btd_[ch_index];
      do {
        if (curr->_descPtr_->DESCADDR.reg == 
          _wbdesc_[ch_index].DESCADDR.reg) {
          return curr;
        }
      } while(curr && curr != _btd_[ch_index]);
    } 
    return nullptr;
  }


} // END OF SIOC::DMA NAMESPACE