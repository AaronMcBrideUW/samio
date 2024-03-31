/*
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

using namespace sioc::dma::hwref;

namespace sioc::dma {

  //// MISC REF VARIABLES ////
  inline constexpr size_t _dsize_ = sizeof(DmacDescriptor);

  //// PRE-ALLOCATED STORAGE ////   
  uint32_t ch_msk{};
  callback_t _cbarr_[DMAC_CH_NUM]{nullptr};
  TransferDescriptor *_btd_[DMAC_CH_NUM]{nullptr};
  volatile SECTION_DMAC_DESCRIPTOR __ALIGNED(16) DmacDescriptor _wbdesc_[DMAC_CH_NUM]{};
  SECTION_DMAC_DESCRIPTOR __ALIGNED(16) DmacDescriptor _bdesc_[DMAC_CH_NUM]{};

  namespace {

    [[always_inline]]
    bool writeback_valid(const uint32_t &ch_index) {
      volatile DmacDescriptor &wb = _wbdesc_[ch_index];
      return !wb.SRCADDR.reg || wb.DESCADDR.reg || wb.BTCNT.reg;
    }

    [[always_inline]]
    void assign_base_td(const uint32_t &ch_index, TransferDescriptor *td) {
      memcpy(&_bdesc_[ch_index], &td->_desc_, _dsize_);
      td->_descPtr_ = &_bdesc_[ch_index];
      _btd_[ch_index] = td;
    }

    [[always_inline]]
    TransferDescriptor *remove_base_td(const uint32_t &ch_index, const bool &clear_btd) {
      memcpy(&_btd_[ch_index]->_desc_, &_bdesc_[ch_index], _dsize_);
      _btd_[ch_index]->_descPtr_ = &_btd_[ch_index]->_desc_;
      return clear_btd ? std::exchange(_btd_[ch_index], nullptr) : _btd_[ch_index];
    }

    [[always_inline]]
    void link_td(TransferDescriptor *const prev, TransferDescriptor *const targ) {
      prev->_descPtr_->DESCADDR.reg = targ ? (uintptr_t)targ->_descPtr_ : 0U;
      prev->_next_ = targ;
    } 

    [[always_inline]]
    TransferDescriptor *unlink_td(TransferDescriptor *td) {
      td->_assig_ = -1;
      td->_descPtr_->DESCADDR.reg = 0U;
      return std::exchange(td->_next_, nullptr);
    }

    [[always_inline]]
    void update_valid(DmacDescriptor *desc_ptr) {
      if (!desc_ptr) [[unlikely]] return;
      desc_ptr->BTCTRL.bit.VALID = desc_ptr->SRCADDR.reg && 
        desc_ptr->DSTADDR.reg && desc_ptr->BTCNT.reg;
    }

    void relink_writeback(const uint32_t &ch_index, TransferDescriptor *new_link) {
      using enum channel_state_e;
      _wbdesc_[ch_index].DESCADDR.reg = new_link ? 
          (uintptr_t)new_link->_descPtr_ : DMAC_SRCADDR_RESETVALUE;

      if (!writeback_valid(ch_index) && DMAC->Channel[ch_index]
        .CHINTFLAG.bit.SUSP && DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE) {
        DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE = 0U;
        while(DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE);
        DMAC->Channel[ch_index].CHCTRLA.bit.ENABLE = 1U;
      }
    }

    uintptr_t addr_mod(DmacDescriptor *desc, const bool &is_src) {
      if (!desc || !(is_src ? desc->BTCTRL.bit.SRCINC 
        : desc->BTCTRL.bit.DSTINC)) return 0;

      bool sel = (desc->BTCTRL.bit.STEPSEL == is_src 
        ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val);
      return desc->BTCNT.reg * (_beat_size_map_[desc->BTCTRL.bit.BEATSIZE])
        * (sel ? (1 << desc->BTCTRL.bit.STEPSIZE) : 1);    
    }

    typedef struct SuspendSection {
      SuspendSection(const int32_t &ch_index) : _index_(ch_index) {
        if (ch_index < 0 || ch_index >= DMAC_CH_NUM) return;
        if (!DMAC->Channel[_index_].CHINTFLAG.bit.SUSP 
          && DMAC->Channel[_index_].CHCTRLA.bit.ENABLE) {
          _flag_ = true;
          DMAC->Channel[_index_].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
          while(!DMAC->Channel[ch_index].CHINTFLAG.bit.SUSP);
        }
      }
      ~SuspendSection() {
        if (_flag_ && DMAC->Channel[_index_].CHINTFLAG.bit.SUSP) {
          DMAC->Channel[_index_].CHINTFLAG.bit.SUSP = 1U;
          DMAC->Channel[_index_].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
        } 
      }
      bool _flag_ = false;
      const int32_t _index_ = -1;
    };

  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: MISC FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////
  
  uint32_t free_channel_count() {
    return DMAC_CH_NUM - std::popcount(ch_msk);
  }
 
  int32_t active_channel_index() {
    return std::countr_zero(DMAC->BUSYCH.reg) - 1;

  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHANNEL CONTROL FUNCTIONS 
///////////////////////////////////////////////////////////////////////////////////////////////////

  #define require_index(_ret_) if (_index_ < 0 || _index_ >= max_channels) return _ret_

  Channel::Channel() : Channel([] -> int32_t {
    for (uint32_t i = 0; i < DMAC_CH_NUM; i++) {
      if ((ch_msk & (1U << i)) == 0U) return i;
    } 
    return -1;
  }()){};

  Channel::Channel(const int32_t &index) : _index_(index) { 
    if (ch_msk == 0U) [[unlikely]] {
      DMAC->CTRL.bit.DMAENABLE = 0U;
      while(DMAC->CTRL.bit.DMAENABLE);
      DMAC->CTRL.bit.SWRST = 1U;
      while(DMAC->CTRL.bit.SWRST);
      
      memset(&_bdesc_, 0U, sizeof(_bdesc_));
      memset((void*)&_wbdesc_, 0U, sizeof(_wbdesc_));

      for (uint32_t i = 0; i < _irqcnt_; i++) {
        NVIC_ClearPendingIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), irq_priority[i]);
        NVIC_EnableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
      }
      auto [prictrl_cfg, ctrl_cfg] = []() consteval {
        uint32_t prictrl_msk = 0U, ctrl_msk = 0U;

        for (uint32_t i = 0; i < DMAC_LVL_NUM; i++) {
          uint32_t lvl_shift = i * (DMAC_CTRL_LVLEN1_Pos 
            - DMAC_CTRL_LVLEN0_Pos) + DMAC_CTRL_LVLEN0_Pos;
          uint32_t rr_shift = i * (DMAC_PRICTRL0_RRLVLEN1_Pos
            - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos;
          uint32_t sq_shift = i * (DMAC_PRICTRL0_QOS1_Pos 
            - DMAC_PRICTRL0_QOS0_Pos) + DMAC_PRICTRL0_QOS0_Pos;

          ctrl_msk    |= (prilvl_enabled[i] << lvl_shift);
          prictrl_msk |= (prilvl_round_robin_mode[i] << rr_shift);
          prictrl_msk |= (prilvl_service_quality[i] << sq_shift);
        }
        return std::make_pair(prictrl_msk, ctrl_msk);
      }();
      DMAC->BASEADDR.reg  = (uintptr_t)&_bdesc_;
      DMAC->WRBADDR.reg   = (uintptr_t)&_wbdesc_;
      DMAC->PRICTRL0.reg  = prictrl_cfg; 
      DMAC->CTRL.reg     |= ctrl_cfg;

      MCLK->AHBMASK.bit.DMAC_  = 1U;
      DMAC->CTRL.bit.DMAENABLE = 1U;
    }
    if ((ch_msk & (1U << index)) == 0) {
      ch_msk |= (1U << index);
      reset(true);
    }
  }

  Channel::Channel(const Channel &other) {
    this->operator=(other);
  }

  Channel &Channel::operator = (const Channel &other) {
    if (this != &other) _index_ = other._index_;
    return *this;
  }

  Channel::Channel(Channel &&other) {
    this->operator=(std::move(other));
  }

  Channel &Channel::operator=(Channel &&other) {
    if (this != &other) std::swap(_index_, other._index_);
    return *this;
  }

  bool Channel::set_state(const channel_state_e &state) {
    using enum channel_state_e;

    if (state != this->state()) {
      if (state == suspended) {
        ch->CHCTRLA.bit.ENABLE = 1U;
        ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
        while(!ch->CHINTFLAG.bit.SUSP);
      } else {
        ch->CHCTRLA.bit.ENABLE = (uint32_t)(state == enabled);
        if (ch->CHINTFLAG.bit.SUSP) {
          ch->CHINTFLAG.bit.SUSP = 1U;
          ch->CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
        }
      }
    }
    return true;
  }

  channel_state_e Channel::state() const {
    using enum channel_state_e;
    if (!ch->CHCTRLA.bit.ENABLE) {
      return disabled;
    } else if (ch->CHINTFLAG.bit.SUSP) {
      return suspended;
    }
    return enabled;
  }

  void Channel::trigger() {
    DMAC->SWTRIGCTRL.reg |= (1 << (_index_ + DMAC_SWTRIGCTRL_SWTRIG0_Pos));
  }

  bool Channel::trigger_pending() const {
    return ch->CHSTATUS.bit.PEND;
  }

  bool Channel::transfer_busy() const {
    return ch->CHSTATUS.bit.BUSY && !ch->CHINTFLAG.bit.SUSP;
  }

  /// @b COMPLETE
  bool Channel::set_config(const ChannelConfig &cfg) {
    uint32_t burst_len_r = 0U, pri_lvl_r = 0U;
    if (cfg.burst_len >= 0) {
      burst_len_r = std::distance(_burst_len_map_.begin(), std::find
        (_burst_len_map_.begin(), _burst_len_map_.end(), cfg.burst_len));
      if (burst_len_r == _burst_len_map_.size()) [[unlikely]] return false;
      ch->CHCTRLA.bit.BURSTLEN = burst_len_r;
    }
    if (cfg.pri_lvl >= 0) {
      pri_lvl_r = std::distance(_pri_lvl_map_.begin(), std::find
        (_pri_lvl_map_.begin(), _pri_lvl_map_.end(), cfg.pri_lvl));
      if (pri_lvl_r == _pri_lvl_map_.size()) [[unlikely]] return false;
      ch->CHPRILVL.bit.PRILVL = pri_lvl_r;
    }   
    if (cfg.mode != transfer_mode_e::null) {
      ch->CHCTRLA.bit.TRIGACT = (uint32_t)cfg.mode;
    }
    if (cfg.callback != detail::dummy_callback) {
      _cbarr_[_index_] = cfg.callback;
    }
    if (cfg.periph != linked_peripheral_e::null) {
      bool en_flag = false;
      if (ch->CHCTRLA.bit.ENABLE) {
        ch->CHCTRLA.bit.ENABLE = 0U;
        while(ch->CHCTRLA.bit.ENABLE);
        en_flag = true;
      }
      ch->CHCTRLA.bit.TRIGSRC = (uint32_t)cfg.periph;
      if (en_flag) ch->CHCTRLA.bit.ENABLE = 1U;    
    }          
    return true;
  }

  /// @b COMPLETE
  bool Channel::set_config(const Channel &other) {
    require_index(false);
    if (this == &other) return false;
    auto en_state = ch->CHCTRLA.bit.ENABLE;

    ch->CHCTRLA.reg = other.ch->CHCTRLA.reg;
    ch->CHPRILVL.reg = other.ch->CHPRILVL.reg;
    _cbarr_[_index_] = _cbarr_[other._index_];

    ch->CHCTRLA.bit.ENABLE = en_state;
    while(ch->CHCTRLA.bit.ENABLE != en_state); 
    return true;
  }

  Channel::ChannelConfig Channel::config() {
    require_index(ChannelConfig{});
    return ChannelConfig{
      .periph    = (linked_peripheral_e)ch->CHCTRLA.bit.TRIGSRC,
      .mode      = (transfer_mode_e)ch->CHCTRLA.bit.TRIGACT,
      .burst_len = (int32_t)_burst_len_map_[ch->CHCTRLA.bit.BURSTLEN],
      .pri_lvl   = (int32_t)_pri_lvl_map_[ch->CHPRILVL.bit.PRILVL],
      .callback = _cbarr_[_index_],
    };
  }

  bool Channel::reset(const bool &clear_transfer) { 
    require_index(false);
    constexpr uint32_t burstlen_r = std::distance(_burst_len_map_.begin(), 
      std::find(_burst_len_map_.begin(), _burst_len_map_.end(), ch_def_burst_len));
    static_assert(burstlen_r < _burst_len_map_.size(), "Invalid config value.");

    constexpr uint32_t prilvl_r = std::distance(_pri_lvl_map_.begin(), 
      std::find(_pri_lvl_map_.begin(), _pri_lvl_map_.end(), ch_def_pri_lvl));
    static_assert(prilvl_r < _pri_lvl_map_.size(), "Invalid config value.");

    if (clear_transfer) {
      set_transfer(nullptr);
      _btd_[_index_] = nullptr;
      memset(&_bdesc_[_index_],  0U, _dsize_);
      memset((void*)&_wbdesc_[_index_], 0U, _dsize_);
    }
    ch->CHCTRLA.bit.ENABLE = 0U;
    while(ch->CHCTRLA.bit.ENABLE);
    ch->CHCTRLA.bit.SWRST = 1U;
    while(ch->CHCTRLA.bit.SWRST);

    ch->CHINTENSET.bit.TCMPL = 1U;
    ch->CHINTENSET.bit.TERR = 1U;
    ch->CHCTRLA.bit.RUNSTDBY = ch_run_in_standby[_index_];

    ch->CHCTRLA.bit.TRIGSRC  = (uint32_t)ch_def_periph;
    ch->CHCTRLA.bit.TRIGACT  = (uint32_t)ch_def_mode;
    ch->CHCTRLA.bit.BURSTLEN = burstlen_r;
    ch->CHPRILVL.bit.PRILVL  = prilvl_r;
    _cbarr_[_index_]        = nullptr;
    return true;
  }

  bool Channel::set_transfer(TransferDescriptor *tdesc, const bool &looped) {
    TransferDescriptor *desc_array = tdesc;
    return set_transfer_impl(&desc_array, 1, looped);
  }

  bool Channel::set_transfer(std::initializer_list<TransferDescriptor*> desclist,
    const bool &looped) {
    uint32_t i = 0;
    TransferDescriptor *desc_array[desclist.size()] = {nullptr};
    for (TransferDescriptor *curr : desclist) { desc_array[i++] = curr; } 
    return set_transfer_impl(desc_array, desclist.size(), looped);
  }

  bool Channel::set_active_transfer(const uint32_t &td_index) {
    require_index(false);
    if (!_btd_[_index_]) return false;
    auto crit = SuspendSection(_index_);

    TransferDescriptor *curr = _btd_[_index_];
    for (uint32_t i = 0; i < td_index; i++) {
      curr = curr->_next_;
      if (!curr || curr == _btd_[_index_]) return false;
    }
    memcpy((void*)&_wbdesc_[_index_], curr->_descPtr_, _dsize_);
    return true;
  }

  bool Channel::set_active_transfer(TransferDescriptor *td, 
    const int32_t &link_index) {
    require_index(false);
    if (!td) return false;
    auto crit = SuspendSection(_index_);
    uintptr_t link_addr = DMAC_SRCADDR_RESETVALUE;
    
    if (link_index >= 0) {
      if (!_btd_[_index_]) return false;
      TransferDescriptor *curr = _btd_[_index_];
      for (uint32_t i = 0; i < link_index; i++) {
        curr = curr->_next_;
        if (!curr || curr == _btd_[_index_]) return false;
      }
      link_addr = (uintptr_t)&curr->_descPtr_;
    }
    memcpy((void*)&_wbdesc_[_index_], td->_descPtr_, _dsize_);
    _wbdesc_[_index_].DESCADDR.reg = link_addr;
    return true;
  }

  TransferDescriptor *Channel::active_transfer() const {
    require_index(nullptr);
    auto crit = SuspendSection(_index_);
    TransferDescriptor *curr = _btd_[_index_]; 

    if (curr && _wbdesc_[_index_].BTCNT.reg) {
      while(curr->_descPtr_->DESCADDR.reg != (uintptr_t)&_wbdesc_[_index_]) {
        if (!curr->_next_ || curr->_next_ == _btd_[_index_]) 
          return curr; 

        curr = curr->_next_;
      }
    }
    return nullptr;
  }

  bool Channel::set_active_transfer_length(const uint32_t &len, 
    const bool &in_bytes) {
    require_index(false);
    auto crit = SuspendSection(_index_);

    if (!_btd_[_index_] || _wbdesc_[_index_].SRCADDR.reg 
      == DMAC_SRCADDR_RESETVALUE) return false;

    uintptr_t s_addr = _wbdesc_[_index_].SRCADDR.reg 
      - addr_mod((DmacDescriptor*)&_wbdesc_[_index_], true);
    uintptr_t d_addr = _wbdesc_[_index_].DSTADDR.reg
      - addr_mod((DmacDescriptor*)&_wbdesc_[_index_], false);

    if (in_bytes && len) {
      _wbdesc_[_index_].BTCNT.reg = len / _beat_size_map_
      [_wbdesc_[_index_].BTCTRL.bit.BEATSIZE];
    } else {
      _wbdesc_[_index_].BTCNT.reg = len;
    }
    _wbdesc_[_index_].SRCADDR.reg = s_addr 
      + addr_mod((DmacDescriptor*)&_wbdesc_[_index_], true);
    _wbdesc_[_index_].DSTADDR.reg = d_addr
      + addr_mod((DmacDescriptor*)&_wbdesc_[_index_], false);
    return true;
  }

  int32_t Channel::active_transfer_length(const bool &in_bytes) {
    require_index(-1);
    auto crit = SuspendSection(_index_);
    if (!_btd_[_index_] || _wbdesc_[_index_].SRCADDR.reg
      == DMAC_SRCADDR_RESETVALUE) return -1;

    return _wbdesc_[_index_].BTCNT.reg * (in_bytes ? _beat_size_map_
      [_wbdesc_[_index_].BTCTRL.bit.BEATSIZE] : 1); 
  }

  Channel::~Channel() {
    require_index();
    sio_assert((ch_msk & (1U << _index_)) == 0U);
    ch_msk &= ~(1U << _index_);
    
    if (!ch_msk) [[unlikely]] { 
      DMAC->CTRL.bit.DMAENABLE = 0U;
      while(DMAC->CTRL.bit.DMAENABLE);
      DMAC->CTRL.bit.SWRST = 1U;
      while(DMAC->CTRL.bit.SWRST);
      MCLK->AHBMASK.bit.DMAC_ = 0U;

      for (uint32_t i = 0; i < _irqcnt_; i++) {
        NVIC_DisableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_ClearPendingIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), 
          (1 << __NVIC_PRIO_BITS) - 1);
      }
    } else {
      set_transfer(nullptr);
      ch->CHCTRLA.bit.ENABLE = 0U;
      while(ch->CHCTRLA.bit.ENABLE);
      ch->CHCTRLA.bit.SWRST = 1U;
      while(ch->CHCTRLA.bit.SWRST);
    }
  }

  bool Channel::set_transfer_impl(TransferDescriptor **desc_list, 
    const uint32_t &length, const bool &looped) {
    require_index(false);
    auto crit = SuspendSection(_index_);

    if (_btd_[_index_]) {
      TransferDescriptor *p_base = _btd_[_index_];
      TransferDescriptor *curr = remove_base_td(_index_, true);
      do { curr = unlink_td(curr); } while(curr && curr != p_base);
    }
    if (desc_list && length) {
      TransferDescriptor *prev = nullptr;
      for (uint32_t i = 0; i < length; i++) {
        if (desc_list[i]) {
          TransferDescriptor *curr = desc_list[i];
          if (curr->_assig_ != -1) curr->unlink();

          if (prev) link_td(prev, curr);
          else assign_base_td(_index_, curr);

          curr->_assig_ = _index_;
          update_valid(curr->_descPtr_);
          prev = curr;
        }
      }
      if (prev && looped) link_td(prev, _btd_[_index_]);
    }   
    relink_writeback(_index_, nullptr);
    return true;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER DESCRIPTOR OBJ
///////////////////////////////////////////////////////////////////////////////////////////////////

  TransferDescriptor::TransferDescriptor() { reset(); }

  TransferDescriptor::TransferDescriptor(const TransferDescriptor &other) {
    memcpy(_descPtr_, other._descPtr_, _dsize_);
    _descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
    update_valid(_descPtr_);
  } 

  TransferDescriptor::TransferDescriptor(TransferDescriptor &&other) {
    this->operator = (std::move(other));
  }

  TransferDescriptor &TransferDescriptor::operator = (const TransferDescriptor &other) {
    if (this == &other) return *this;
    _descPtr_->BTCTRL.bit.VALID = 0U;

    auto prev_link = _descPtr_->DESCADDR.reg;
    memcpy(_descPtr_, other._descPtr_, _dsize_); 
    _descPtr_->DESCADDR.reg = prev_link;

    update_valid(_descPtr_);
    return *this;
  }
   
  TransferDescriptor &TransferDescriptor::operator = (TransferDescriptor &&other) {
    if (this == &other) return *this;
    if (_assig_ != -1) unlink();

    if (other._assig_ != -1 && _btd_[other._assig_] == &other) {
      _descPtr_ = std::exchange(other._descPtr_, &other._desc_);
      _btd_[other._assig_] = this;
    } else {
      memcpy(&_desc_, &other._desc_, _dsize_);

      if (other._assig_ != -1) {
        TransferDescriptor *prev = _btd_[other._assig_];
        while(prev->_next_ != &other) prev = prev->_next_;

        auto crit = SuspendSection(other._assig_); 
        if (_wbdesc_[other._assig_].DESCADDR.reg == (uintptr_t)&other._desc_) {
          relink_writeback(other._assig_, this);
        }
        link_td(prev, this);
      } 
    }
    _assig_ = std::exchange(other._assig_, -1);
    _next_ = std::exchange(other._next_, nullptr);
    update_valid(_descPtr_);
    return *this;
  }

  bool TransferDescriptor::set_config(const Config &cfg) {
    if ((cfg.src_inc > 1 && cfg.dst_inc > 1) || _assig_ != -1)  return false;
    
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
    if (cfg.src_inc >= 0) _descPtr_->BTCTRL.bit.SRCINC = cfg.src_inc;
    if (cfg.dst_inc >= 0) _descPtr_->BTCTRL.bit.DSTINC = cfg.dst_inc;
    if (max_inc >= 0) _descPtr_->BTCTRL.bit.STEPSIZE   = stepsize_r;
    if (max_inc >= 0) _descPtr_->BTCTRL.bit.STEPSEL    = cfg.src_inc >
      cfg.dst_inc ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;

    if (cfg.beat_size >= 0) _descPtr_->BTCTRL.bit.BEATSIZE = beatsize_r;
    if (cfg.susp_ch >= 0) _descPtr_->BTCTRL.bit.BLOCKACT = cfg.susp_ch ? 
      DMAC_BTCTRL_BLOCKACT_SUSPEND_Val : DMAC_BTCTRL_BLOCKACT_NOACT_Val;

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

  void TransferDescriptor::reset() {
    auto prev_link = _descPtr_->DESCADDR.reg;
    memset(_descPtr_, 0U, _dsize_);
    _descPtr_->DESCADDR.reg = prev_link;

    _descPtr_->BTCTRL.reg = []() consteval {
      constexpr uint32_t ss_reg = std::distance(_step_size_map_.begin(), 
        std::find(_step_size_map_.begin(),_step_size_map_.end(), 
          std::max(td_def_src_inc, td_def_dst_inc)));

      constexpr uint32_t ss_reg_f = std::max(td_def_src_inc, td_def_dst_inc) 
        > 1 ? ss_reg : DMAC_BTCTRL_STEPSIZE_X1_Val;
      static_assert(ss_reg_f < _step_size_map_.size(), "Invalid config value.");

      constexpr uint32_t bs_reg = std::distance(_beat_size_map_.begin(), 
        std::find(_beat_size_map_.begin(), _beat_size_map_.end(), 
          (td_def_beat_size)));
      static_assert(bs_reg < _beat_size_map_.size(), "Invalid config value.");

      constexpr uint32_t sinc_reg = (td_def_src_inc > td_def_dst_inc) ? 
        DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;
      constexpr uint32_t dinc_reg = (td_def_src_inc > td_def_dst_inc) 
        ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;
      constexpr uint32_t act_reg = td_def_susp_ch 
        ? DMAC_BTCTRL_BLOCKACT_SUSPEND_Val : DMAC_BTCTRL_BLOCKACT_NOACT_Val;

      return (uint32_t)
        ((ss_reg_f << DMAC_BTCTRL_STEPSIZE_Pos) |
        (dinc_reg << DMAC_BTCTRL_DSTINC_Pos) |
        (sinc_reg << DMAC_BTCTRL_SRCINC_Pos) |
        ((bool)td_def_src_inc << DMAC_BTCTRL_SRCINC_Pos) |
        ((bool)td_def_dst_inc << DMAC_BTCTRL_DSTINC_Pos) |
        (bs_reg << DMAC_BTCTRL_BEATSIZE_Pos) |
        (act_reg << DMAC_BTCTRL_BLOCKACT_Pos));
    }();
  }

  bool TransferDescriptor::set_length(const uint32_t &len, const bool &in_bytes) {
    _descPtr_->BTCTRL.bit.VALID = 0U;
    uint32_t new_cnt = len * (in_bytes ? _beat_size_map_
      [_descPtr_->BTCTRL.bit.BEATSIZE] : 1);
    if (new_cnt > max_td_length) return false; 

    uintptr_t s_addr = _descPtr_->SRCADDR.reg - addr_mod(_descPtr_, true);
    uintptr_t d_addr = _descPtr_->DSTADDR.reg - addr_mod(_descPtr_, false);

    _descPtr_->BTCNT.reg = new_cnt;
    _descPtr_->SRCADDR.reg = s_addr + addr_mod(_descPtr_, true);
    _descPtr_->DSTADDR.reg = d_addr + addr_mod(_descPtr_, false);
    update_valid(_descPtr_);
    return true;
  }

  /// @b TESTED
  uint32_t TransferDescriptor::length(const bool &in_bytes) const {
    return _descPtr_->BTCNT.reg * (in_bytes ? _beat_size_map_
      [_descPtr_->BTCTRL.bit.BEATSIZE] : 1);
  }

  bool TransferDescriptor::valid() const {
    return _descPtr_->SRCADDR.reg && _descPtr_->DSTADDR.reg
      && _descPtr_->BTCNT.reg;
  }

  /// @b WRITEBACK_FIX_V1
  void TransferDescriptor::unlink() {
    if (_assig_ == -1) return; 
    auto crit = SuspendSection(_assig_);

    if (this == _btd_[_assig_]) {
      remove_base_td(_assig_, true);

      if (_next_ && _next_ != this) {
        assign_base_td(_assig_, _next_);
      } else {
        memset(&_bdesc_[_assig_], 0U, _dsize_);
        relink_writeback(_assig_, nullptr);
      }
    } else {
      TransferDescriptor *prev = _btd_[_assig_];
      while(prev->_next_ != this) { prev = prev->_next_; }
      link_td(prev, _next_);
      
      if (_wbdesc_[_assig_].DESCADDR.reg == (uintptr_t)&_desc_) {
        relink_writeback(_assig_, _next_);
      }
    }
    _assig_ = -1;
    _next_ = nullptr;
    _descPtr_->DESCADDR.reg = 0U;
  }

  TransferDescriptor::~TransferDescriptor() {
    if (_assig_ != -1) unlink();
  }


} // END OF SIOC::DMA NAMESPACE

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: INTERRUPT HANDLER
///////////////////////////////////////////////////////////////////////////////////////////////////

  // Must be outside ns
  void DMAC_0_Handler() { 
    using namespace sioc::dma;
    using enum callback_flag_e;
    uint32_t idx = DMAC->INTPEND.bit.ID; // channel index

    if (_cbarr_[idx]) {
      // Transfer error cb -> check is fetch error status active?
      if (DMAC->INTPEND.bit.TERR) { 
        _cbarr_[idx](idx, DMAC->INTPEND.bit.FERR ? desc_err : tr_err);
      }
      // Transfer complete cb
      if (DMAC->INTPEND.bit.TCMPL) { 
        _cbarr_[idx](idx, tr_done);
      }
    }
    // Clear all interrupt flags (wont work otherwise!)
    DMAC->INTPEND.bit.TCMPL = 1U;
    DMAC->INTPEND.bit.TERR = 1U;
  } 
  // Re-route all other handlers to 0
  void DMAC_1_Handler() { DMAC_0_Handler(); }
  void DMAC_2_Handler() { DMAC_0_Handler(); }
  void DMAC_3_Handler() { DMAC_0_Handler(); }
  void DMAC_4_Handler() { DMAC_0_Handler(); }

*/