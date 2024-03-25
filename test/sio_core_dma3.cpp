
#include "sio_core_dma3.h"
#include <Arduino.h>

using namespace sio::core;

#define SIO_AUTO_INCR_SIZE 4

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: UTILITY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

  bool enter_critical(BaseModule *base, const unsigned int &index) {
    if ((base->_reg_->Channel[index].CHSTATUS.reg
      & (DMAC_CHSTATUS_PEND | DMAC_CHSTATUS_BUSY))
      && !base->_reg_->Channel[index].CHINTFLAG.bit.SUSP) {

      base->_reg_->Channel[index].CHCTRLB.bit.CMD
        = DMAC_CHCTRLB_CMD_SUSPEND_Val;
      while(base->_reg_->Channel[index].CHCTRLB.bit.CMD
        = DMAC_CHCTRLB_CMD_SUSPEND_Val);
      return true;
    }
    return false;
  }
  void leave_critical(const bool &crit_flag, BaseModule *base, 
    const unsigned int &index) {
    if (crit_flag) {
      base->_reg_->Channel[index].CHCTRLB.bit.CMD 
        = DMAC_CHCTRLB_CMD_RESUME_Val;
      base->_reg_->Channel[index].CHINTFLAG.bit.SUSP = 1U;
    }
  }
  
  inline bool update_valid(DmacDescriptor *desc) {
    if (desc->SRCADDR.reg && desc->DSTADDR.reg && desc->BTCNT.reg) {
      desc->BTCTRL.bit.VALID = 1U;
      return true;
    } // Else:
    desc->BTCTRL.bit.VALID = 0U;
    return false;
  }

  unsigned int addr_mod(const TransferDescriptor &desc, const bool &is_src) {
    if (is_src ? desc._cfg_._srcmod_ : desc._cfg_._dstmod_) {
      bool sel = desc._descPtr_->BTCTRL.bit.STEPSEL == is_src 
        ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val; 
      return desc._descPtr_->BTCNT.reg * (desc._descPtr_->BTCTRL.bit.BEATSIZE 
        + 1) * (sel ? (1U << desc._descPtr_->BTCTRL.bit.STEPSIZE) : 1);
    }
    return 1;
  }

  unsigned int calc_beatsize(const TransferDescriptor &td, const int &new_bytes = 0) {
    const auto bytes = new_bytes ? new_bytes : td._descPtr_->BTCNT.reg 
      * BaseModule::bs_ref[td._descPtr_->BTCTRL.bit.BEATSIZE];
    const size_t src_s = (td._cfg_._srcsize_ < bytes) * td._cfg_._srcsize_;
    const size_t dst_s = (td._cfg_._dstsize_ < bytes) * td._cfg_._dstsize_;
    for (int i = std::size(BaseModule::bs_ref) - 1; i >= 0; i--) {
      size_t bs = BaseModule::bs_ref[i];
      if (!(bytes % bs) && !(src_s % bs) && !(dst_s % bs)) {
        return i;
      }
    } // Will never reach this point
    return 0; 
  }

} // End of anon namespace

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DMA INTERRUPT HANDLER 
///////////////////////////////////////////////////////////////////////////////////////////////////

  void dma_irq_handler(volatile BaseModule *base) {
    using enum INTERRUPT_FLAG;
    unsigned int index = base->_reg_->INTPEND.bit.ID;
    INTERRUPT_FLAG flag = TRANSFER_COMPLETE;

    if (base->_reg_->INTPEND.bit.TCMPL) {
      base->_reg_->Channel[index].CHINTFLAG.bit.TCMPL = 1U;
      base->_wbdesc_[index].BTCTRL.bit.VALID = 0U;

      if (base->_wbdesc_[index].DESCADDR.reg == DMAC_SRCADDR_RESETVALUE) {
        volatile DmacDescriptor *curr = &base->_bdesc_[index];
        bool first_flag = true;
        while(curr && (first_flag || curr != &base->_bdesc_[index])) {
          first_flag = false;
          if (curr->SRCADDR.reg && curr->DSTADDR.reg && curr->BTCNT.reg) {
            curr->BTCTRL.bit.VALID = 1U;
          }      
          curr = (DmacDescriptor*)curr->DESCADDR.reg;
        }
      } // NOTE -> ERROR INTERRUPT CURRENTLY NOT FUNCTIONAL
    } else if (base->_reg_->INTPEND.bit.TERR) { 
      base->_reg_->Channel[index].CHINTFLAG.bit.TERR; 
      flag = (base->_reg_->INTPEND.bit.FERR) 
        ? DESCRIPTOR_ERROR : TRANSFER_ERROR;
    }
    if (base->_cbarray_[index]) {
      base->_cbarray_[index](index, flag);
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: SYSTEM MODULE METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////
  
  void SysModule::initialized(const bool &init) {
    if (init && initialized()) return;
    MCLK->AHBMASK.bit.DMAC_ = (unsigned int)init;
    _base_->_reg_->CTRL.bit.DMAENABLE = 0U;
      while(_base_->_reg_->CTRL.bit.DMAENABLE);
    _base_->_reg_->CTRL.bit.SWRST = 1U;
      while(_base_->_reg_->CTRL.bit.SWRST);

    memset(_base_->_bdesc_, 0U, sizeof(_base_->_bdesc_));
    memset(_base_->_wbdesc_, 0U, sizeof(_base_->_wbdesc_));
    std::fill(std::begin(_base_->_btd_), std::end(_base_->_btd_), nullptr);

    for (int i = 0; i < DMAC_LVL_NUM; i++) {
      NVIC_ClearPendingIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), 
          (1 << __NVIC_PRIO_BITS) - 1);
      if (init) {
        NVIC_EnableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
        _base_->_reg_->CTRL.reg |= (1U << (i + DMAC_CTRL_LVLEN0_Pos));
      } else {
        NVIC_DisableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
      }
    }
    if (init) {
      for (int i = 0; i < DMAC_CH_NUM; i++) {
        _base_->_reg_->Channel[i].CHINTENSET.bit.TCMPL = 1U;
        /// @todo -> Error when enabling the TERR (transfer error) irq
      }
      _base_->_reg_->BASEADDR.reg = (uintptr_t)&_base_->_bdesc_;
      _base_->_reg_->WRBADDR.reg = (uintptr_t)&_base_->_wbdesc_;
    }
  }
  
  bool SysModule::initialized() const {
    return _base_->_reg_->BASEADDR.reg == (uintptr_t)_base_->_bdesc_
      && _base_->_reg_->WRBADDR.reg == (uintptr_t)_base_->_wbdesc_
      && (MCLK->AHBMASK.bit.DMAC_);
  }

  bool SysModule::enabled(const bool &enabled) {
      if (enabled && initialized()) {
        _base_->_reg_->CTRL.bit.DMAENABLE = 1U;
        return true;
      } else if (!enabled) {
        _base_->_reg_->CTRL.bit.DMAENABLE = 0U;
          while(_base_->_reg_->CTRL.bit.DMAENABLE);
        return true;
      }
      return false;
  }
  
  bool SysModule::enabled() const {
    return (_base_->_reg_->CTRL.bit.DMAENABLE);
  }

  int SysModule::active_index() const {
    auto index = std::countr_zero(_base_->_reg_->BUSYCH.reg);
    return index >= (sizeof(_base_->_reg_->BUSYCH.reg) * 8) 
      ? -1 : index;
  }

  int SysModule::pend_count() const {
    return std::popcount(_base_->_reg_->PENDCH.reg) + 
       std::popcount(_base_->_reg_->BUSYCH.reg);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: PRIORITY LEVEL MODULE METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

  
  void PrilvlModule::round_robin_mode(const bool &enabled) {
    unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
      - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
    if (enabled) {
      _base_->_reg_->PRICTRL0.reg |= (1U << shift);
    } else {
      _base_->_reg_->PRICTRL0.reg &= ~(1U << shift);
    }
  }
  
  bool PrilvlModule::round_robin_mode() const {
    unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
      - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
    return (_base_->_reg_->PRICTRL0.reg & (1U << shift));
  }

  
  bool PrilvlModule::service_quality(const unsigned int &quality_lvl) {
    unsigned int squal_r = std::distance(std::begin(_base_->squal_ref), 
      std::find(std::begin(_base_->squal_ref), std::end(_base_->squal_ref),
        quality_lvl));
    if (squal_r < std::size(_base_->squal_ref)) [[likely]] {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos 
        - DMAC_PRICTRL0_QOS0_Pos));

      _base_->_reg_->PRICTRL0.reg &= ~(DMAC_PRICTRL0_QOS0_Msk << shift);
      _base_->_reg_->PRICTRL0.reg |= (squal_r << (shift + DMAC_PRICTRL0_QOS0_Pos));
      return true;
    }
    return false;
  }
  
  int PrilvlModule::service_quality() const {
    unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
      - DMAC_PRICTRL0_QOS0_Pos));
    return _base_->squal_ref[shift >> (_base_->_reg_->PRICTRL0.reg 
      & (DMAC_PRICTRL0_QOS0_Msk << shift))];
  }
  
  
  void PrilvlModule::enabled(const bool &enabled) {
    if (enabled) {
      _base_->_reg_->CTRL.reg |= (1U << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
    } else {
      _base_->_reg_->CTRL.reg &= ~(1U << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
    }
  }
  
  bool PrilvlModule::enabled() const {
    return (_base_->_reg_->CTRL.reg & (1U << (_lvl_ + DMAC_CTRL_LVLEN0_Pos)));
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER DESCRIPTOR METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////
  
  TransferDescriptor &TransferDescriptor::operator = 
    (const TransferDescriptor &other) {
    if (this != &other) {
      auto link = _descPtr_->DESCADDR.reg;
      memcpy(&_cfg_, &other._cfg_, sizeof(_cfg_));
      memcpy(_descPtr_, other._descPtr_, sizeof(DmacDescriptor));
      _descPtr_->DESCADDR.reg = link;
    }
    return *this;
  }

  /// @b COMPLETE_V2
  TransferDescriptor &TransferDescriptor::operator = 
    (TransferDescriptor &&other) {
    if (this != &other) {
      if (_assig_) unlink();
      other._descPtr_->BTCTRL.bit.VALID = 0U;
      if (other._descPtr_ != &other._desc_) { 
        _descPtr_ = std::exchange(other._descPtr_, &other._desc_);
        other._assig_->_base_->_btd_[other._assig_->_index_] = this;
      } else {
        memmove(_descPtr_, &other._desc_, sizeof(DmacDescriptor));
        if (other._assig_) {
          TransferDescriptor *prev = other._assig_
            ->_base_->_btd_[other._assig_->_index_];
          bool first_flag = true;

          while(prev && (first_flag || prev != other._assig_
            ->_base_->_btd_[other._assig_->_index_])) {
            first_flag = false; 
            if (prev->_next_ == &other) {
              prev->_next_ = this;
              prev->_descPtr_->DESCADDR.reg = (uintptr_t)_descPtr_;
              break;
            }
            prev = prev->_next_;
          }
        }
      }
      memmove(&_cfg_, &other._cfg_, sizeof(_cfg_));
      _assig_ = std::exchange(other._assig_, nullptr);
      _next_ = std::exchange(other._next_, nullptr);
      update_valid(_descPtr_);
    }
    return *this;
  }

  bool TransferDescriptor::valid() const {
    return update_valid(_descPtr_);
  }
  
  void *TransferDescriptor::source() const {
    auto addr = _descPtr_->SRCADDR.reg - addr_mod(*this, true);
    return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
  }
  
  void *TransferDescriptor::dest() const {
    auto addr = _descPtr_->DSTADDR.reg - addr_mod(*this, false);
    return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
  }
  
  bool TransferDescriptor::length(const length_t &bytes) {
    if (!bytes) return false;
    uintptr_t src_addr = _descPtr_->SRCADDR.reg - addr_mod(*this, true);
    uintptr_t dst_addr = _descPtr_->DSTADDR.reg - addr_mod(*this, false);

    auto beatsize_r = calc_beatsize(*this, bytes);
    _descPtr_->BTCTRL.bit.BEATSIZE = beatsize_r; 
    _descPtr_->BTCNT.reg = bytes / BaseModule::bs_ref[beatsize_r];

    _descPtr_->SRCADDR.reg = src_addr + addr_mod(*this, true);
    _descPtr_->DSTADDR.reg = dst_addr + addr_mod(*this, false);
    update_valid(_descPtr_);
    return true;
  }
  
  length_t TransferDescriptor::length() const {
    return (_descPtr_->BTCNT.bit.BTCNT * BaseModule::bs_ref
      [_descPtr_->BTCTRL.bit.BEATSIZE]);
  }
  
  void TransferDescriptor::suspend_mode(const bool &enabled) {
    _descPtr_->BTCTRL.bit.BLOCKACT = enabled ? DMAC_BTCTRL_BLOCKACT_NOACT_Val
      : DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
  }
  
  bool TransferDescriptor::suspend_mode() const {
    return (_descPtr_->BTCTRL.bit.BLOCKACT == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val);
  }

  /// @b COMPLETE_V2
  void TransferDescriptor::unlink() {
    if (_assig_) {
      TransferDescriptor *prev = _assig_->_base_->_btd_[_assig_->_index_];
      if (_descPtr_ != &_desc_) {
        memmove(&_desc_, &_assig_->_base_->_bdesc_[_assig_->_index_],
          sizeof(DmacDescriptor));
        _descPtr_ = &_desc_;

        if (_next_ && _next_ != this) {
          memcpy(&_assig_->_base_->_bdesc_[_assig_->_index_], &_next_->_desc_,
            sizeof(DmacDescriptor));
          _next_->_descPtr_ = &_assig_->_base_->_bdesc_[_assig_->_index_];
          _assig_->_base_->_btd_[_assig_->_index_] = _next_;
        } else {
          memset(&_assig_->_base_->_bdesc_[_assig_->_index_], 0U,
            sizeof(DmacDescriptor));
          _assig_->_base_->_btd_[_assig_->_index_] = nullptr;
          goto exit;
        }
      }
      while(prev->_next_ && prev->_next_ 
        != _assig_->_base_->_btd_[_assig_->_index_]) {
        if (prev->_next_ == this) break;
        prev = prev->_next_;
      }
      if (prev->_next_ == this) {
        prev->_next_ = _next_;
        prev->_descPtr_->DESCADDR.reg = _descPtr_->DESCADDR.reg;
        if (_assig_->_base_->_wbdesc_[_assig_->_index_].DESCADDR.reg 
          == (uintptr_t)_descPtr_) {
          auto crit = enter_critical(_assig_->_base_, _assig_->_index_);
          _assig_->_base_->_wbdesc_[_assig_->_index_].DESCADDR.reg = _next_ 
            ? (uintptr_t)_next_->_descPtr_ : DMAC_SRCADDR_RESETVALUE;
          leave_critical(crit, _assig_->_base_, _assig_->_index_);
        }
      }
      exit:
      _next_ = nullptr;
      _assig_ = nullptr;
      _descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
    }
  }

  void TransferDescriptor::set_location(const bool &src, void *loc_ptr, 
    const size_t &t_size, const int &increment, const bool &is_array) {

    static auto src_info = std::make_tuple(DMAC_BTCTRL_SRCINC,
      DMAC_BTCTRL_STEPSEL_SRC_Val, &_descPtr_->SRCADDR.reg, 
      &_cfg_._srcmod_, &_cfg_._srcsize_);
    static auto dst_info = std::make_tuple(DMAC_BTCTRL_DSTINC,
      DMAC_BTCTRL_STEPSEL_DST_Val, &_descPtr_->DSTADDR.reg, 
      &_cfg_._dstmod_, &_cfg_._dstsize_);
    auto&& [inc_msk, sel_val, addr_reg, mod_var, size_var]
        = src ? src_info : dst_info;

    *size_var = t_size;
    const auto bytes = _descPtr_->BTCNT.reg 
      * BaseModule::bs_ref[_descPtr_->BTCTRL.bit.BEATSIZE];
    const auto beatsize_r = calc_beatsize(*this);

    _descPtr_->BTCTRL.bit.VALID = 0U;
    _descPtr_->BTCTRL.bit.BEATSIZE = beatsize_r;
    _descPtr_->BTCNT.reg = bytes ? (bytes 
      / BaseModule::bs_ref[beatsize_r]) : 0U;
    
    if (increment && (t_size > BaseModule::bs_ref
      [std::size(BaseModule::bs_ref)] || is_array)) {
      _descPtr_->BTCTRL.reg |= inc_msk;
    } else {
     _descPtr_->BTCTRL.reg &= ~inc_msk;
    }
    if (increment > 1) {
      unsigned int step_r = std::distance(std::begin(BaseModule::ss_ref),
        std::find(std::begin(BaseModule::ss_ref), std::end(BaseModule::ss_ref),
          increment / BaseModule::bs_ref[beatsize_r]));
      if (step_r < std::size(BaseModule::ss_ref)) {
        _descPtr_->BTCTRL.bit.STEPSEL = sel_val;
        _descPtr_->BTCTRL.bit.STEPSIZE = step_r;
      }
    } else if (_descPtr_->BTCTRL.bit.STEPSEL == sel_val) {
     _descPtr_->BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;
    }
    *mod_var = false;
    uintptr_t addr = (uintptr_t)loc_ptr;
    for (auto&& [start_addr, end_addr] : BaseModule::ram_addr_ref) {
      if (addr >= start_addr && addr < end_addr) {
        *mod_var = true; break;
      }
    }
    *addr_reg = addr + addr_mod(*this, src);
    update_valid(_descPtr_);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHANNEL METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

  void ChannelModule::set_transfer(TransferDescriptor **desc_array, 
    const size_t &length, const bool &looped) {  
    bool en_flag = _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE;
    if (en_flag) {
      _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 0U;
        while(_base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE);
    }
    TransferDescriptor *curr = _base_->_btd_[_index_];
    if (curr) {
      memcpy(&curr->_desc_, &_base_->_bdesc_[_index_],
        sizeof(DmacDescriptor));
      curr->_descPtr_ = &curr->_desc_;
      _base_->_btd_[_index_] = nullptr;

      bool first_flag = true;
      while(curr && (first_flag || curr != _base_->_btd_[_index_])) {
        first_flag = false;
        curr->_assig_ = nullptr;
        curr->_next_ = nullptr;
        curr->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
      }
    }
    if (desc_array && length > 0) { 
    curr = nullptr;    
      for (int i = 0; i < length; i++) {
        if (desc_array[i]) {
          if (desc_array[i]->_assig_) {
            desc_array[i]->unlink();
          }
          if (!curr) { 
            memcpy(&_base_->_bdesc_[_index_], &desc_array[i]->_desc_, 
              sizeof(DmacDescriptor));
            desc_array[i]->_descPtr_ = &_base_->_bdesc_[_index_];
            _base_->_btd_[_index_] = desc_array[i];
          } else { 
            curr->_next_ = desc_array[i];
            curr->_descPtr_->DESCADDR.reg = (uintptr_t)desc_array[i]->_descPtr_;
          }
          desc_array[i]->_assig_ = this;
          curr = desc_array[i];
        }
      } 
      if (looped) {
        curr->_next_ = _base_->_btd_[_index_];
        curr->_descPtr_->DESCADDR.reg = (uintptr_t)&_base_->_bdesc_[_index_];
      }
    }
    memset(&_base_->_wbdesc_[_index_], 0U, sizeof(DmacDescriptor));
    if (en_flag) {
      _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 1U;
    }
  }

  bool ChannelModule::state(const CHANNEL_STATE &value) {
    if (!_base_->_reg_->CTRL.bit.DMAENABLE) return false;
    switch(value) {
      case DISABLED: {
        _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 0U;
        _base_->_reg_->Channel[_index_].CHINTFLAG.reg &= DMAC_CHINTFLAG_MASK;
        break;
      } case IDLE: {
        _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 0U;
          while(_base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE);
        _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 1U;
        _base_->_reg_->Channel[_index_].CHINTFLAG.bit.SUSP = 1U;
        break;
      } case SUSPENDED: {
        _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 1U;
        _base_->_reg_->Channel[_index_].CHCTRLB.bit.CMD 
          = DMAC_CHCTRLB_CMD_SUSPEND_Val; 
        while(!_base_->_reg_->Channel[_index_].CHINTFLAG.bit.SUSP);
        break;
      } case ACTIVE: {
        _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 1U;
        if (_base_->_reg_->Channel[_index_].CHINTFLAG.bit.SUSP) {
          _base_->_reg_->Channel[_index_].CHCTRLB.bit.CMD
            = DMAC_CHCTRLB_CMD_RESUME_Val;
          _base_->_reg_->Channel[_index_].CHINTFLAG.bit.SUSP = 1U;
        }
        _base_->_reg_->SWTRIGCTRL.reg 
          |= (1 << (DMAC_SWTRIGCTRL_SWTRIG0_Pos + _index_));
        break;
      } default: return false;
    }
    return true;
  }

  CHANNEL_STATE ChannelModule::state() const {
    using enum CHANNEL_STATE;
    if (!_base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE) {
      return DISABLED;
    } else if (_base_->_reg_->Channel[_index_].CHINTFLAG.bit.SUSP) {
      return SUSPENDED;
    } else if ((_base_->_reg_->Channel[_index_].CHSTATUS.reg 
      & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND))) {
      return ACTIVE;
    }
    return IDLE; 
  }

  void ChannelModule::reset_transfer(const bool &current_only) {
    if (_base_->_wbdesc_[_index_].SRCADDR.reg != DMAC_SRCADDR_RESETVALUE
      && _base_->_btd_[_index_]) {
      auto crit = enter_critical(_base_, _index_);
      if (current_only) {
        TransferDescriptor *curr = current_transfer();
        memcpy(&_base_->_wbdesc_[_index_], curr->_descPtr_, 
          sizeof(DmacDescriptor));
      } else {
        memcpy(&_base_->_wbdesc_[_index_], &_base_->_bdesc_[_index_], 
          sizeof(DmacDescriptor));
      }
      leave_critical(crit, _base_, _index_);
    }
  }

  void ChannelModule::reset_channel() {
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      &= ~DMAC_CHCTRLA_ENABLE;
    while(_base_->_reg_->Channel[_index_].CHCTRLA.reg 
      & DMAC_CHCTRLA_ENABLE);
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      |= DMAC_CHCTRLA_SWRST;
    while(_base_->_reg_->Channel[_index_].CHCTRLA.reg 
      & DMAC_CHCTRLA_SWRST);

    memset(&_base_->_wbdesc_[_index_], 0U, sizeof(DmacDescriptor));
    memset(&_base_->_bdesc_[_index_], 0U, sizeof(DmacDescriptor));
    set_transfer(nullptr);
  }

  /// @b FIXED
  bool ChannelModule::queue_transfer(const unsigned int &index) {
    if (_base_->_btd_[_index_]) {
      TransferDescriptor *curr = _base_->_btd_[_index_];
      for (unsigned int i = 0; i < index; i++) {
        if (!curr->_next_ || curr->_next_ == _base_->_btd_[_index_]) {
          return false;
        }
        curr = curr->_next_;
      }
      if ((_base_->_reg_->Channel[_index_].CHSTATUS.reg
        & (DMAC_CHSTATUS_PEND | DMAC_CHSTATUS_BUSY))
        && (_base_->_reg_->Channel[_index_].CHINTFLAG.reg
        & DMAC_CHINTFLAG_SUSP)) {
        _base_->_reg_->Channel[_index_].CHCTRLB.bit.CMD
          = DMAC_CHCTRLB_CMD_SUSPEND_Val;
        while(!_base_->_reg_->Channel[_index_].CHINTFLAG.bit.SUSP);
      }
      memcpy(&_base_->_wbdesc_[_index_], curr->_descPtr_, 
        sizeof(DmacDescriptor));
      return true;
    }
    return false;
  }

  bool ChannelModule::is_active() const {
    return ((_base_->_reg_->Channel[_index_].CHSTATUS.reg
      & (DMAC_CHSTATUS_PEND | DMAC_CHSTATUS_BUSY))
      && (_base_->_reg_->Channel[_index_].CHINTFLAG.reg
      & DMAC_CHINTFLAG_SUSP));
  }

  bool ChannelModule::remaining_bytes(const length_t &bytes) {
    if (bytes && _base_->_wbdesc_[_index_].SRCADDR.reg 
      != DMAC_SRCADDR_RESETVALUE) {
      auto b_div = std::div(bytes, BaseModule::bs_ref[_base_->_wbdesc_
        [_index_].BTCTRL.bit.BEATSIZE]);
      if (!b_div.rem) { 
        auto crit = enter_critical(_base_, _index_);
        _base_->_wbdesc_[_index_].BTCNT.reg 
          = static_cast<length_t>(b_div.quot);
        leave_critical(crit, _base_, _index_);
        return true;
      }
    }
    return false;
  }

  length_t ChannelModule::remaining_bytes() const {
    return _base_->_wbdesc_[_index_].BTCNT.reg * _base_->bs_ref
      [_base_->_wbdesc_[_index_].BTCTRL.bit.BEATSIZE];
  }

  TransferDescriptor *ChannelModule::current_transfer() const {
    if (_base_->_btd_[_index_]) {
      TransferDescriptor *curr = _base_->_btd_[_index_];
      bool first_flag = true;
      while(curr && (first_flag || curr != _base_->_btd_[_index_])) {
        first_flag = false;
        if (curr->_descPtr_->DESCADDR.reg 
          == _base_->_wbdesc_[_index_].DESCADDR.reg) {
          return curr;
        }
        curr = curr->_next_;
      }
    }
    return nullptr;
  }
  
  /// @note Disables DMA channel to change setting
  void ChannelModule::linked_peripheral(const LINKED_PERIPHERAL &periph) {
    bool enable_flag = false;
    if (_base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE) {
      enable_flag = true;
      _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 0U;
      while(_base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE);
    }
    using enum_t = std::underlying_type_t<LINKED_PERIPHERAL>;
    _base_->_reg_->Channel[_index_].CHCTRLA.bit.TRIGSRC 
      = static_cast<enum_t>(periph); 
    if (enable_flag) {
      _base_->_reg_->Channel[_index_].CHCTRLA.bit.ENABLE = 1U;
    }
  }

  LINKED_PERIPHERAL ChannelModule::linked_peripheral() const {
    return static_cast<LINKED_PERIPHERAL>(_base_->_reg_->Channel[_index_]
      .CHCTRLA.bit.TRIGSRC);
  }

  bool ChannelModule::packet_size(const unsigned int &elements) {       
    using reg_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
    reg_t bus_r = std::distance(std::begin(_base_->bus_ref), std::find
      (std::begin(_base_->bus_ref), std::end(_base_->bus_ref), elements));

    if (bus_r < std::size(_base_->bus_ref)) [[likely]] {
      _base_->_reg_->Channel[_index_].CHCTRLA.bit.BURSTLEN = bus_r;
      return true;
    }
    return false;
  }

  bool ChannelModule::packet_size() const {
    return _base_->bus_ref[_base_->_reg_->Channel[_index_]
      .CHCTRLA.bit.BURSTLEN];
  }

  void ChannelModule::transfer_mode(const TRANSFER_MODE &mode) {
    using enum_t = std::underlying_type_t<TRANSFER_MODE>;
    _base_->_reg_->Channel[_index_].CHCTRLA.bit.TRIGACT 
      = static_cast<enum_t>(mode);
  }

  TRANSFER_MODE ChannelModule::transfer_mode() const {
    return static_cast<TRANSFER_MODE>(_base_->_reg_->Channel[_index_]
      .CHCTRLA.bit.TRIGACT);
  }

  bool ChannelModule::priority_lvl(const unsigned int &lvl) {
    int lvl_r = std::distance(std::begin(_base_->chpri_ref), std::find
      (std::begin(_base_->chpri_ref), std::end(_base_->chpri_ref), lvl));
    if (lvl_r < std::size(_base_->chpri_ref)) [[likely]] {
      _base_->_reg_->Channel[_index_].CHPRILVL.bit.PRILVL = lvl_r;
      return true;
    }
    return false;
  }

  int ChannelModule::priority_lvl() const {
    return _base_->chpri_ref[_base_->_reg_->Channel[_index_]
      .CHPRILVL.bit.PRILVL];
  }

  void ChannelModule::enabled_durring_standby(const bool &enabled) {
    _base_->_reg_->Channel[_index_].CHCTRLA.bit.RUNSTDBY 
      = (unsigned int)enabled;
  }

  bool ChannelModule::enabled_durring_standby() const {
    return _base_->_reg_->Channel[_index_].CHCTRLA.bit.RUNSTDBY;
  }

  void ChannelModule::interrupt_callback(ch_interrupt_t interrupt_fn) {
    _base_->_cbarray_[_index_] = interrupt_fn;
  }
