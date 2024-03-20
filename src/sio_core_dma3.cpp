
#include "sio_core_dma3.h"

using namespace sio::core;

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: UTILITY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////
    
  [[no_discard]]
  bool enter_critical(BaseModule *base, const int &index) noexcept {
    if ((base->_reg_->Channel[index].CHSTATUS.reg
        & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND))
      && (base->_reg_->Channel[index].CHINTFLAG.reg
        & DMAC_CHINTFLAG_SUSP)) {

      base->_reg_->Channel[index].CHCTRLB.reg 
        |= DMAC_CHCTRLB_CMD_SUSPEND;
      while(base->_reg_->Channel[index].CHCTRLB.reg
        & DMAC_CHCTRLB_CMD_SUSPEND);
      return true;
    }
    return false;
  }

  void leave_critical(const bool &crit_flag, BaseModule *base, 
    const int &index) noexcept {
    if (crit_flag) {
      base->_reg_->Channel[index].CHCTRLB.reg 
        |= DMAC_CHCTRLB_CMD_RESUME;
      base->_reg_->Channel[index].CHINTFLAG.reg
        &= ~DMAC_CHINTFLAG_SUSP;
    }
  }
  
  bool update_valid(DmacDescriptor *desc) {
    if (desc->SRCADDR.reg != DMAC_SRCADDR_RESETVALUE
      && desc->DSTADDR.reg != DMAC_SRCADDR_RESETVALUE
      && desc->BTCNT.reg > 0) {
      desc->BTCTRL.reg |= DMAC_BTCTRL_VALID;
      return true;
    } else {
      desc->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
      return false;
    }
  }
  
  void dma_irq_handler(BaseModule *base) noexcept {
    using enum INTERRUPT_FLAG;
    unsigned int index = (base->_reg_->INTPEND.reg & DMAC_INTPEND_ID_Msk);
    INTERRUPT_FLAG flag = TRANSFER_COMPLETE;

    if (base->_reg_->INTPEND.reg & DMAC_INTPEND_TCMPL) {
      base->_reg_->Channel[index].CHINTFLAG.reg |= DMAC_CHINTFLAG_TCMPL;
      base->_wbdesc_[index].BTCTRL.reg |= DMAC_BTCTRL_VALID;

      DmacDescriptor *curr = &base->_bdesc_[index];
      bool first_flag = true;
      while(curr && (first_flag || curr != &base->_bdesc_[index])) {
        first_flag = false;
        update_valid(curr);
        curr = (DmacDescriptor*)curr->DESCADDR.reg;
      }
    } else if (base->_reg_->INTPEND.reg & DMAC_INTPEND_TERR) { 
      base->_reg_->Channel[index].CHINTFLAG.reg |= DMAC_CHINTFLAG_TERR;
      flag = (base->_reg_->INTPEND.reg & DMAC_INTPEND_FERR) 
        ? DESCRIPTOR_ERROR : TRANSFER_ERROR;
    }
    if (base->_cbarray_[index]) {
      base->_cbarray_[index](index, flag);
    }
  }

  // static BaseModule *const irq_base = &dma_base; 
  // [[interrupt("irq")]] void DMAC_0_Handler() { dma_irq_handler(*irq_base); }
  // [[interrupt("irq")]] void DMAC_1_Handler() { dma_irq_handler(*irq_base); }
  // [[interrupt("irq")]] void DMAC_2_Handler() { dma_irq_handler(*irq_base); }
  // [[interrupt("irq")]] void DMAC_3_Handler() { dma_irq_handler(*irq_base); }
  // [[interrupt("irq")]] void DMAC_4_Handler() { dma_irq_handler(*irq_base); }


///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: SYSTEM MODULE METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

  
  void SysModule::initialized(const bool &init) {
    if (!(init && initialized())) {
      _base_->_reg_->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
        while(_base_->_reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
      _base_->_reg_->CTRL.reg |= DMAC_CTRL_SWRST;
        while(_base_->_reg_->CTRL.reg & DMAC_CTRL_SWRST);
      
      memset(_base_->_bdesc_, 0, sizeof(_base_->_bdesc_));
      memset(_base_->_wbdesc_, 0, sizeof(_base_->_wbdesc_));
      std::fill(std::begin(_base_->_btd_), 
        std::end(_base_->_btd_), nullptr);

      for (int i = 0; i < DMAC_LVL_NUM; i++) {
        NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
        NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), 0);
        if (init) {
          NVIC_EnableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
        } else {
          NVIC_DisableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
        }
      }
      if (init) {
        _base_->_reg_->BASEADDR.reg = (uintptr_t)_base_->_bdesc_;
        _base_->_reg_->WRBADDR.reg = (uintptr_t)_base_->_wbdesc_;
        MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
      } else {
        MCLK->AHBMASK.reg &= ~MCLK_AHBMASK_DMAC;
      }
    }
  }
  
  bool SysModule::initialized() const noexcept {
    return _base_->_reg_->BASEADDR.reg == (uintptr_t)_base_->_bdesc_
      && _base_->_reg_->WRBADDR.reg == (uintptr_t)_base_->_wbdesc_
      && (MCLK->AHBMASK.reg & MCLK_AHBMASK_DMAC);
  }

  
  bool SysModule::enabled(const bool &enabled) noexcept {
      if (enabled && initialized()) {
        _base_->_reg_->CTRL.reg |= DMAC_CTRL_DMAENABLE;
        return true;
      } else if (!enabled) {
        _base_->_reg_->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
          while(_base_->_reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
        return true;
      }
      return false;
  }
  
  bool SysModule::enabled() const {
    return (_base_->_reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
  }

  
  inline int SysModule::active_index() const noexcept {
    auto index = std::countr_zero(_base_->_reg_->BUSYCH.reg);
    return index >= (sizeof(_base_->_reg_->BUSYCH.reg) * 8) 
      ? -1 : index;
  }

  
  inline int SysModule::pend_count() const noexcept {
    return std::popcount(_base_->_reg_->PENDCH.reg) 
      + std::popcount(_base_->_reg_->BUSYCH.reg);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: PRIORITY LEVEL MODULE METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

  
  void PrilvlModule::round_robin_mode(const bool &enabled) noexcept {
    unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
      - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
    if (enabled) {
      _base_->_reg_->PRICTRL0.reg |= (1 << shift);
    } else {
      _base_->_reg_->PRICTRL0.reg &= ~(1 << shift);
    }
  }
  
  inline bool PrilvlModule::round_robin_mode() const noexcept {
    unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
      - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
    return (_base_->_reg_->PRICTRL0.reg & (1 << shift));
  }

  
  bool PrilvlModule::service_quality(const int &quality_lvl) noexcept {
    unsigned int squal_r = std::distance(std::begin(_base_->squal_ref), 
      std::find(std::begin(_base_->squal_ref), std::end(_base_->squal_ref),
        quality_lvl));
    if (squal_r < std::size(_base_->squal_ref)) [[likely]] {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
        - DMAC_PRICTRL0_QOS0_Pos));

      _base_->_reg_->PRICTRL0.reg &= ~(DMAC_PRICTRL0_QOS0_Msk << shift);
      _base_->_reg_->PRICTRL0.reg |= (squal_r << shift);
      return true;
    }
    return false;
  }
  
  int PrilvlModule::service_quality() const noexcept {
    unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
      - DMAC_PRICTRL0_QOS0_Pos));
    return _base_->squal_ref[_base_->_reg_->PRICTRL0.reg 
      & (DMAC_PRICTRL0_QOS0_Msk << shift)];
  }
  
  
  void PrilvlModule::enabled(const bool &enabled) noexcept {
    if (enabled) {
      _base_->_reg_->CTRL.reg |= (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
    } else {
      _base_->_reg_->CTRL.reg &= ~(1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
    }
  }
  
  inline bool PrilvlModule::enabled() const noexcept {
    return (_base_->_reg_->CTRL.reg & (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos)));
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER DESCRIPTOR METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

  
  TransferDescriptor::TransferDescriptor
    (const TransferDescriptor &other) noexcept {
    this->operator=(other);
  }
  
  TransferDescriptor::TransferDescriptor
    (TransferDescriptor &&other) noexcept {
    this->operator=(std::move(other));
  }

  
  TransferDescriptor &TransferDescriptor::operator = 
    (const TransferDescriptor &other) noexcept {
    if (this != &other) {
      auto link = _descPtr_->DESCADDR.reg;
      memcpy(&_cfg_, &other._cfg_, sizeof(_cfg_));
      memcpy(_descPtr_, other._descPtr_, sizeof(DmacDescriptor));
      _descPtr_->DESCADDR.reg = link;
    }
    return *this;
  }

  TransferDescriptor &TransferDescriptor::operator = 
    (TransferDescriptor &&other) noexcept {
    if (this != &other) {
      if (_assig_) {
        this->~TransferDescriptor();
      }
      memcpy(&_cfg_, &other._cfg_, sizeof(_cfg_));
      memcpy(_descPtr_, other._descPtr_, sizeof(DmacDescriptor));
      _assig_ = std::exchange(other._assig_, nullptr);
      _next_ = std::exchange(other._next_, nullptr);
      
      if (other._descPtr_ != &other._desc_) {
        other._descPtr_ = &other._desc_;
        other._assig_->_base_->_btd_[other._assig_->_index_] = this;
      }
    }
    return *this;
  }

  inline TransferDescriptor::operator bool() noexcept {
    return _descPtr_->SRCADDR.reg != DMAC_SRCADDR_RESETVALUE
      && _descPtr_->DSTADDR.reg != DMAC_SRCADDR_RESETVALUE
      && _descPtr_->BTCNT.reg > 0;
  }
  
  inline void *TransferDescriptor::source() const noexcept {
    auto addr = _descPtr_->SRCADDR.reg - addr_mod(true);
    return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
  }
  
  inline void *TransferDescriptor::destination() const noexcept {
    auto addr = _descPtr_->DSTADDR.reg - addr_mod(false);
    return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
  }
  
  bool TransferDescriptor::length(const length_t &bytes) noexcept {
    div_t div_res = std::div(bytes, BaseModule::bs_ref[_descPtr_->BTCTRL.reg
      & DMAC_BTCTRL_BEATSIZE_Msk]);
    if (!div_res.rem) {
      uintptr_t src_addr = _descPtr_->SRCADDR.reg - addr_mod(true);
      uintptr_t dst_addr = _descPtr_->DSTADDR.reg - addr_mod(false);

      _descPtr_->BTCNT.reg = div_res.quot;
      update_valid(_descPtr_);
      _descPtr_->SRCADDR.reg = src_addr + addr_mod(true);
      _descPtr_->DSTADDR.reg = dst_addr + addr_mod(false);
      return true;
    }
    return false;
  }
  
  inline length_t TransferDescriptor::length() const noexcept {
    return _descPtr_->BTCNT.reg ? 0 : (_descPtr_->BTCNT.reg 
      / BaseModule::bs_ref[_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk]);
  }
  
  
  inline void TransferDescriptor::suspend_mode(const bool &enabled) noexcept {
    _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_BLOCKACT_Msk;
    _descPtr_->BTCTRL.reg |= enabled ? DMAC_BTCTRL_BLOCKACT_NOACT
      : DMAC_BTCTRL_BLOCKACT_SUSPEND;
  }
  
  inline bool TransferDescriptor::suspend_mode() const noexcept {
    return (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BLOCKACT_Msk)
      == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
  }

  
  inline DescriptorModule *TransferDescriptor::assigned_list() const noexcept {
    return _assig_;
  }

  
  inline void TransferDescriptor::unlink() noexcept {
    if (_assig_) {
      _assig_->remove(*this);
    }
  }

  
  TransferDescriptor::~TransferDescriptor() noexcept {
    if (_assig_) {
      _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
      TransferDescriptor *prev = nullptr;
      // Handle descriptor @ base
      if (this == _assig_->_base_->_btd_[_assig_->_index_]) { 
        if (!_next_ || _next_ == this) {
          memset(&_assig_->_base_->_wbdesc_[_assig_->_index_], 0, sizeof(DmacDescriptor));
        } else {
          memcpy(&_assig_->_base_->_bdesc_[_assig_->_index_], &_next_->_desc_,
            sizeof(DmacDescriptor));
          _next_->_descPtr_ = &_next_->_desc_;
          _assig_->_base_->_btd_[_assig_->_index_] = _next_;
        }
      } else { 
        // Handle descriptor in middle/end of list
        while(prev->_next_ != this) {
          prev = prev->_next_;
          sio_assert(prev);
        }
      }
      // Re-link previous descriptor
      if (prev) {
        prev->_next_ = _next_;
        prev->_descPtr_->DESCADDR.reg = _descPtr_->DESCADDR.reg;
      }
    }
  }

  template<bool _src_, typename loc_t, int _incr_>
  void set_location(TransferDescriptor &desc, loc_t *loc_ptr) noexcept {
    static auto src_info = std::make_tuple(DMAC_BTCTRL_SRCINC,
      DMAC_BTCTRL_STEPSEL_SRC_Val,
      &desc._descPtr_->SRCADDR.reg, &desc._cfg_._srcmod_, &desc._cfg_._srcsize_);
    static auto dst_info = std::make_tuple(DMAC_BTCTRL_DSTINC,
      DMAC_BTCTRL_STEPSEL_DST_Val,
      &desc._descPtr_->DSTADDR.reg, &desc._cfg_._dstmod_, &desc._cfg_._dstsize_);

    auto&& [inc_msk, sel_val, addr_reg, mod_var, size_var]
      = _src_ ? src_info : dst_info;
    desc._descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
    *size_var = sizeof(loc_t);

    for (int i = std::size(BaseModule::bs_ref) - 1; i >= 0; i--) {
      auto bs = BaseModule::bs_ref[i];
      if (!(desc._cfg_._srcmod_ % bs) && !(desc._cfg_._dstsize_ % bs)) {
        desc._descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_BEATSIZE_Msk;
        desc._descPtr_->BTCTRL.reg |= (i << DMAC_BTCTRL_BEATSIZE_Pos); 
        break;
      }
    }
    if constexpr ((_incr_ < 0 && std::is_array_v<loc_t>) 
      || _incr_ > 0) {
      desc._descPtr_->BTCTRL.reg |= inc_msk;
    } else {
      desc._descPtr_->BTCTRL.reg &= inc_msk;
    }
    if constexpr (_incr_ > 1) { 
      constexpr unsigned int step_r = std::distance(std::begin(BaseModule::ss_ref),
        std::find(std::begin(BaseModule::ss_ref), std::end(BaseModule::ss_ref),
          _incr_ * sizeof(loc_t)));
      if constexpr (step_r < std::size(BaseModule::ss_ref)) {
        desc._descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSEL;
        desc._descPtr_->BTCTRL.reg |= (sel_val << DMAC_BTCTRL_STEPSEL_Pos);

        desc._descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
        desc._descPtr_->BTCTRL.reg |= (step_r << DMAC_BTCTRL_STEPSEL_Pos);
      }       
    } else if ((desc._descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL) == sel_val) {
      desc._descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
    }
    *mod_var = false;
    uintptr_t addr = std::bit_cast<uintptr_t>(loc_ptr);
    for (auto&& [start_addr, end_addr] : BaseModule::ram_addr_ref) {
      if (addr >= start_addr && addr < end_addr) {
        *mod_var = true; break;
      }
    }
    *addr_reg = addr + desc.addr_mod(_src_);
    ::update_valid(desc._descPtr_);
  }

  
  unsigned int TransferDescriptor::addr_mod(const bool &is_src) const noexcept {
    if (is_src ? _cfg_._srcmod_ : _cfg_._dstmod_) {
      bool sel = (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL) == is_src 
        ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val; 

      return _descPtr_->BTCNT.reg * (_descPtr_->BTCTRL.bit.BEATSIZE + 1)
      * (sel ? (1 << _descPtr_->BTCTRL.bit.STEPSIZE) : 1);
    }
    return 1;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DESCRIPTOR LIST METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

    bool DescriptorModule::set(TransferDescriptor **desc_list,
      const size_t &length) noexcept {
      if (length > 0) {
        for (int i = 0; i < length; i++) {
          if (!desc_list[i]->_assig_) return false;
        }
        clear(true);    
        auto crit = enter_critical(_base_, _index_);
        // Set new base descriptor
        memcpy(&_base_->_bdesc_[_index_], &desc_list[0]->_desc_, 
          sizeof(DmacDescriptor));
        desc_list[0]->_descPtr_ = &_base_->_bdesc_[_index_];
        _base_->_btd_[_index_] = desc_list[0];

        // Link all descriptors together
        for (int i = 0; i < length - 1; i++) {
          desc_list[i]->_assig_ = this;
          desc_list[i]->_next_ = desc_list[i + 1];
          desc_list[i]->_descPtr_->DESCADDR.reg 
            = (uintptr_t)desc_list[i + 1]->_descPtr_;
        }
        // loop descriptors
        if (_looped_) {
          desc_list[length - 1]->_next_ = desc_list[0];
          desc_list[length - 1]->_descPtr_->DESCADDR.reg 
            = (uintptr_t)desc_list[0]->_descPtr_;
        }
        leave_critical(crit, _base_, _index_);
        return true;
      }
      return false;
    }

  inline bool DescriptorModule::set(TransferDescriptor& desc) noexcept {
    TransferDescriptor *desc_array[1] = { &desc };
    return set(desc_array, 1);
  }

  bool DescriptorModule::add(const int &index, 
    TransferDescriptor &targ) noexcept {
    if (!targ._assig_) {
      TransferDescriptor *prev = nullptr;
      TransferDescriptor *targ_next = nullptr;
      // Get previous & next descriptors 
      if (index <= 0) {
        if (_looped_) prev = get_last();
        targ_next = get(index);
      } else if (index >= size()) {
        prev = get_last();
        if (_looped_) targ_next = _base_->_btd_[_index_];
      } else {
        prev = get(index - 1);
        targ_next = prev->_next_;
      }
      if (prev)  prev->_descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
      // Handle insert @ base
      if (index <= 0) {
        if (targ_next) {
          memmove(&targ_next->_desc_, _base_->_btd_[_index_], 
            sizeof(DmacDescriptor));
          targ_next->_descPtr_ = &targ_next->_desc_;
        }
        memcpy(_base_->_btd_[_index_], &targ._desc_, 
          sizeof(DmacDescriptor));
        targ._descPtr_ = &_base_->_bdesc_[_index_];
      }
      // Update targ & previous
      targ._assig_ = this;
      targ._next_ = targ_next;
      targ._descPtr_->DESCADDR.reg = targ_next 
        ? (uintptr_t)targ_next->_descPtr_ : DMAC_SRCADDR_RESETVALUE;
      if (prev) {
        prev->_next_ = &targ;
        prev->_descPtr_->DESCADDR.reg = (uintptr_t)targ._descPtr_;
        update_valid(prev->_descPtr_);
      }
      // Update writeback
      if (_base_->_wbdesc_[_index_].DESCADDR.reg 
          == (uintptr_t)targ_next->_descPtr_) {
        auto crit = enter_critical(_base_, _index_);
        _base_->_wbdesc_[_index_].DESCADDR.reg = (uintptr_t)targ._descPtr_;
        leave_critical(crit, _base_, _index_);
      }
      return true;
    } 
    return false;
  }

  TransferDescriptor *DescriptorModule::get(const int &index) noexcept {
    if (index < 0) return nullptr;
    TransferDescriptor *curr = _base_->_btd_[_index_];
    for (int i = 0; i < index; i++) {
      if (!curr->_next_ || curr->_next_ == _base_->_btd_[_index_]) {
        return nullptr;
      }
      curr = curr->_next_;
    }
    return curr;
  }

  TransferDescriptor &DescriptorModule::operator [] (const int &index) 
    noexcept {
    TransferDescriptor *found = get(index);
    if (!get(index)) {
      found = _base_->_btd_[_index_]; 
    }
    return *found;
  }

  TransferDescriptor *DescriptorModule::get_last() noexcept {
    TransferDescriptor *curr = _base_->_btd_[_index_];
    while(curr->_next_ && curr->_next_ != _base_->_btd_[_index_]) {
      curr = curr->_next_;
    }
    return curr;
  }


  TransferDescriptor *DescriptorModule::remove(const unsigned int &index) 
    noexcept {
    TransferDescriptor *prev = nullptr;
    TransferDescriptor *targ = nullptr;
    if (index < size() && index >= 0) {
      if (!index) {
        if (_looped_) prev = get_last();
        targ = _base_->_btd_[_index_];
      } else if (index) {
        prev = get(index - 1);
        targ = prev->_next_;
      }
      if (targ) {
        if (prev) prev->_descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
        // Handle target @ base
        if (!index) {
            memmove(&targ->_desc_, &_base_->_bdesc_[_index_], 
              sizeof(DmacDescriptor));
            targ->_descPtr_ = &targ->_desc_;
          if (targ->_next_) {
            memcpy(&_base_->_bdesc_[_index_], targ->_next_->_descPtr_, 
              sizeof(DmacDescriptor));
            targ->_next_->_descPtr_ = &_base_->_bdesc_[_index_];
          } else {
            memset(&_base_->_bdesc_[_index_], 0, sizeof(DmacDescriptor));
          }
          _base_->_btd_[_index_] = targ->_next_;
        }
        // Re-link previous descriptor
        if (prev) {
          if (_base_->_wbdesc_[_index_].DESCADDR.reg 
            == prev->_descPtr_->DESCADDR.reg) {
            auto crit = enter_critical(_base_, _index_);
            _base_->_wbdesc_[_index_].DESCADDR.reg 
              = targ->_descPtr_->DESCADDR.reg;
            leave_critical(crit, _base_, _index_);
          }
          prev->_next_ = targ->_next_;
          prev->_descPtr_->DESCADDR.reg = targ->_descPtr_->DESCADDR.reg;
        }
        targ->_assig_ = nullptr;
        targ->_next_ = nullptr;
        targ->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
        update_valid(prev->_descPtr_);
      }
    }
    return targ;
  }

  inline void DescriptorModule::remove(TransferDescriptor &targ) noexcept {
    remove(indexOf(targ));
  }

  void DescriptorModule::clear(const bool &clear_active) noexcept {
    TransferDescriptor *curr = _base_->_btd_[_index_];
    if (curr) {
      auto crit = enter_critical(_base_, _index_);
      // Remove base
      memmove(&curr->_desc_, &_base_->_bdesc_[_index_], 
        sizeof(DmacDescriptor));
      curr->_descPtr_ = &curr->_desc_;
      _base_->_btd_[_index_] = nullptr;

      // Unlink all descriptors
      bool first_flag = true;
      while(curr && (first_flag || curr != _base_->_btd_[_index_])) {
        first_flag = false;
        curr->_assig_ = nullptr;
        curr->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
        curr = std::exchange(curr->_next_, nullptr);
      }
      // Update writeback descriptor link
      if (clear_active) {
        memset(&_base_->_wbdesc_[_index_], 0, sizeof(DmacDescriptor));
        _base_->_reg_->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
      } else {
        leave_critical(crit, _base_, _index_);
        _base_->_wbdesc_[_index_].DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
      }
    }
  }

  size_t DescriptorModule::size() const noexcept {
    TransferDescriptor *curr = _base_->_btd_[_index_];
    size_t count = 0;
    while(curr && (!count || curr != _base_->_btd_[_index_])) {
      count++;
      curr = curr->_next_;
    }
    return count;
  }

  int DescriptorModule::indexOf(TransferDescriptor &targ) const noexcept {
    TransferDescriptor *curr = _base_->_btd_[_index_];
    int index = 0;      
    while(curr && (!index || curr != _base_->_btd_[_index_])) {
      if (curr == &targ) {
        return index;
      }
      index++;
      curr = curr->_next_;
    }
    return -1;
  }
  
  void DescriptorModule::looped(const bool &enabled) noexcept {
    if (enabled == _looped_) return;
    _looped_ = enabled;
    if (_base_->_btd_[_index_]) {
      TransferDescriptor *end_desc = get_last();
      if (enabled) {
        end_desc->_next_ = _base_->_btd_[_index_];
        end_desc->_descPtr_->DESCADDR.reg = (uintptr_t)end_desc;
      } else {
        end_desc->_next_ = nullptr;
        end_desc->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
      }
    }
  }

  
  inline bool DescriptorModule::looped() const noexcept {
    return _looped_;
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHANNEL METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

  bool ChannelModule::state(const CHANNEL_STATE &value) noexcept {
    if (_base_->_reg_->Channel[_index_].CHINTENSET.reg < 2) {
      _base_->_reg_->Channel[_index_].CHINTENSET.reg 
        |= (DMAC_CHINTENSET_TCMPL | DMAC_CHINTENSET_TERR);
    }
    switch(value) {
      case DISABLED: {
        _base_->_reg_->Channel[_index_].CHCTRLA.reg 
          &= ~DMAC_CHCTRLA_ENABLE;
        _base_->_reg_->Channel[_index_].CHINTFLAG.reg 
          &= DMAC_CHINTFLAG_MASK;
        break;
      }
      case IDLE: {
        _base_->_reg_->Channel[_index_].CHCTRLA.reg 
          &= ~DMAC_CHCTRLA_ENABLE;
        while(_base_->_reg_->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_ENABLE);
        _base_->_reg_->Channel[_index_].CHCTRLA.reg 
          |= DMAC_CHCTRLA_ENABLE;
        _base_->_reg_->Channel[_index_].CHINTFLAG.reg 
          &= ~DMAC_CHINTFLAG_SUSP;
        break;
      }
      case SUSPENDED: {
        _base_->_reg_->Channel[_index_].CHCTRLA.reg 
          |= DMAC_CHCTRLA_ENABLE;
        _base_->_reg_->Channel[_index_].CHCTRLB.reg 
          |= DMAC_CHCTRLB_CMD_SUSPEND;
        while(_base_->_reg_->Channel[_index_].CHCTRLB.reg 
          & DMAC_CHCTRLB_CMD_SUSPEND);
        break;
      }
      case ACTIVE: {
        _base_->_reg_->Channel[_index_].CHCTRLA.reg
          |= DMAC_CHCTRLA_ENABLE;
        if (_base_->_reg_->Channel[_index_].CHINTFLAG.reg
          & DMAC_CHINTFLAG_SUSP) {

          _base_->_reg_->Channel[_index_].CHCTRLB.reg
            |= DMAC_CHCTRLB_CMD_RESUME;
          _base_->_reg_->Channel[_index_].CHINTFLAG.reg
            &= ~DMAC_CHINTFLAG_SUSP;
        }
        _base_->_reg_->SWTRIGCTRL.reg 
          |= (1 << (DMAC_SWTRIGCTRL_SWTRIG0_Pos + _index_));
        break;
      }
      default: return false;
    }
    return true;
  }

  CHANNEL_STATE ChannelModule::state() const noexcept {
    using enum CHANNEL_STATE;
    if (!(_base_->_reg_->Channel[_index_].CHCTRLA.reg 
      & DMAC_CHCTRLA_ENABLE)) {
      return DISABLED;
    } else if (_base_->_reg_->Channel[_index_].CHINTFLAG.reg 
      & DMAC_CHINTFLAG_SUSP) {
      return SUSPENDED;
    } else if ((_base_->_reg_->Channel[_index_].CHSTATUS.reg 
      & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND))) {
      return ACTIVE;
    }
    return IDLE; 
  }

  inline void ChannelModule::reset_transfer(const bool &forced) noexcept {
    if (forced) {
      _base_->_reg_->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
      memset(&_base_->_wbdesc_[_index_], 0, sizeof(DmacDescriptor));
    } else {
      auto crit = enter_critical(_base_, _index_);
      _base_->_wbdesc_[_index_].DESCADDR.reg 
        = (uintptr_t)&_base_->_bdesc_[_index_];
      leave_critical(crit, _base_, _index_);
    }
  }

  void ChannelModule::reset_channel() noexcept {
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      &= ~DMAC_CHCTRLA_ENABLE;
    while(_base_->_reg_->Channel[_index_].CHCTRLA.reg 
      & DMAC_CHCTRLA_ENABLE);
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      |= DMAC_CHCTRLA_SWRST;
    while(_base_->_reg_->Channel[_index_].CHCTRLA.reg 
      & DMAC_CHCTRLA_SWRST);

    memset(&_base_->_wbdesc_[_index_], 0, sizeof(DmacDescriptor));
    memset(&_base_->_bdesc_[_index_], 0, sizeof(DmacDescriptor));
    _desc_->clear(true);
  }

  bool ChannelModule::queue_transfer(const int &index, const bool &forced) noexcept {
    if (_base_->_btd_[_index_]) {
      DmacDescriptor *curr = &_base_->_bdesc_[_index_];
      uintptr_t base_addr = (uintptr_t)curr;
      for (int i = 0; i < index; i++) {
        if (curr->DESCADDR.reg == DMAC_SRCADDR_RESETVALUE
          || curr->DESCADDR.reg == base_addr) {
          return false;
        }
        curr = (DmacDescriptor*)curr->DESCADDR.reg;
      }
      auto crit = enter_critical(_base_, _index_);
      if (forced) {
        memcpy(&_base_->_wbdesc_[_index_], curr, sizeof(DmacDescriptor));
      } else {
        _base_->_wbdesc_[_index_].DESCADDR.reg = (uintptr_t)curr;
      }
      leave_critical(crit, _base_, _index_);
      return true;
    }
    return false;
  }

  inline bool ChannelModule::is_busy() const noexcept {
    return (_base_->_reg_->Channel[_index_].CHSTATUS.reg
      & (DMAC_CHSTATUS_PEND | DMAC_CHSTATUS_BUSY))
      && (_base_->_reg_->Channel[_index_].CHINTFLAG.reg
      & DMAC_CHINTFLAG_SUSP);
  }

  inline int ChannelModule::remaining_bytes(const bool &in_elements) const noexcept {
    unsigned int mod = in_elements ? 1 : _base_->bs_ref[_base_->_wbdesc_
      [_index_].BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk];
    return _base_->_wbdesc_[_index_].BTCNT.reg * mod;
  }

  int ChannelModule::current_index() const noexcept {
    if (!_base_->_btd_[_index_]) {
      DmacDescriptor *curr = &_base_->_bdesc_[_index_];
      DmacDescriptor *targ = &_base_->_wbdesc_[_index_];
      int count = 0;
      while(curr != targ) {
        sio_assert(curr);
        count++;
        curr = (DmacDescriptor*)curr->DESCADDR.reg;
      }
      return count;
    }
    return -1;
  }
  
  void ChannelModule::linked_peripheral(const LINKED_PERIPHERAL &periph) 
    noexcept {
    using enum_t = std::underlying_type_t<LINKED_PERIPHERAL>;
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      &= ~DMAC_CHCTRLA_TRIGSRC_Msk;
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      |= (static_cast<enum_t>(periph) << DMAC_CHCTRLA_TRIGSRC_Pos);
  }

  inline LINKED_PERIPHERAL ChannelModule::linked_peripheral() const noexcept {
    return static_cast<LINKED_PERIPHERAL>(_base_->_reg_->Channel[_index_]
      .CHCTRLA.reg & DMAC_CHCTRLA_TRIGSRC_Msk);
  }

  bool ChannelModule::packet_size(const int &elements) noexcept {       
    using reg_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
    reg_t bus_r = std::distance(std::begin(_base_->bus_ref), std::find
      (std::begin(_base_->bus_ref), std::end(_base_->bus_ref), elements));

    if (bus_r < std::size(_base_->bus_ref)) [[likely]] {
        DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_BURSTLEN_Msk;
        DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_BURSTLEN(bus_r);
        return true;
    }
    return false;
  }

  inline bool ChannelModule::packet_size(const int &) const noexcept {
    return _base_->bus_ref[_base_->_reg_->Channel[_index_].CHCTRLA.reg
      & DMAC_CHCTRLA_BURSTLEN_Msk];
  }

  void ChannelModule::transfer_mode(const TRANSFER_MODE &mode) noexcept {
    using enum_t = std::underlying_type_t<TRANSFER_MODE>;
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      &= ~DMAC_CHCTRLA_TRIGACT_Msk;
    _base_->_reg_->Channel[_index_].CHCTRLA.reg 
      |= (static_cast<enum_t>(mode) << DMAC_CHCTRLA_TRIGACT_Pos);
  }

  inline TRANSFER_MODE ChannelModule::transfer_mode() const noexcept {
    return static_cast<TRANSFER_MODE>(_base_->_reg_->Channel[_index_]
      .CHCTRLA.reg & DMAC_CHCTRLA_TRIGACT_Msk);
  }

  bool ChannelModule::priority_lvl(const int &lvl) noexcept {
    int lvl_r = std::distance(std::begin(_base_->chpri_ref), std::find
      (std::begin(_base_->chpri_ref), std::end(_base_->chpri_ref), lvl));

    if (lvl_r < std::size(_base_->chpri_ref)) [[likely]] {
      _base_->_reg_->Channel[_index_].CHPRILVL.reg 
        &= ~DMAC_CHPRILVL_PRILVL_Msk;
      _base_->_reg_->Channel[_index_].CHPRILVL.reg 
        |= (lvl_r << DMAC_CHPRILVL_PRILVL_Pos); 
      return true;
    }
    return false;
  }

  inline int ChannelModule::priority_lvl() const noexcept {
    return _base_->chpri_ref[_base_->_reg_->Channel[_index_].CHPRILVL.reg
      & DMAC_CHPRILVL_PRILVL_Msk];
  }

  inline void ChannelModule::interrupt_callback(ch_interrupt_t interrupt_fn) noexcept {
    _base_->_cbarray_[_index_] = interrupt_fn;
  }
