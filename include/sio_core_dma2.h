/*
///////////////////////////////////////////////////////////////////////////////////////////////////
//// FILE: DIRECT MEMORY ACCESS (DMA) 
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <sio_core_config.h>
#include <sio_temp.h>
#include <sam.h>
#include <utility>
#include <algorithm>
#include <iterator>
#include <string.h>
#include <bit>
#include <tuple>


namespace sio::core {

  struct SysModule;
  struct PrilvlModule;
  struct ChannelModule;
  struct DescriptorModule;
  struct TransferDescriptor;
  struct DmaPeripheral;

  using BM = BaseModule;
  using length_t = decltype(std::declval<DmacDescriptor>().BTCNT.reg);


  enum class INTERRUPT_FLAG {
    TRANSFER_COMPLETE,
    TRANSFER_ERROR,
    DESCRIPTOR_ERROR
  };

  enum class CHANNEL_STATE {
    DISABLED,
    IDLE,
    SUSPENDED,
    ACTIVE,
  };

  enum class TRANSFER_MODE : int {
    PACKET  = DMAC_CHCTRLA_TRIGACT_BURST_Val,
    TASK    = DMAC_CHCTRLA_TRIGACT_BLOCK_Val,
    ALL     = DMAC_CHCTRLA_TRIGACT_TRANSACTION_Val
  };

  enum class LINKED_PERIPHERAL : int {
    ADC,
    /// TBC
  };

  using ch_interrupt_t = void (*)(const int, const INTERRUPT_FLAG); 

  struct BaseModule {
    Dmac *const _reg_;

    ch_interrupt_t _cbarray_[DMAC_CH_NUM]{};
    TransferDescriptor *_btask_[DMAC_CH_NUM]{};
    DmacDescriptor _wbdesc_[DMAC_CH_NUM]{};
    DmacDescriptor _bdesc_[DMAC_CH_NUM]{};

    static constexpr int bs_ref[3]      = {1, 2, 4};  
    static constexpr int ss_ref[8]      = {1, 2, 4, 8, 16, 32, 62, 128};
    static constexpr int bus_ref[16]    = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    static constexpr int thresh_ref[4]  = {1, 2, 4, 8};
    static constexpr int chpri_ref[4]   = {1, 2, 3, 4};
    static constexpr int squal_ref[4]   = {1, 2, 3, 4};
    static constexpr int irq_pri_ref    = 1;
    static constexpr std::pair<uintptr_t, uintptr_t> ram_addr_ref[] = {
      {FLASH_ADDR, FLASH_ADDR + FLASH_SIZE}, 
      {HSRAM_ADDR, HSRAM_ADDR + HSRAM_SIZE}, 
      {BKUPRAM_ADDR, BKUPRAM_ADDR + BKUPRAM_SIZE}
    };
  }; 

  namespace {
    /// @b COMPLETE
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

    /// @b COMPLETE
    void leave_critical(const bool &crit_flag, BaseModule *base, 
      const int &index) noexcept {
      if (crit_flag) {
        base->_reg_->Channel[index].CHCTRLB.reg 
          |= DMAC_CHCTRLB_CMD_RESUME;
        base->_reg_->Channel[index].CHINTFLAG.reg
          &= ~DMAC_CHINTFLAG_SUSP;
      }
    }

    /// @b COMPLETE
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

    /// @b COMPLETE
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
  }

  struct SysModule {
    friend DmaPeripheral;
    BaseModule *const _base_;

    /// @b COMPLETE
    void initialized(const bool &init) {
      if (!(init && initialized())) {
        _base_->_reg_->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
          while(_base_->_reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
        _base_->_reg_->CTRL.reg |= DMAC_CTRL_SWRST;
          while(_base_->_reg_->CTRL.reg & DMAC_CTRL_SWRST);
        
        memset(_base_->_bdesc_, 0, sizeof(_base_->_bdesc_));
        memset(_base_->_wbdesc_, 0, sizeof(_base_->_wbdesc_));
        std::fill(std::begin(_base_->_btask_), 
          std::end(_base_->_btask_), nullptr);

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
    /// @b COMPLETE
    bool initialized() const noexcept {
      return _base_->_reg_->BASEADDR.reg == (uintptr_t)_base_->_bdesc_
        && _base_->_reg_->WRBADDR.reg == (uintptr_t)_base_->_wbdesc_
        && (MCLK->AHBMASK.reg & MCLK_AHBMASK_DMAC);
    }

    /// @b COMPLETE
    bool enabled(const bool &enabled) noexcept {
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
    /// @b COMPLETE
    inline bool enabled() const noexcept {
      return (_base_->_reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
    }

    /// @b COMPLETE
    inline int active_index() const noexcept {
      auto index = std::countr_zero(_base_->_reg_->BUSYCH.reg);
      return index >= (sizeof(_base_->_reg_->BUSYCH.reg) * 8) 
        ? -1 : index;
    }

    /// @b COMPLETE
    inline int pend_count() const noexcept {
      return std::popcount(_base_->_reg_->PENDCH.reg) 
        + std::popcount(_base_->_reg_->BUSYCH.reg);
    }
          
  };

  
  struct PrilvlModule {

    friend DmaPeripheral;
    BaseModule *const _base_;
    const int _lvl_;

    /// @b COMPLETE
    void round_robin_mode(const bool &enabled) noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
        - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
      if (enabled) {
        _base_->_reg_->PRICTRL0.reg |= (1 << shift);
      } else {
        _base_->_reg_->PRICTRL0.reg &= ~(1 << shift);
      }
    }
    /// @b COMPLETE
    inline bool round_robin_mode() const noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
        - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
      return (_base_->_reg_->PRICTRL0.reg & (1 << shift));
    }

    /// @b COMPLETE
    bool service_quality(const int &quality_lvl) noexcept {
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
    /// @b COMPLETE
    int service_quality() const noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
        - DMAC_PRICTRL0_QOS0_Pos));
      return _base_->squal_ref[_base_->_reg_->PRICTRL0.reg 
        & (DMAC_PRICTRL0_QOS0_Msk << shift)];
    }
    
    /// @b COMPLETE
    void enabled(const bool &enabled) noexcept {
      if (enabled) {
        _base_->_reg_->CTRL.reg |= (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
      } else {
        _base_->_reg_->CTRL.reg &= ~(1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
      }
    }
    /// @b COMPLETE
    inline bool enabled() const noexcept {
      return (_base_->_reg_->CTRL.reg & (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos)));
    }

  };



  /// @b COMPLETE
  struct TransferDescriptor {

    /// @b COMPLETE
    TransferDescriptor() noexcept = default;

    /// @b COMPLETE
    TransferDescriptor(const TransferDescriptor &other) noexcept {
      this->operator=(other);
    }
    /// @b COMPLETE
    TransferDescriptor(TransferDescriptor &&other) noexcept {
      this->operator=(std::move(other));
    }

    /// @b COMPLETE
    TransferDescriptor &operator = (const TransferDescriptor 
      &other) noexcept {
      if (this != &other) {
        auto link = _descPtr_->DESCADDR.reg;
        memcpy(&_cfg_, &other._cfg_, sizeof(_cfg_));
        memcpy(_descPtr_, other._descPtr_, sizeof(DmacDescriptor));
        _descPtr_->DESCADDR.reg = link;
      }
      return *this;
    }

    /// @b COMPLETE
    TransferDescriptor &operator = (TransferDescriptor &&other) noexcept {
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
          other._assig_->_base_->_btask_[other._assig_->_index_] = this;
        }
      }
      return *this;
    }

    /// @b COMPLETE
    explicit inline operator bool() noexcept {
      return _descPtr_->SRCADDR.reg != DMAC_SRCADDR_RESETVALUE
        && _descPtr_->DSTADDR.reg != DMAC_SRCADDR_RESETVALUE
        && _descPtr_->BTCNT.reg > 0;
    }

    /// @b COMPLETE
    template<typename src_t, int _incr_ = -1>
    void source(src_t *src_ptr, int increment = -1) noexcept {
      set_location<true, src_t, _incr_>(src_ptr); 
    }
    /// @b COMPLETE
    inline void *source() const noexcept {
      auto addr = _descPtr_->SRCADDR.reg - addr_mod(true);
      return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
    }

    /// @b COMPLETE
    template<typename dst_t, bool _incr_ = -1>
    void destination(dst_t *dst_ptr, int increment = -1) noexcept {
      set_location<false, dst_t, _incr_>(dst_ptr); 
    }
    /// @b COMPLETE
    inline void *destination() const noexcept {
      auto addr = _descPtr_->DSTADDR.reg - addr_mod(false);
      return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
    }

    /// @b COMPLETE
    bool length(const length_t &bytes) noexcept {
      div_t div_res = std::div(bytes, BM::bs_ref[_descPtr_->BTCTRL.reg
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
    /// @b COMPLETE
    inline length_t length() const noexcept {
      return _descPtr_->BTCNT.reg ? 0 : (_descPtr_->BTCNT.reg 
        / BM::bs_ref[_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk]);
    }
    
    /// @b COMPLETE
    inline void suspend_mode(const bool &enabled) noexcept {
      _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_BLOCKACT_Msk;
      _descPtr_->BTCTRL.reg |= enabled ? DMAC_BTCTRL_BLOCKACT_NOACT
        : DMAC_BTCTRL_BLOCKACT_SUSPEND;
    }
    /// @b COMPLETE
    inline bool suspend_mode() const noexcept {
      return (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BLOCKACT_Msk)
        == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
    }

    /// @b COMPLETE
    inline ChannelModule *assigned_channel() noexcept {
      return _assig_;
    }

    /// @b COMPLETE
    inline void unlink() noexcept {
      if (_assig_) {
        _assig_->remove(*this);
      }
    }

    /// @b COMPLETE
    ~TransferDescriptor() noexcept {
      if (_assig_) {
        _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
        TransferDescriptor *prev = nullptr;
        // Handle descriptor @ base
        if (this == _assig_->_base_->_btask_[_assig_->_index_]) { 
          if (!_next_ || _next_ == this) {
            memset(&_assig_->_base_->_wbdesc_[_assig_->_index_], 0, sizeof(DmacDescriptor));
          } else {
            memcpy(&_assig_->_base_->_bdesc_[_assig_->_index_], &_next_->_desc_,
              sizeof(DmacDescriptor));
            _next_->_descPtr_ = &_next_->_desc_;
            _assig_->_base_->_btask_[_assig_->_index_] = _next_;
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

    protected:
      friend ChannelModule;
      friend DescriptorModule;

      __PACKED_STRUCT {
        uint16_t _srcsize_;
        uint16_t _dstsize_;
        bool _srcmod_ = false;
        bool _dstmod_ = false;
      }_cfg_;
      ChannelModule *_assig_;
      TransferDescriptor *_next_ = nullptr;
      DmacDescriptor *_descPtr_ = &_desc_;
      DmacDescriptor _desc_{};

      /// @b COMPLETE
      template<bool _src_, typename loc_t, int _incr_>
      void set_location(loc_t *loc_ptr) noexcept {
        static auto src_info = std::make_tuple(DMAC_BTCTRL_SRCINC,
          DMAC_BTCTRL_STEPSEL_SRC_Val,
          &_descPtr_->SRCADDR.reg, &_cfg_._srcmod_, &_cfg_._srcsize_);
        static auto dst_info = std::make_tuple(DMAC_BTCTRL_DSTINC,
          DMAC_BTCTRL_STEPSEL_DST_Val,
          &_descPtr_->DSTADDR.reg, &_cfg_._dstmod_, &_cfg_._dstsize_);

        auto&& [inc_msk, sel_val, addr_reg, mod_var, size_var]
          = _src_ ? src_info : dst_info;
        _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
        *size_var = sizeof(loc_t);

        for (int i = std::size(BM::bs_ref) - 1; i >= 0; i--) {
          auto bs = BM::bs_ref[i];
          if (!(_cfg_._srcmod_ % bs) && !(_cfg_._dstsize_ % bs)) {
            _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_BEATSIZE_Msk;
            _descPtr_->BTCTRL.reg |= (i << DMAC_BTCTRL_BEATSIZE_Pos); 
            break;
          }
        }
        if constexpr ((_incr_ < 0 && std::is_array_v<loc_t>) 
          || _incr_ > 0) {
          _descPtr_->BTCTRL.reg |= inc_msk;
        } else {
          _descPtr_->BTCTRL.reg &= inc_msk;
        }
        if constexpr (_incr_ > 1) { 
          constexpr unsigned int step_r = std::distance(std::begin(BM::ss_ref),
            std::find(std::begin(BM::ss_ref), std::end(BM::ss_ref), 
              _incr_ * sizeof(loc_t)));
          if constexpr (step_r < std::size(BM::ss_ref)) {
            _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSEL;
            _descPtr_->BTCTRL.reg |= (sel_val << DMAC_BTCTRL_STEPSEL_Pos);

            _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
            _descPtr_->BTCTRL.reg |= (step_r << DMAC_BTCTRL_STEPSEL_Pos);
          }       
        } else if ((_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL) == sel_val) {
          _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
        }
        *mod_var = false;
        uintptr_t addr = std::bit_cast<uintptr_t>(loc_ptr);
        for (auto&& [start_addr, end_addr] : BM::ram_addr_ref) {
          if (addr >= start_addr && addr < end_addr) {
            *mod_var = true; break;
          }
        }
        *addr_reg = addr + addr_mod(_src_);
        update_valid(_descPtr_);
      }

      /// @b COMPLETE
      unsigned int addr_mod(const bool &is_src) const noexcept {
        if (is_src ? _cfg_._srcmod_ : _cfg_._dstmod_) {
          bool sel = (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL) == is_src 
            ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val; 

          return _descPtr_->BTCNT.reg * (_descPtr_->BTCTRL.bit.BEATSIZE + 1)
          * (sel ? (1 << _descPtr_->BTCTRL.bit.STEPSIZE) : 1);
        }
        return 1;
      }

  };

  /// @b COMPLETE_V2
  struct DescriptorModule {

    friend DmaPeripheral;
    friend TransferDescriptor;
    BaseModule *const _base_;
    const int _index_;
    bool _looped_ = false;


    /// @b COMPLETE_V3
    template<size_t N>
    bool set(TransferDescriptor *(&descList)[N]) noexcept {
      if (N > 0) {
        for (int i = 0; i < N; i++) {
          if (!descList[i]->_assig_) return false;
        }
        clear(true);    
        auto crit = enter_critical(_base_, _index_);
        // Set new base descriptor
        memcpy(&_base_->_bdesc_[_index_], &descList[0]->_desc_, 
          sizeof(DmacDescriptor));
        descList[0]->_descPtr_ = &_base_->_bdesc_[_index_];
        _base_->_btask_[_index_] = descList[0];

        // Link all descriptors together
        for (int i = 0; i < N - 1; i++) {
          descList[i]->_assig_ = this;
          descList[i]->_next_ = descList[i + 1];
          descList[i]->_descPtr_->DESCADDR.reg 
            = (uintptr_t)descList[i + 1]->_descPtr_;
        }
        // loop descriptors
        if (_looped_) {
          descList[N - 1]->_next_ = descList[0];
          descList[N - 1]->_descPtr_->DESCADDR.reg 
            = (uintptr_t)descList[0]->_descPtr_;
        }
        leave_critical(crit, _base_, _index_);
        return true;
      }
      return false;
    }

    /// @b COMPLETE_V2
    template<TransferDescriptor& ...desc_list>
    inline bool set(TransferDescriptor&...) noexcept {
      TransferDescriptor *desc_array[sizeof...(desc_list)] = { (&desc_list)... };
      return set(desc_array);
    }

    /// @b COMPLETE_V2
    inline bool set(TransferDescriptor& desc) noexcept {
      TransferDescriptor *desc_array[1] = { &desc };
      return set(desc_array);
    }

    /// @b COMPLETE_V3
    bool add(const int &index, TransferDescriptor &targ) noexcept {
      if (!targ._assig_) {
        TransferDescriptor *prev = nullptr;
        TransferDescriptor *targ_next = nullptr;
        // Get previous & next descriptors 
        if (index <= 0) {
          if (_looped_) prev = get_last();
          targ_next = get(index);
        } else if (index >= size()) {
          prev = get_last();
          if (_looped_) targ_next = _base_->_btask_[_index_];
        } else {
          prev = get(index - 1);
          targ_next = prev->_next_;
        }
        if (prev)  prev->_descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
        // Handle insert @ base
        if (index <= 0) {
          if (targ_next) {
            memmove(&targ_next->_desc_, _base_->_btask_[_index_], 
              sizeof(DmacDescriptor));
            targ_next->_descPtr_ = &targ_next->_desc_;
          }
          memcpy(_base_->_btask_[_index_], &targ._desc_, 
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

    /// @b COMPLETE_V2
    TransferDescriptor *get(const int &index) noexcept {
      if (index < 0) return nullptr;
      TransferDescriptor *curr = _base_->_btask_[_index_];
      for (int i = 0; i < index; i++) {
        if (!curr->_next_ || curr->_next_ == _base_->_btask_[_index_]) {
          return nullptr;
        }
        curr = curr->_next_;
      }
      return curr;
    }

    /// @b COMPLETE_V2
    TransferDescriptor &operator [] (const int &index) noexcept {
      TransferDescriptor *found = get(index);
      if (!get(index)) {
        found = _base_->_btask_[_index_]; 
      }
      return *found;
    }

    /// @b COMPLETE_V2
    TransferDescriptor *get_last() noexcept {
      TransferDescriptor *curr = _base_->_btask_[_index_];
      while(curr->_next_ && curr->_next_ != _base_->_btask_[_index_]) {
        curr = curr->_next_;
      }
      return curr;
    }

    /// @b COMPLETE_V3
    TransferDescriptor *remove(const unsigned int &index) noexcept {
      TransferDescriptor *prev = nullptr;
      TransferDescriptor *targ = nullptr;
      if (index < size() && index >= 0) {
        if (!index) {
          if (_looped_) prev = get_last();
          targ = _base_->_btask_[_index_];
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
            _base_->_btask_[_index_] = targ->_next_;
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
    /// @b COMPLETE_V2
    inline void remove(TransferDescriptor &targ) noexcept {
      remove(indexOf(targ));
    }

    /// @b COMPLETE_V2
    void clear(const bool &clear_active) noexcept {
      TransferDescriptor *curr = _base_->_btask_[_index_];
      if (curr) {
        auto crit = enter_critical(_base_, _index_);
        // Remove base
        memmove(&curr->_desc_, &_base_->_bdesc_[_index_], 
          sizeof(DmacDescriptor));
        curr->_descPtr_ = &curr->_desc_;
        _base_->_btask_[_index_] = nullptr;

        // Unlink all descriptors
        bool first_flag = true;
        while(curr && (first_flag || curr != _base_->_btask_[_index_])) {
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

    /// @b COMPLETE_V2
    size_t size() const noexcept {
      TransferDescriptor *curr = _base_->_btask_[_index_];
      size_t count = 0;
      while(curr && (!count || curr != _base_->_btask_[_index_])) {
        count++;
        curr = curr->_next_;
      }
      return count;
    }

    /// @b COMPLETE
    int indexOf(TransferDescriptor &targ) const noexcept {
      TransferDescriptor *curr = _base_->_btask_[_index_];
      int index = 0;      
      while(curr && (!index || curr != _base_->_btask_[_index_])) {
        if (curr == &targ) {
          return index;
        }
        index++;
        curr = curr->_next_;
      }
      return -1;
    }

    /// @b COMPLETE
    void looped(const bool &enabled) noexcept {
      if (enabled == _looped_) return;
      _looped_ = enabled;
      if (_base_->_btask_[_index_]) {
        TransferDescriptor *end_desc = get_last();
        if (enabled) {
          end_desc->_next_ = _base_->_btask_[_index_];
          end_desc->_descPtr_->DESCADDR.reg = (uintptr_t)end_desc;
        } else {
          end_desc->_next_ = nullptr;
          end_desc->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
        }
      }
    }
    /// @b COMPLETE
    inline bool looped() const noexcept {
      return _looped_;
    }

  };

  /// TO DO -> ADD CHECKING FOR BASE INIT
  /// @b COMPLETE
  struct ChannelModule {
    using enum CHANNEL_STATE;
    friend DmaPeripheral;
    BaseModule *const _base_;
    DescriptorModule *const _desc_;
    const int _index_;

    /// @b COMPLETE_V2
    bool state(const CHANNEL_STATE &value) noexcept {
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
    /// @b COMPLETE_V2
    CHANNEL_STATE state() const noexcept {
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

    /// @b COMPLETE_V2
    inline void reset_transfer(const bool &forced = false) noexcept {
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

    /// @b COMPLETE_V2
    void reset_channel() noexcept {
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

    /// @b COMPLETE_V2
    bool queue_transfer(const int &index, const bool &forced = false) noexcept {
      if (_base_->_btask_[_index_]) {
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

    /// @b COMPLETE_V2
    inline bool is_busy() const noexcept {
      return (_base_->_reg_->Channel[_index_].CHSTATUS.reg
        & (DMAC_CHSTATUS_PEND | DMAC_CHSTATUS_BUSY))
        && (_base_->_reg_->Channel[_index_].CHINTFLAG.reg
        & DMAC_CHINTFLAG_SUSP);
    }

    /// @b COMPLETE_V2
    inline int remaining_bytes(const bool &in_elements) const noexcept {
      unsigned int mod = in_elements ? 1 : _base_->bs_ref[_base_->_wbdesc_
        [_index_].BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk];
      return _base_->_wbdesc_[_index_].BTCNT.reg * mod;
    }

    /// @b COMPLETE_V2
    int current_index() const noexcept {
      if (!_base_->_btask_[_index_]) {
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
    
    /// @b COMPLETE_V2
    void linked_peripheral(const LINKED_PERIPHERAL &periph) noexcept {
      using enum_t = std::underlying_type_t<LINKED_PERIPHERAL>;
      _base_->_reg_->Channel[_index_].CHCTRLA.reg 
        &= ~DMAC_CHCTRLA_TRIGSRC_Msk;
      _base_->_reg_->Channel[_index_].CHCTRLA.reg 
        |= (static_cast<enum_t>(periph) << DMAC_CHCTRLA_TRIGSRC_Pos);
    }
    /// @b COMPLETE_V2
    inline LINKED_PERIPHERAL linked_peripheral() const noexcept {
      return static_cast<LINKED_PERIPHERAL>(_base_->_reg_->Channel[_index_]
        .CHCTRLA.reg & DMAC_CHCTRLA_TRIGSRC_Msk);
    }

    /// @b COMPLETE_V2
    bool packet_size(const int &elements) noexcept {       
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
    /// @b COMPLETE_V2
    inline bool packet_size(const int &) const noexcept {
      return _base_->bus_ref[_base_->_reg_->Channel[_index_].CHCTRLA.reg
        & DMAC_CHCTRLA_BURSTLEN_Msk];
    }

    /// @b COMPLETE_V2
    void transfer_mode(const TRANSFER_MODE &mode) noexcept {
      using enum_t = std::underlying_type_t<TRANSFER_MODE>;
      _base_->_reg_->Channel[_index_].CHCTRLA.reg 
        &= ~DMAC_CHCTRLA_TRIGACT_Msk;
      _base_->_reg_->Channel[_index_].CHCTRLA.reg 
        |= (static_cast<enum_t>(mode) << DMAC_CHCTRLA_TRIGACT_Pos);
    }
    /// @b COMPLETE_V2
    inline TRANSFER_MODE transfer_mode() const noexcept {
      return static_cast<TRANSFER_MODE>(_base_->_reg_->Channel[_index_]
        .CHCTRLA.reg & DMAC_CHCTRLA_TRIGACT_Msk);
    }

    /// @b COMPLETE_V2
    bool priority_lvl(const int &lvl) noexcept {
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
    /// @b COMPLETE_V2
    inline int priority_lvl() const noexcept {
      return _base_->chpri_ref[_base_->_reg_->Channel[_index_].CHPRILVL.reg
        & DMAC_CHPRILVL_PRILVL_Msk];
    }

    /// @b COMPLETE_V2
    inline void interrupt_callback(ch_interrupt_t interrupt_fn) noexcept {
      _base_->_cbarray_[_index_] = interrupt_fn;
    }

  };


  template<typename T>
  constexpr int init_seq(const bool &incr = true) {
    static int count = 0;
    if (count == std::size(T)) count = 0;
    return incr ? count++ : count;
  }

  struct DmaPeripheral {
    BaseModule base{DMAC};
    SysModule sys{&base};
    PrilvlModule prilvl[DMAC_LVL_NUM]{{&base, init_seq<decltype(prilvl)>()}};
    DescriptorModule descList[DMAC_CH_NUM]{{&base, init_seq<decltype(descList)>()}};
    ChannelModule channel[DMAC_CH_NUM]{{&base, &descList[init_seq<decltype(channel)>()], 
      init_seq<decltype(channel)>(false)}};
  }dma[DMAC_INST_NUM]{};


  static BaseModule *const irq_base = &dma_base; 
  [[interrupt("irq")]] void DMAC_0_Handler() { dma_irq_handler(*irq_base); }
  [[interrupt("irq")]] void DMAC_1_Handler() { dma_irq_handler(*irq_base); }
  [[interrupt("irq")]] void DMAC_2_Handler() { dma_irq_handler(*irq_base); }
  [[interrupt("irq")]] void DMAC_3_Handler() { dma_irq_handler(*irq_base); }
  [[interrupt("irq")]] void DMAC_4_Handler() { dma_irq_handler(*irq_base); }

}
*/
