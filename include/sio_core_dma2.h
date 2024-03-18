///////////////////////////////////////////////////////////////////////////////////////////////////
//// FILE: DIRECT MEMORY ACCESS (DMA) 
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <sio_core_config.h>
#include <sam.h>
#include <utility>
#include <algorithm>
#include <iterator>
#include <string.h>
#include <bit>
#include <type_traits>
#include <tuple>


namespace sio::core {

  class TransferDescriptor;

  enum class CALLBACK_FLAG {
    TRANSFER_COMPLETE,
    CHANNEL_ERROR
  };

  enum class CHANNEL_STATE {
    DISABLED,
    IDLE,
    SUSPENDED,
    ACTIVE,
  };

  enum class CHANNEL_ERROR {
    NONE,
    TRANSFER_ERROR,
    DESCRIPTOR_ERROR
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

  using ch_callback_t = void (*)(const int, const CALLBACK_FLAG); 

  


  struct BaseModule {
    BaseModule(Dmac *reg) : _reg_(reg) {}
    Dmac *const _reg_;
    ch_callback_t _cbarray_[DMAC_CH_NUM]{};
    TransferDescriptor *_btask_[DMAC_CH_NUM]{};
    DmacDescriptor _wbdesc_[DMAC_CH_NUM]{};
    DmacDescriptor _bdesc_[DMAC_CH_NUM]{};
    
    static constexpr int bs_ref[3]     = {1, 2, 4};  
    static constexpr int ss_ref[8]     = {1, 2, 4, 8, 16, 32, 62, 128};
    static constexpr int bus_ref[16]   = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    static constexpr int thresh_ref[4] = {1, 2, 4, 8};
    static constexpr int chpri_ref[4]  = {1, 2, 3, 4};
    static constexpr int squal_ref[4]  = {1, 2, 3, 4};
    static constexpr int crcpoly_ref[2]   = {16, 32};
    static constexpr int crc_src_shift  = 20;
    static constexpr int irq_pri_ref   = 1;
    static constexpr std::pair<uintptr_t, uintptr_t> ram_addr_ref[] = {
      {FLASH_ADDR, FLASH_ADDR + FLASH_SIZE}, 
      {HSRAM_ADDR, HSRAM_ADDR + HSRAM_SIZE}, 
      {BKUPRAM_ADDR, BKUPRAM_ADDR + BKUPRAM_SIZE}
    };
  };
  
  struct SysModule {
    BaseModule &_base_;

    /// @b COMPLETE
    void initialized(const bool &init) {
      if (!(init && initialized())) {
        _base_._reg_->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
          while(_base_._reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
        _base_._reg_->CTRL.reg |= DMAC_CTRL_SWRST;
          while(_base_._reg_->CTRL.reg & DMAC_CTRL_SWRST);
        
        memset(_base_._bdesc_, 0, sizeof(_base_._bdesc_));
        memset(_base_._wbdesc_, 0, sizeof(_base_._wbdesc_));
        std::fill(std::begin(_base_._btask_), 
          std::end(_base_._btask_), nullptr);

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
          _base_._reg_->BASEADDR.reg = (uintptr_t)_base_._bdesc_;
          _base_._reg_->WRBADDR.reg = (uintptr_t)_base_._wbdesc_;
          MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
        } else {
          MCLK->AHBMASK.reg &= ~MCLK_AHBMASK_DMAC;
        }
      }
    }
    /// @b COMPLETE
    bool initialized() const noexcept {
      return _base_._reg_->BASEADDR.reg == (uintptr_t)_base_._bdesc_
        && _base_._reg_->WRBADDR.reg == (uintptr_t)_base_._wbdesc_
        && (MCLK->AHBMASK.reg & MCLK_AHBMASK_DMAC);
    }

    /// @b COMPLETE
    bool enabled(const bool &enabled) noexcept {
        if (enabled && initialized()) {
          _base_._reg_->CTRL.reg |= DMAC_CTRL_DMAENABLE;
          return true;
        } else if (!enabled) {
          _base_._reg_->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
            while(_base_._reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
          return true;
        }
        return false;
    }
    /// @b COMPLETE
    inline bool enabled() const noexcept {
      return (_base_._reg_->CTRL.reg & DMAC_CTRL_DMAENABLE);
    }

    /// @b COMPLETE
    inline int active_index() const noexcept {
      auto index = std::countr_zero(_base_._reg_->BUSYCH.reg);
      return index >= (sizeof(_base_._reg_->BUSYCH.reg) * 8) 
        ? -1 : index;
    }

    /// @b COMPLETE
    inline int pend_count() const noexcept {
      return std::popcount(_base_._reg_->PENDCH.reg) 
        + std::popcount(_base_._reg_->BUSYCH.reg);
    }

  };

  struct DescriptorModule {
  DescriptorModule &_ch_;

    template<size_t N>
    bool set_descriptors(TransferDescriptor (*descList)[N]) {
      
    };
  

      
  };



  struct PrilvlModule {
    BaseModule &_base_;
    const int _lvl_;
  
    /// @b COMPLETE
    void round_robin_mode(const bool &enabled) noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
        - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
      if (enabled) {
        _base_._reg_->PRICTRL0.reg |= (1 << shift);
      } else {
        _base_._reg_->PRICTRL0.reg &= ~(1 << shift);
      }
    }
    /// @b COMPLETE
    inline bool round_robin_mode() const noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
        - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
      return (_base_._reg_->PRICTRL0.reg & (1 << shift));
    }

    /// @b COMPLETE
    bool service_quality(const int &quality_lvl) noexcept {
      unsigned int squal_r = std::distance(std::begin(_base_.squal_ref), 
        std::find(std::begin(_base_.squal_ref), std::end(_base_.squal_ref),
          quality_lvl));
      if (squal_r < std::size(_base_.squal_ref)) [[likely]] {
        unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
          - DMAC_PRICTRL0_QOS0_Pos));

        _base_._reg_->PRICTRL0.reg &= ~(DMAC_PRICTRL0_QOS0_Msk << shift);
        _base_._reg_->PRICTRL0.reg |= (squal_r << shift);
        return true;
      }
      return false;
    }
    /// @b COMPLETE
    int service_quality() const noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
        - DMAC_PRICTRL0_QOS0_Pos));
      return _base_.squal_ref[_base_._reg_->PRICTRL0.reg 
        & (DMAC_PRICTRL0_QOS0_Msk << shift)];
    }
    
    /// @b COMPLETE
    void enabled(const bool &enabled) noexcept {
      if (enabled) {
        _base_._reg_->CTRL.reg |= (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
      } else {
        _base_._reg_->CTRL.reg &= ~(1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
      }
    }
    /// @b COMPLETE
    inline bool enabled() const noexcept {
      return (_base_._reg_->CTRL.reg & (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos)));
    }
  };

  using BM = BaseModule;
  using length_t = decltype(std::declval<DmacDescriptor>().BTCNT.reg);

  struct TransferDescriptor {
    
    TransferDescriptor() noexcept {}

    /// @b COMPLETE
    template<typename src_t, int _incr_ = -1>
    void source(src_t *src_ptr, int increment = -1) noexcept {
      set_location<true, src_t, _incr_>(src_ptr); 
    }
    /// @b COMPLETE
    void *source() const noexcept {
      auto addr = _descPtr_->SRCADDR.reg - addr_mod(true);
      return addr == DMAC_SRCADDR_RESETVALUE ? nullptr : (void*)addr;
    }

    /// @b COMPLETE
    template<typename dst_t, bool _incr_ = -1>
    void destination(dst_t *dst_ptr, int increment = -1) noexcept {
      set_location<false, dst_t, _incr_>(dst_ptr); 
    }
    /// @b COMPLETE
    void *destination() const noexcept {
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
        _descPtr_->SRCADDR.reg = src_addr + addr_mod(true);
        _descPtr_->DSTADDR.reg = dst_addr + addr_mod(false);
        return true;
      }
      return false;
    }
    /// @b COMPLETE
    length_t length() const noexcept {
      return _descPtr_->BTCNT.reg ? 0 : (_descPtr_->BTCNT.reg 
        / BM::bs_ref[_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk]);
    }
    
    /// @b COMPLETE
    void suspend_mode(const bool &enabled) noexcept {
      _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_BLOCKACT_Msk;
      _descPtr_->BTCTRL.reg |= enabled ? DMAC_BTCTRL_BLOCKACT_NOACT
        : DMAC_BTCTRL_BLOCKACT_SUSPEND;
    }
    /// @b COMPLETE
    bool suspend_mode() const noexcept {
      return (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BLOCKACT_Msk)
        == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
    }



    private:
      __PACKED_STRUCT {
        uint16_t _srcsize_;
        uint16_t _dstsize_;
        bool _srcmod_ = false;
        bool _dstmod_ = false;
      }_cfg_;
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




  }


  struct ChannelModule {
    using enum CHANNEL_STATE;
    BaseModule &_base_;
    const int _index_;
    DescriptorModule &_desc_;

    /// @b COMPLETE
    bool state(const CHANNEL_STATE &value) noexcept {
      switch(value) {
        case DISABLED: {
          _base_._reg_->Channel[_index_].CHCTRLA.reg 
            &= ~DMAC_CHCTRLA_ENABLE;
          _base_._reg_->Channel[_index_].CHINTFLAG.reg 
            &= DMAC_CHINTFLAG_MASK;
          break;
        }
        case IDLE: {
          _base_._reg_->Channel[_index_].CHCTRLA.reg 
            &= ~DMAC_CHCTRLA_ENABLE;
          while(_base_._reg_->Channel[_index_].CHCTRLA.reg 
            & DMAC_CHCTRLA_ENABLE);
          _base_._reg_->Channel[_index_].CHCTRLA.reg 
            |= DMAC_CHCTRLA_ENABLE;
          _base_._reg_->Channel[_index_].CHINTFLAG.reg 
            &= ~DMAC_CHINTFLAG_SUSP;
          break;
        }
        case SUSPENDED: {
          _base_._reg_->Channel[_index_].CHCTRLA.reg 
            |= DMAC_CHCTRLA_ENABLE;
          _base_._reg_->Channel[_index_].CHCTRLB.reg 
            |= DMAC_CHCTRLB_CMD_SUSPEND;
          while(_base_._reg_->Channel[_index_].CHCTRLB.reg 
            & DMAC_CHCTRLB_CMD_SUSPEND);
          break;
        }
        case ACTIVE: {
          _base_._reg_->Channel[_index_].CHCTRLA.reg
            |= DMAC_CHCTRLA_ENABLE;
          if (_base_._reg_->Channel[_index_].CHINTFLAG.reg
            & DMAC_CHINTFLAG_SUSP) {
            _base_._reg_->Channel[_index_].CHCTRLB.reg
              |= DMAC_CHCTRLB_CMD_RESUME;
          }
          _base_._reg_->SWTRIGCTRL.reg 
            |= (1 << (DMAC_SWTRIGCTRL_SWTRIG0_Pos + _index_));
          break;
        }
        default: return false;
      }
      return true;
    }
    /// @b COMPLETE
    CHANNEL_STATE state() const noexcept {
      using enum CHANNEL_STATE;
      if (!(_base_._reg_->Channel[_index_].CHCTRLA.reg 
        & DMAC_CHCTRLA_ENABLE)) {
        return DISABLED;
      } else if (_base_._reg_->Channel[_index_].CHINTFLAG.reg 
        & DMAC_CHINTFLAG_SUSP) {
        return SUSPENDED;
      } else if ((_base_._reg_->Channel[_index_].CHSTATUS.reg 
        & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND))) {
        return ACTIVE;
      }
      return IDLE; 
    }

    /// @b COMPLETE
    CHANNEL_ERROR error() const noexcept {
      using enum CHANNEL_ERROR;
      if (_base_._reg_->Channel[_index_].CHINTFLAG.reg 
        & DMAC_CHINTFLAG_TERR) {
        _base_._reg_->Channel[_index_].CHINTFLAG.reg &= 
          ~DMAC_CHINTFLAG_TERR;
        return (_base_._reg_->Channel[_index_].CHSTATUS.reg
          & DMAC_CHSTATUS_FERR) ? DESCRIPTOR_ERROR : TRANSFER_ERROR;
      }
      return NONE;
    }

    /// @b COMPLETE
    inline void reset_transfer() noexcept {
      _base_._reg_->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
      memset(&_base_._wbdesc_[_index_], 0, sizeof(DmacDescriptor));
    }

    /// @b COMPLETE
    inline void reset_channel() noexcept {
      _base_._reg_->Channel[_index_].CHCTRLA.reg 
        &= ~DMAC_CHCTRLA_ENABLE;
      while(_base_._reg_->Channel[_index_].CHCTRLA.reg 
        & DMAC_CHCTRLA_ENABLE);
      _base_._reg_->Channel[_index_].CHCTRLA.reg 
        |= DMAC_CHCTRLA_SWRST;
      while(_base_._reg_->Channel[_index_].CHCTRLA.reg 
        & DMAC_CHCTRLA_SWRST);
      memset(&_base_._wbdesc_[_index_], 0, sizeof(DmacDescriptor));
      memset(&_base_._bdesc_[_index_], 0, sizeof(DmacDescriptor));
      _base_._btask_[_index_] = nullptr;
    }

    /// @b COMPLETE
    inline int remaining_elements(const bool &in_bytes) const noexcept {
      unsigned int mod = in_bytes ? _base_.bs_ref[_base_._wbdesc_
        [_index_].BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk] : 1;
      return _base_._wbdesc_[_index_].BTCNT.reg * mod;
    }

    /// @b COMPLETE
    void linked_peripheral(const LINKED_PERIPHERAL &periph) noexcept {
      using enum_t = std::underlying_type_t<LINKED_PERIPHERAL>;
      _base_._reg_->Channel[_index_].CHCTRLA.reg 
        &= ~DMAC_CHCTRLA_TRIGSRC_Msk;
      _base_._reg_->Channel[_index_].CHCTRLA.reg 
        |= (static_cast<enum_t>(periph) << DMAC_CHCTRLA_TRIGSRC_Pos);
    }
    /// @b COMPLETE
    inline LINKED_PERIPHERAL linked_peripheral() const noexcept {
      return static_cast<LINKED_PERIPHERAL>(_base_._reg_->Channel[_index_]
        .CHCTRLA.reg & DMAC_CHCTRLA_TRIGSRC_Msk);
    }

    /// @b COMPLETE
    bool packet_size(const int &elements) noexcept {       
      using reg_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
      reg_t bus_r = std::distance(std::begin(_base_.bus_ref), std::find
        (std::begin(_base_.bus_ref), std::end(_base_.bus_ref), elements));

      if (bus_r < std::size(_base_.bus_ref)) [[likely]] {
          DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_BURSTLEN_Msk;
          DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_BURSTLEN(bus_r);
          return true;
      }
      return false;
    }
    /// @b COMPLETE
    inline bool packet_size(const int &) const noexcept {
      return _base_.bus_ref[_base_._reg_->Channel[_index_].CHCTRLA.reg
        & DMAC_CHCTRLA_BURSTLEN_Msk];
    }

    /// @b COMPLETE
    void transfer_mode(const TRANSFER_MODE &mode) noexcept {
      using enum_t = std::underlying_type_t<TRANSFER_MODE>;
      _base_._reg_->Channel[_index_].CHCTRLA.reg 
        &= ~DMAC_CHCTRLA_TRIGACT_Msk;
      _base_._reg_->Channel[_index_].CHCTRLA.reg 
        |= (static_cast<enum_t>(mode) << DMAC_CHCTRLA_TRIGACT_Pos);
    }
    /// @b COMPLETE
    inline TRANSFER_MODE get_transfer_mode() const noexcept {
      return static_cast<TRANSFER_MODE>(_base_._reg_->Channel[_index_]
        .CHCTRLA.reg & DMAC_CHCTRLA_TRIGACT_Msk);
    }

    /// @b COMPLETE
    bool priority_lvl(const int &lvl) noexcept {
      int lvl_r = std::distance(std::begin(_base_.chpri_ref), std::find
        (std::begin(_base_.chpri_ref), std::end(_base_.chpri_ref), lvl));

      if (lvl_r < std::size(_base_.chpri_ref)) [[likely]] {
        _base_._reg_->Channel[_index_].CHPRILVL.reg 
          &= ~DMAC_CHPRILVL_PRILVL_Msk;
        _base_._reg_->Channel[_index_].CHPRILVL.reg 
          |= (lvl_r << DMAC_CHPRILVL_PRILVL_Pos); 
        return true;
      }
      return false;
    }
    /// @b COMPLETE
    inline int priority_lvl() const noexcept {
      return _base_.chpri_ref[_base_._reg_->Channel[_index_].CHPRILVL.reg
        & DMAC_CHPRILVL_PRILVL_Msk];
    }

    /// @b COMPLETE
    inline void irq_callback(ch_callback_t callback_fn) noexcept {
      _base_._cbarray_[_index_] = callback_fn;
    }



  };






  

}
