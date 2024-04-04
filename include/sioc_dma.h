
#pragma once
#include <sioc_dma_defs.h>
#include <sioc_dma_ref.h>
#include <sam.h>
#include <array>
#include <iterator>
#include <algorithm>
#include <utility>
#include <tuple>
#include <bit>
#include <span>

#define DD_ATTR SECTION_DMAC_DESCRIPTOR __ALIGNED(16)

namespace sioc::dma {

  struct Channel;
  struct TransferDescriptor;

  /// @b TO_REMOVE
  constexpr uint32_t inst = 0; 

  namespace detail {

    template<uint32_t inst_num>
    struct InstCommon {
      static inline uint32_t ch_msk;
      static inline DmacDescriptor bdesc[ref::max_ch]{};            // Base transfer descriptor
      volatile static inline DmacDescriptor wbdesc[ref::max_ch]{};  // Writeback transfer descriptors
      static inline TransferDescriptor *btd[ref::max_ch]{nullptr};  // Base transfer descriptor objs
      volatile static inline bool suspf[ref::max_ch]{false};        // Channel suspend flags
      static inline callback_t cbarr[ref::max_ch]{nullptr};         // Channel callback func ptrs
    };

    inline struct {}dummy_loc;
    inline void dummy_cb(const uint32_t &a, const callback_flag_e &b) { }
    inline constexpr size_t _dsize_ = sizeof(DmacDescriptor);


    [[__always_inline__]]
    inline void update_valid(DmacDescriptor &desc) {
      desc.BTCTRL.bit.VALID = (desc.SRCADDR.reg && desc.DSTADDR.reg && desc.BTCNT.reg);
    }


    // -> Ref datasheet 22.2.6.7  
    inline uintptr_t addr_mod(const DmacDescriptor &desc, const bool &is_src) {
      uint32_t mod = 0;
      if (is_src ? desc.BTCTRL.bit.SRCINC : desc.BTCTRL.bit.DSTINC) {
        mod = desc.BTCNT.reg * ref::beatsize_map[desc.BTCTRL.bit.BEATSIZE];       
        uint32_t sel = is_src ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;

        if (desc.BTCTRL.bit.STEPSEL == sel) {
          mod *= (1 << desc.BTCTRL.bit.STEPSIZE);
        }
      }
      return mod;
    }


    /// @b TODO
    struct SuspendSection 
    { 
      using comm = InstCommon<inst>;
      SuspendSection(const uint32_t &ch_id) : id(ch_id) {
        if (id >= 0 && id < ref::max_ch && DMAC->Channel[id].CHCTRLA.bit.ENABLE 
          && !comm::suspf[id] && !DMAC->Channel[id].CHINTFLAG.bit.SUSP) {
          susp_flag = true;
          comm::suspf[id] = true;
          DMAC->Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
          while(DMAC->Channel[id].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
        }
      }
      ~SuspendSection() {
        if (susp_flag && DMAC->Channel[id].CHCTRLA.bit.ENABLE) {
          comm::suspf[id] = false;
          DMAC->Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
        }
      }
      const uint32_t id;      // Channel id
      bool susp_flag = false; // True = not initially suspended
    };


  } // END OF ANON NAMESPACE 


struct TransferDescriptor : private detail::InstCommon<inst>
  {

    TransferDescriptor() = default; 
    
    TransferDescriptor(const TransferDescriptor &other) {
      this->operator=(other);
    }

    TransferDescriptor &operator = (const TransferDescriptor &other) {
      if (&other != this) {
        auto l_addr = _descPtr_->DESCADDR.reg;
        memcpy(_descPtr_, other._descPtr_, detail::_dsize_);
        _descPtr_->DESCADDR.reg = l_addr;
      }
      return *this;
    }

    struct Config {
      uint32_t beatsize   = -1; // > numeric
      void *src           = &detail::dummy_loc;
      int32_t srcinc      = -1; // > numeric
      void *dst           = &detail::dummy_loc;
      int32_t dstinc      = -1; // > numeric
      blockact_t blockact = blockact_t::null;
    };

    template<Config cfg>
    void set_config() {
      using namespace ref;
      auto prev_bs = beatsize_map[_descPtr_->BTCTRL.bit.BEATSIZE];
      uintptr_t prev_srcaddr, prev_dstaddr;

      // Capture previous, non-modified address if beatsize changed
      if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        if constexpr (cfg.src != &detail::dummy_loc) {
          prev_srcaddr = _descPtr_->SRCADDR.reg - detail::addr_mod(_descPtr_, true);
        }
        if constexpr (cfg.dst != &detail::dummy_loc && ) {
          prev_dstaddr = _descPtr_->DSTADDR.reg - detail::addr_mod(_descPtr_, false);
        }
      }
      // Compute ctrl register mask
      constexpr auto btctrl_masks = [&]() consteval {
        using ctrl_t = decltype(std::declval<DMAC_BTCTRL_Type>().reg);        
        ctrl_t clr_msk{0}, set_msk{0};

        if (cfg.beatsize != -1) {
          constexpr uint32_t bs_reg = std::distance(beatsize_map.begin(), 
              std::find(beatsize_map.begin(), beatsize_map.end(), cfg.beatsize));  
          static_assert(bs_reg < beatsize_map.size(), "SIO ERROR: DMA descriptor config");
          clr_msk |= (DMAC_BTCTRL_BEATSIZE_Msk);
          set_msk |= (bs_reg << DMAC_BTCTRL_BEATSIZE_Pos);
        }        
        if (std::max(cfg.srcinc, cfg.dstinc) > 1) {
          constexpr uint32_t ssize_r = std::distance(stepsize_map.begin(), std::find
          (stepsize_map.begin(), stepsize_map.end(), std::max(cfg.srcinc, cfg.dstinc)));
          static_assert(ssize_r < stepsize_map.size(), "SIO ERROR: DMA descriptor config");
          clr_msk |= (DMAC_BTCTRL_STEPSIZE_Msk);
          set_msk |= (ssize_r << DMAC_BTCTRL_STEPSIZE_Pos);
        }
        if (cfg.srcinc != -1) {
          clr_msk |= (DMAC_BTCTRL_SRCINC);
          set_msk |= (static_cast<bool>(cfg.srcinc) << DMAC_BTCTRL_SRCINC_Pos);
        }
        if (cfg.dstinc != -1) {
          clr_msk |= (DMAC_BTCTRL_DSTINC);
          set_msk |= (static_cast<bool>(cfg.dstinc) << DMAC_BTCTRL_DSTINC_Pos);
        }
        if (cfg.srcinc > 1) {
          static_assert(cfg.dstinc <= 1, "SIO ERROR: DMA descriptor config");
          clr_msk |= (DMAC_BTCTRL_STEPSEL);
          set_msk |= (DMAC_BTCTRL_STEPSEL_SRC);
        }
        if (cfg.dstinc > 1) {
          static_assert(cfg.srcinc <= 1, "SIO ERROR: DMA descriptor config");
          clr_msk |= (DMAC_BTCTRL_STEPSEL);
          set_msk |= (DMAC_BTCTRL_STEPSEL_DST);
        }
        return std::pair(clr_msk, set_msk);      
      }();
      // Clear and set specified config registers
      _descPtr_->BTCTRL.reg &= ~btctrl_masks.first; 
      _descPtr_->BTCTRL.reg |= btctrl_masks.second;

      // Set source address or re-calc previous if beatsize changed
      if constexpr (cfg.src != &detail::dummy_loc) {
        _descPtr_->SRCADDR.reg = cfg.src ? static_cast<uintptr_t>(cfg.src) 
          + detail::addr_mod(_descPtr_, true) : 0;
      } else if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        _descPtr_->SRCADDR.reg = prev_srcaddr + detail::addr_mod(_descPtr_, true);
      }
      // Set destination address or re-calc prev if beatsize changed
      if constexpr (cfg.dst != &detail::dummy_loc) {
        _descPtr_->DSTADDR.reg = cfg.dst ? static_cast<uintptr_t>(cfg.dst) 
          + detail::addr_mod(_descPtr_, false) : 0;         
      } else if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        _descPtr_->DSTADDR.reg = prev_dstaddr + detail::addr_mod(_descPtr_, false);
      }
    }

    void set_length(const uint32_t &length, const bool &in_bytes) {
      _descPtr_->BTCTRL.bit.VALID = 0;
      uintptr_t s_addr = _descPtr_->SRCADDR.reg - detail::addr_mod(*_descPtr_, true);
      uintptr_t d_addr = _descPtr_->DSTADDR.reg - detail::addr_mod(*_descPtr_, false);

      uint32_t cnt_reg = length;
      if (in_bytes && length) {
          cnt_reg /= ref::beatsize_map[_descPtr_->BTCTRL.bit.BEATSIZE];
      }
      // Set new beatcount & re-calc src/dst (mod changes w/ cnt)
      _descPtr_->BTCNT.reg = cnt_reg;
      _descPtr_->SRCADDR.reg = s_addr + detail::addr_mod(*_descPtr_, true);
      _descPtr_->DSTADDR.reg = d_addr + detail::addr_mod(*_descPtr_, false);
    }

    uint32_t length(const bool &in_bytes) const {
      uint32_t length = _descPtr_->BTCNT.reg;
      if (in_bytes) {
        length /= ref::beatsize_map[_descPtr_->BTCTRL.bit.BEATSIZE];
      }
      return length;
    }

    bool unlink() {
      if (assig_ch != -1) {
        auto crit = detail::SuspendSection(assig_ch);

        // If base -> move base back into local memory
        if (btd[assig_ch] == this) {
          memcpy(&_desc_, &bdesc[assig_ch], detail::_dsize_);
          _descPtr_ = &_desc_;
          btd[assig_ch] = next;

          // If next -> move down into base memory
          if (next && next != this) {
            memcpy(&bdesc[assig_ch], &next->_desc_, detail::_dsize_);
            next->_descPtr_ = &next->_desc_;
          } else {
            memset(&bdesc[assig_ch], 0U, detail::_dsize_);
            memset((void*)&wbdesc[assig_ch], 0U, detail::_dsize_);
          }
        } else { 
          // Find & Re-link writeback/previous descriptor
          TransferDescriptor *prev{nullptr};
          while(prev->next != this) {
            prev = prev->next;
          }
          if (wbdesc[assig_ch].DESCADDR.reg == prev->_descPtr_->DESCADDR.reg) {
            wbdesc[assig_ch].DESCADDR.reg = _descPtr_->DESCADDR.reg;
          } 
          prev->_descPtr_->DESCADDR.reg = std::exchange(_descPtr_->DESCADDR.reg, 0);
          prev->next = next;
        }
        next = nullptr;
        assig_ch = -1;
      }
      return true;
    }

    ~TransferDescriptor() {
      unlink();
    }

    DmacDescriptor *_descPtr_{nullptr};
    DmacDescriptor _desc_{};
    uint32_t assig_ch{-1};
    TransferDescriptor *next{nullptr};
  };


  struct BasePeripheral : private detail::InstCommon<inst>
  {

    struct Config {
      std::array<int32_t, DMAC_LVL_NUM> prilvl_en{-1}; // Boolean cond
      std::array<int32_t, DMAC_LVL_NUM> prilvl_rr{-1}; // Boolean cond
      std::array<int32_t, DMAC_LVL_NUM> prilvl_squal{-1}; 
      std::array<int32_t, ref::irq_array.size()> irq_priority{}; 
    };


    template<Config cfg>
    void set_config() {
      using namespace ref;

      // Compute masks for clearing & setting cfg in prictrl reg
      constexpr auto prictrl_masks = []() consteval {
        using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
        prictrl_t set_msk{0}, clr_msk{0};
        
        for (uint32_t i = 0; i < DMAC_LVL_NUM; i++) {
          constexpr int32_t rr_val = cfg.prilvl_rr.at(i);
          constexpr int32_t sq_val = cfg.prilvl_squal.at(i);

          if (rr_val != -1) {
            constexpr squal_reg = std::distance(squal_map.begin(), 
              std::find(squal_map.begin(), squal_map.end(), sq_val));
            static_assert(squal_reg < squal_map.size(), "SIO ERROR: DMA controller config");
            
            uint32_t off = i * (DMAC_PRICTRL0_QOS1_Pos - DMAC_PRICTRL0_QOS0_Pos);
            clr_msk |= (DMAC_PRICTRL0_QOS0_Msk << off);
            set_msk |= (cfg.prilvl_squal.at(i) << (off + DMAC_PRICTRL0_QOS0_Pos));
          }
          if (sq_val != -1) {
            uint32_t off = i * (DMAC_PRICTRL0_RRLVLEN1_Pos - DMAC_PRICTRL0_RRLVLEN0_Pos);
            clr_msk |= (1U << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
            set_msk |= (static_cast<bool>(rr_val) << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
          }
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      DMAC[inst].PRICTRL0.reg &= ~prictrl_masks.first;
      DMAC[inst].PRICTRL0.reg |= prictrl_masks.second;
      
      // Compute masks for clearing & setting config in ctrl reg
      constexpr auto ctrl_masks = []() consteval {
        using ctrl_t = decltype(std::declval<DMAC_CTRL_Type>().reg);
        ctrl_t clr_msk{0}, set_msk{0};

        for (uint32_t i = 0; i < DMAC_LVL_NUM; i++) {
          constexpr int32_t lvlen_val = cfg.prilvl_en.at(i);

          if (lvlen_val != -1) {
            uint32_t off = i * (DMAC_CTRL_LVLEN1_Pos - DMAC_CTRL_LVLEN0_Pos);
            clr_msk |= (1U << (off + DMAC_CTRL_LVLEN0_Pos));
            set_msk |= (static_cast<bool>(lvlen_val) << (off + DMAC_CTRL_LVLEN0_Pos));
          }
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      DMAC[inst].CTRL.reg &= ~ctrl_masks.first;
      DMAC[inst].CTRL.reg |= ctrl_masks.second;

      // Compute and set all irq priority lvl values
      constexpr auto irqpri_array = []() consteval {
        std::array<uint32_t, irq_num> pri_array;

        for (uint32_t i = 0; i < irq_num; i++) {
          constexpr int32_t pri_val = cfg.irq_priority.at(i);
          if (pri_val != -1) {
            constexpr uint32_t pri_reg = std::distance(irqpri_map.begin(),
                std::find(irqpri_map.begin(), irqpri_map.end(), pri_val));
            static_assert(pri_reg < irqpri_map.size(), "SIO ERROR: DMA controller config");
            pri_array.at(i) = pri_reg;
          }
        }
        return pri_array;
      }(); 
      for (uint32_t i = 0; i < irq_num; i++) {
        NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), irqpri_array[i]);
      }
    }


    void init() {
      // Ensure exit. If used after exit -> exit again;
      if (!get_init()) {
        if (DMAC[inst].BASEADDR.reg) { 
          exit();
        } 
        // Enable interrupts
        constexpr uint32_t inst_irq_off = ref::irq_array.size() * inst;
        for (uint32_t i = 0; i < ref::irq_array.size(); i++) {
          NVIC_EnableIRQ(static_cast<IRQn_Type>(inst_irq_off + ref::irq_array[i]));
        }
        // Set base descriptor memory section address.
        DMAC[inst].BASEADDR.reg = reinterpret_cast<uintptr_t>(bdesc);
        DMAC[inst].WRBADDR.reg = reinterpret_cast<uintptr_t>(wbdesc);
        
        // Enable AHB bus clock, enable priority lvls (by default) & enable DMA
        DMAC[inst].CTRL.vec.LVLEN = 1;
        MCLK->AHBMASK.reg |= (1 << (DMAC_CLK_AHB_ID + inst));
        DMAC[inst].CTRL.bit.DMAENABLE = 1;
      }
    }


    void exit() {
      // Disable & reset DMA registers
      DMAC[inst].CTRL.bit.DMAENABLE = 0;
      while(DMAC[inst].CTRL.bit.DMAENABLE);
      DMAC[inst].CTRL.bit.SWRST = 1;
      while(DMAC[inst].CTRL.bit.SWRST);
      MCLK->AHBMASK.reg &= ~(1 << (DMAC_CLK_AHB_ID + inst)); 

      // Reset all interrupts -> offset of irq num = instance num * irq count
      constexpr uint32_t inst_irq_off = ref::irq_array.size() * inst;
      for (uint32_t i = 0; i < ref::irq_array.size(); i++) {
        IRQn_Type curr_irq = static_cast<IRQn_Type>(inst_irq_off + ref::irq_array[i]);
        NVIC_DisableIRQ(curr_irq);
        NVIC_ClearPendingIRQ(curr_irq);
        NVIC_SetPriority(curr_irq, 0);
      }
      // Reset non-register memory sections
      memset(bdesc, 0U, sizeof(bdesc));
      memset(const_cast<DmacDescriptor*>(wbdesc), 0U, sizeof(wbdesc));
      ch_msk = 0;
    }


    inline bool get_init() const {
      return DMAC[inst].CTRL.bit.DMAENABLE
          && DMAC[inst].BASEADDR.reg == reinterpret_cast<uintptr_t>(&bdesc)
          && DMAC[inst].WRBADDR.reg == reinterpret_cast<uintptr_t>(&wbdesc);  
    }

    inline uint32_t allocate_channel() {
      
    }

    inline uint32_t free_channel_count() const {
      return ref::max_ch - std::popcount(ch_msk);
    }

    inline uint32_t active_channel_id() const {
      return DMAC[inst].ACTIVE.bit.ID;
    }

    inline uint32_t active_channel_count() const {
      uint32_t count = 0;
      for (uint32_t i = 0; i < ref::max_ch; i++) {
        if ((DMAC[inst].BUSYCH.reg & (1 << (i + DMAC_BUSYCH_BUSYCH0_Pos)) ||  // Channel busy
            DMAC[inst].PENDCH.reg & (1 << (i + DMAC_PENDCH_PENDCH0_Pos))) &&  // OR Channel pending
            !suspf[i] && !DMAC[inst].Channel[i].CHINTFLAG.bit.SUSP) {       // AND channel NOT suspended
          count++;
        }
      }
      return count;
    }

  }; // END OF BASE_PERIPHERAL STRUCT


  

  struct Channel : private detail::InstCommon<inst> 
  {
    const uint32_t id = 3;

    struct Config {
      int32_t burstlen   = -1; // > Numeric 
      trigsrc_t trigsrc  = trigsrc_t::null;
      trigact_t trigact  = trigact_t::null;
      int32_t prilvl     = -1; 
      int32_t threshold  = -1; // > Numeric
      int32_t runstdby   = -1; // > Boolean cond
    };

    template<Config cfg>
    inline void set_config() {
      using namespace ref;

      // Compute masks for config in ctrl register
      constexpr auto chctrl_masks = [&]() consteval {
        using ctrl_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
        ctrl_t clr_msk{0}, set_msk{0};

        if (cfg.burstlen != -1) {
          constexpr uint32_t bl_reg = std::distance(burstlen_map.begin(),
              std::find(burstlen_map.begin(), burstlen_map.end(), cfg.burstlen));
          clr_msk |= (DMAC_CHCTRLA_BURSTLEN_Msk);
          set_msk |= (bl_reg << DMAC_CHCTRLA_BURSTLEN_Pos);
        }
        if (cfg.threshold != -1) {
          constexpr uint32_t th_reg = std::distance(threshold_map.begin(),
              std::find(threshold_map.begin(), threshold_map.end(), cfg.threshold));
          clr_msk |= (DMAC_CHCTRLA_THRESHOLD_Msk);
          set_msk |= (th_reg << DMAC_CHCTRLA_THRESHOLD_Pos);
        }
        if (cfg.trigsrc != trigsrc_t::null) {
          clr_msk |= (DMAC_CHCTRLA_TRIGSRC_Msk);
          set_msk |= (static_cast<uint32_t>(cfg.trigsrc) << DMAC_CHCTRLA_TRIGSRC_Pos);
        }
        if (cfg.trigact != trigact_t::null) {
          clr_msk |= (DMAC_CHCTRLA_TRIGACT_Msk);
          set_msk |= (static_cast<uint32_t>(cfg.trigact) << DMAC_CHCTRLA_TRIGACT_Pos);
        }
        if (cfg.runstdby != -1) {
          clr_msk |= (DMAC_CHCTRLA_RUNSTDBY);
          set_msk |= (static_cast<bool>(cfg.runstdby) << DMAC_CHCTRLA_RUNSTDBY_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }(); 
      DMAC[inst].Channel[id].CHCTRLA.reg &= ~chctrl_masks.first; 
      DMAC[inst].Channel[id].CHCTRLA.reg |= chctrl_masks.second; 

      // Compute masks for config in prictrl register & set them
      constexpr auto chprilvl_masks = [&]() consteval {
        using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
        prictrl_t clr_msk{0}, set_msk{0};

        if (cfg.prilvl != -1) {
         constexpr uint32_t plvl_reg = std::distance(prilvl_map.begin(),
              std::find(prilvl_map.begin(), prilvl_map.end(), cfg.prilvl));
          clr_msk |= (DMAC_CHPRILVL_PRILVL_Msk);
          set_msk |= (plvl_reg << DMAC_CHPRILVL_PRILVL_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      DMAC[inst].Channel[id].CHPRILVL.reg &= ~chprilvl_masks.first;
      DMAC[inst].Channel[id].CHPRILVL.reg |= chprilvl_masks.second;
    }


    struct InterruptConfig
    {
      callback_t cb      = &detail::dummy_cb;
      int32_t susp_int   = -1; // > Boolean cond
      int32_t err_int    = -1; // > Boolean cond
      int32_t tcmpl_int  = -1; // > Boolean cond
    };

    template<InterruptConfig cfg>
    void set_interrupt_config() {
      // Compute masks for intenset register & set them
      constexpr auto chint_masks = [&]() consteval {
        using inten_t = decltype(std::declval<DMAC_CHINTENSET_Type>().reg);
        inten_t clr_msk{0}, set_msk{0};

        if (cfg.susp_int != -1) {
          clr_msk |= (DMAC_CHINTENCLR_SUSP);
          set_msk |= (static_cast<bool>(cfg.susp_int) << DMAC_CHINTENSET_SUSP_Pos);
        }
        if (cfg.err_int != -1) {
          clr_msk |= (DMAC_CHINTENCLR_TERR);
          set_msk |= (static_cast<bool>(cfg.err_int) << DMAC_CHINTENSET_TERR_Pos);
        }
        if (cfg.tcmpl_int != -1) {
          clr_msk |= (DMAC_CHINTENCLR_TCMPL);
          set_msk |= (static_cast<bool>(cfg.tcmpl_int) << DMAC_CHINTENSET_TCMPL_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      DMAC[inst].Channel[id].CHINTENSET.reg &= ~chint_masks.first;
      DMAC[inst].Channel[id].CHINTENSET.reg |= chint_masks.second;

      // Save callback to external array for interrupt to access
      if constexpr (cfg.cb != &detail::dummy_cb) { 
        detail::_cbarr_[id] = cfg.cb;
      }
    }

    void reset(const bool &clear_transfer) {
      DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;     // Disable DMA channel
      while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);  // Wait for abort
      DMAC[inst].Channel[id].CHCTRLA.bit.SWRST = 1;      // Software reset ch registers
      while(DMAC[inst].Channel[id].CHCTRLA.bit.SWRST);   // Wait for sw reset

      if (clear_transfer) {
        set_transfer<nullptr>(false);
      }
      memset(&bdesc[id], 0U, detail::_dsize_);
      memset(const_cast<DmacDescriptor*>(&wbdesc[id]), 0U, detail::_dsize_); 
      cbarr[id] = nullptr;
      suspf[id] = false;
    }

    bool set_state(const channel_state_t &state) {
      if (state != this->state()) {
        // Disable/enable channel (wait on disable -> abort transfer)
        if (state == channel_state_t::disabled) {
          DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;
          while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);
        } else {
          DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 1;
        }
        // Set flag and suspend/resume channel (wait on susp only)
        if (state == channel_state_t::suspended) {
          suspf[id] = true;
          DMAC[inst].Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
          while(DMAC[inst].Channel[id].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
        } else {
          suspf[id] = false;
          DMAC[inst].Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
          DMAC[inst].Channel[id].CHINTFLAG.bit.SUSP = 1;
        }
        return state == this->state();
      }
      return false;
    }

    inline channel_state_t state() const {
      if (!DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE) {
        return channel_state_t::disabled;
      } else if (DMAC[inst].Channel[id].CHINTFLAG.bit.SUSP || suspf[id]) {
        return channel_state_t::suspended;
      } 
      return channel_state_t::enabled;
    }

    inline void trigger() {
      DMAC[inst].SWTRIGCTRL.reg |= (1 << (id + DMAC_SWTRIGCTRL_SWTRIG0_Pos));
    }

    inline bool trigger_pending() const {
      return DMAC[inst].Channel[id].CHSTATUS.bit.PEND;
    }

    inline bool transfer_busy() const {
      return DMAC[inst].Channel[id].CHSTATUS.bit.BUSY
          && this->state() != channel_state_t::suspended;
    }

    void clear_transfer() {
      if (btd[id]) { 
        auto crit = detail::SuspendSection(id);

        // Move DmacDesc back into base td
        TransferDescriptor *curr = btd[id];
        memcpy(&curr->_desc_, &bdesc[id], detail::_dsize_);
        curr->_descPtr_ = &curr->_desc_;

        do { // Unlink all descriptors 
          curr->assig_ch = -1;
          curr->_descPtr_->DESCADDR.reg = 0;
          curr = std::exchange(curr->next, nullptr);
        } while(curr && curr != btd[id]);

        // Clear writeback & base ptr
        memset((void*)&wbdesc[id], 0U, detail::_dsize_);
        btd[id] = nullptr;
      } 
    }

    template<uint32_t N> requires (N > 0)
    void set_transfer(std::array<TransferDescriptor&, N> &tdlist, const bool &looped = false) {
      if (btd[id]) {
        clear_transfer();
      }
      auto crit = detail::SuspendSection(id);
      static inline TransferDescriptor dumm_td{};
      TransferDescriptor *prev = looped ? &tdlist[N - 1] : &dumm_td;

      // Move base desc into base mem section
      memcpy(&bdesc[id], &tdlist[0]._desc_, detail::_dsize_);
      tdlist[0]._descPtr_ = &tdlist[0]._desc_;
      btd[id] = &tdlist[0];

      // Iterate through & link all descriptors
      for (TransferDescriptor &curr : tdlist) {
        prev->_descPtr_->DESCADDR.reg = reinterpret_cast
            <uintptr_t>(curr._descPtr_);
        prev->next = &curr;
        curr.assig_ch = id;
        prev = &curr;
      }      
    }

    void set_active_transfer(uint32_t td_index) {
      if (btd[id]) {
        auto crit = detail::SuspendSection(id);
        TransferDescriptor *curr = btd[id];
        for (uint32_t i = 0; i < td_index; i++) {
          curr = curr->next;
          if (!curr->next || curr->next == btd[id]) {
            break;
          }
        } // Copy descriptor @ index into writeback descriptor
        memcpy((void*)&wbdesc[id], curr->_descPtr_, detail::_dsize_);
      }
    }

    void set_active_transfer(TransferDescriptor *td) {
      if (btd[id]) {
        auto crit = detail::SuspendSection(id);
        if (td) {
          memcpy((void*)&wbdesc[id], td->_descPtr_, detail::_dsize_);
        } else {
          memset((void*)&wbdesc[id], 0U, detail::_dsize_);
          set_state(channel_state_t::disabled);
        }
      }
    }

    TransferDescriptor *active_transfer() const {
      auto crit = detail::SuspendSection(id);
      TransferDescriptor *active = btd[id];
      if (active && wbdesc[id].SRCADDR.reg) {
        do { 
          if (active->_descPtr_->DESCADDR.reg == wbdesc[id].DESCADDR.reg) {
            break;
          }
          active = active->next;
        } while(active && active != btd[id]);
      }
      return active;
    }

    void set_active_transfer_length(const uint32_t &length, const bool &in_bytes) {
      auto crit = detail::SuspendSection(id);

      if (btd[id] && wbdesc[id].SRCADDR.reg) {
        DmacDescriptor &wb = const_cast<DmacDescriptor&>(wbdesc[id]);

        // Save initial (non mod) addresses -> must update with new mod 
        uintptr_t s_addr = wb.SRCADDR.reg - detail::addr_mod(wb, true);
        uintptr_t d_addr = wb.DSTADDR.reg - detail::addr_mod(wb, false);

        // Update btcnt reg & src/dst addr
        uint32_t new_cnt = length;
        if (in_bytes && length) {
          new_cnt /= ref::beatsize_map[wb.BTCTRL.bit.BEATSIZE];
        }
        wb.BTCNT.reg = new_cnt;
        wb.SRCADDR.reg = s_addr + detail::addr_mod(wb, true);
        wb.DSTADDR.reg = d_addr + detail::addr_mod(wb, false);
      }
    }

    int32_t active_transfer_length(const bool &in_bytes) const {
      int32_t length = -1;
      if(btd[id] && wbdesc[id].SRCADDR.reg) {
        length = wbdesc[id].BTCNT.reg;
        if (in_bytes) {
          length *= ref::beatsize_map[wbdesc[id].BTCTRL.bit.BEATSIZE];
        } 
      } 
      return length;
    }

  };  


}


