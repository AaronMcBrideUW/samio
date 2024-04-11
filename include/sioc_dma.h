
#pragma once
#include <sioc_dma_defs.h>
#include <sioc_dma_ref.h>
#include <sam.h>
#include <array>
#include <iterator>
#include <algorithm>
#include <utility>
#include <tuple>
#include <type_traits>
#include <bit>
#include <Arduino.h>
#include <span>

#define err_ch_cfg_invalid "SIO ERROR: dma channel config invalid."
#define err_periph_cfg_invalid "SIO ERROR: dma peripheral config invalid."
#define err_desc_cfg_invalid "SIO ERROR: dma descriptor config invalid."
#define err_internal "SIO ERROR: dma internal."

namespace sioc::dma {

  struct TransferDescriptor;

  namespace detail {

    inline struct {}dummy_loc;
    inline void dummy_cb(const uint32_t &a, const callback_flag_t &b) { }
    inline constexpr size_t _dsize_ = sizeof(DmacDescriptor);
    using dumm_t = decltype(detail::dummy_loc);
    using remove_fn = void (*)(TransferDescriptor*);

    /// @b COMPLETE_V2
    inline void update_valid(DmacDescriptor &desc) {
      desc.BTCTRL.bit.VALID = (desc.SRCADDR.reg && desc.DSTADDR.reg && desc.BTCNT.reg);
    }

    /// @b COMPLETE_V2
    inline uintptr_t addr_mod(const DmacDescriptor &desc, const bool &is_src) {
      uint32_t mod = 0;
      if (is_src ? desc.BTCTRL.bit.SRCINC : desc.BTCTRL.bit.DSTINC) {
        mod = desc.BTCNT.reg * ref::beatsize_map[desc.BTCTRL.bit.BEATSIZE]; // -> Ref datasheet 22.2.6.7  
        uint32_t sel = is_src ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;

        if (desc.BTCTRL.bit.STEPSEL == sel) {
          mod *= (1 << desc.BTCTRL.bit.STEPSIZE); // 2 ^ stepsize
        }
      }
      return mod;
    }

    /// @b TESTED_V1
    struct SuspendSection 
    { 
      SuspendSection(const int8_t &ch_inst, const int8_t &ch_id) 
          : inst(ch_inst), id(ch_id) {
        if (id >= 0 && id < ref::max_ch && 
            inst >= 0 && inst < DMAC_INST_NUM && 
            DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE && 
            !InstCommon[id].suspf[id] && 
            !DMAC[inst].Channel[id].CHINTFLAG.bit.SUSP) {

          susp_flag = true;
          InstCommon[inst].suspf[id] = true;
          DMAC[inst].Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;         
          while(DMAC[inst].Channel[id].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
        }
      }
      ~SuspendSection() {
        if (susp_flag) {
          auto &comm = InstCommon[inst];
          if (!comm.wbdesc[id].BTCNT.reg && !comm.wbdesc[id].DESCADDR.reg) {
            DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;
            DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 1;
          }
          comm.suspf[id] = false;
          DMAC[inst].Channel[id].CHINTFLAG.bit.SUSP = 1;
          DMAC[inst].Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
        }
      }
      const int8_t inst;
      const int8_t id;      // Channel id
      bool susp_flag = false; // True = not initially suspended
    };

  } // END OF ANON NAMESPACE 



  struct TransferDescriptor
  {
    
    /// @b COMPLETE_V2
    TransferDescriptor() = default;

    /// @b COMPLETE_V2
    TransferDescriptor(const TransferDescriptor &other) {
      this->operator = (other);
    }

    /// @b COMPLETE_V1
    TransferDescriptor(TransferDescriptor &&other) {
      this->operator = (std::move(other));
    }

    /// @b TESTED_V1
    TransferDescriptor &operator = (const TransferDescriptor &other) {
      if (&other != this) {
        auto l_addr = _descPtr_->DESCADDR.reg;
        memcpy(_descPtr_, other._descPtr_, detail::_dsize_);
        _descPtr_->DESCADDR.reg = l_addr;
      }
      return *this;
    }

    /// @b TODO
    TransferDescriptor &operator = (TransferDescriptor &&other) {
      if (&other != this) {
        auto crit = detail::SuspendSection(other.assig.inst, other.assig.id);
        //unlink(); 
        // Get other's descriptor or ptr (if base only)
        if (other._descPtr_ != &other._desc_) {
          _descPtr_ = std::exchange(other._descPtr_, &other._desc_);
        } else {
          memcpy(&_desc_, &other._desc_, detail::_dsize_);
        }
        // Relink prev/writeback if other is linked
        if (other.assig.inst != -1 && other.assig.id != -1) {
          auto &comm = detail::InstCommon[other.assig.inst];
          TransferDescriptor *prev = comm.btd[other.assig.id];
          do {
            if (prev->next == &other) {
              auto l_addr = reinterpret_cast<uintptr_t>(_descPtr_); 
              if (comm.wbdesc[other.assig.id].DESCADDR.reg == 
                  prev->_descPtr_->DESCADDR.reg) {
                comm.wbdesc[other.assig.id].DESCADDR.reg = l_addr;
              }
              prev->_descPtr_->DESCADDR.reg = l_addr;
              prev->next = this;
            }
          } while(prev && prev != comm.btd[other.assig.id]);
          if (_descPtr_ != &_desc_) {
            comm.btd[other.assig.id] = this;
          }
        } // Exchange fields
        next = std::exchange(other.next, nullptr);
        assig.inst = std::exchange(other.assig.inst, -1);
        assig.id = std::exchange(other.assig.id, -1);
      }
      return *this;
    }

    /// @b TESTED_V1
    void swap(TransferDescriptor &other, const bool &swap_descriptors = true) {
      if (&other != this) {
        auto crit_o = detail::SuspendSection(other.assig.inst, other.assig.id);
        auto crit_t = detail::SuspendSection(assig.inst, assig.id);

        if (swap_descriptors) { // Swap descriptors or ptrs (if base only)
          DmacDescriptor *this_ptr{_descPtr_}; 
          DmacDescriptor this_desc{};
          memcpy(&this_desc, _descPtr_, detail::_dsize_);

          if (other._descPtr_ != &other._desc_) {
            _descPtr_ = other._descPtr_;
          } else {
            memcpy(&_desc_, &other._desc_, detail::_dsize_);
            _descPtr_ = &_desc_;
          }
          if (this_ptr != &_desc_) {
            other._descPtr_ = this_ptr;
          } else {
            memcpy(&other._desc_, &this_desc, detail::_dsize_);
            other._descPtr_ = &other._desc_;
          }
        } else { // If this/other = base -> move this/other into & out of base mem
          if (other._descPtr_ != &other._desc_ && _descPtr_ != &_desc_) {
            DmacDescriptor this_temp{};
            memcpy(&this_temp, _descPtr_, detail::_dsize_);
            memcpy(_descPtr_, other._descPtr_, detail::_dsize_);
            memcpy(other._descPtr_, &this_temp, detail::_dsize_);

          } else if (other._descPtr_ != &other._desc_) {
            memcpy(&other._desc_, other._descPtr_, detail::_dsize_);
            memcpy(&other._descPtr_, &_desc_, detail::_dsize_);
            _descPtr_ = std::exchange(other._descPtr_, &other._desc_);
          
          } else if (_descPtr_ != &_desc_) {
            memcpy(&_desc_, _descPtr_, detail::_dsize_);
            memcpy(_descPtr_, &other._desc_, detail::_dsize_);
            other._descPtr_ = std::exchange(_descPtr_, &_desc_);
          }
          std::swap(_descPtr_->DESCADDR.reg, other._descPtr_->DESCADDR.reg);
        }
        // Relink other's prev -> this & this prev -> other
        auto relink_prev = [&](TransferDescriptor &old, TransferDescriptor &targ) {
          if (old.assig.inst != -1 && old.assig.id != -1) {
            auto &comm = detail::InstCommon[old.assig.inst];
            auto wb = const_cast<DmacDescriptor*>(&comm.wbdesc[targ.assig.id]);
            TransferDescriptor *prev = comm.btd[old.assig.id];
            do {
              if (prev->next == &old) {
                auto new_addr = reinterpret_cast<uintptr_t>(targ._descPtr_);
                if (wb->DESCADDR.reg == prev->_descPtr_->DESCADDR.reg) {
                  comm.wbdesc[targ.assig.id].DESCADDR.reg = new_addr;
                }
                prev->_descPtr_->DESCADDR.reg = new_addr;
                prev->next = &targ;
              }
              prev = prev->next;
            } while(prev && prev != comm.btd[old.assig.id]);
          }
        };
        relink_prev(other, *this);
        relink_prev(*this, other);

        // Reassign base td & swap fields
        if (_descPtr_ != &_desc_) {
          detail::InstCommon[other.assig.inst].btd[other.assig.id] = this;
        }
        if (other._descPtr_ != &other._desc_) {
          detail::InstCommon[assig.inst].btd[assig.id] = &other;
        }
        std::swap(assig, other.assig);
        std::swap(next, other.next);
        detail::update_valid(*_descPtr_);
        detail::update_valid(*other._descPtr_);
      }
    }

    /// @b TESTED_V2
    template<typename src_t = detail::dumm_t, 
             typename dst_t = detail::dumm_t>
    struct Config 
    {
      uint32_t beatsize   = -1; // > numeric
      src_t *src          = &detail::dummy_loc; // > ptr
      int32_t srcinc      = -1; // > numeric
      dst_t *dst          = &detail::dummy_loc; // > ptr
      int32_t dstinc      = -1; // > numeric
      blockact_t blockact = blockact_t::null;
    };

    /// @b TESTED_V2
    template<Config cfg>
    void set_config() {
      using namespace ref;
      _descPtr_->BTCTRL.bit.VALID = 0;
      auto prev_bs = beatsize_map[_descPtr_->BTCTRL.bit.BEATSIZE];
      uintptr_t prev_srcaddr{0}, prev_dstaddr{0};

      // Capture previous, non-modified address if beatsize changed
      if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        if constexpr (std::is_same_v<std::remove_pointer_t<decltype(cfg.src)>, detail::dumm_t>) {
          prev_srcaddr = _descPtr_->SRCADDR.reg - detail::addr_mod(*_descPtr_, true);
        }
        if constexpr (std::is_same_v<std::remove_pointer_t<decltype(cfg.dst)>, detail::dumm_t>) {
          prev_dstaddr = _descPtr_->DSTADDR.reg - detail::addr_mod(*_descPtr_, false);
        }
      }
      // Compute ctrl register mask
      constexpr auto btctrl_masks = [&]() consteval {
        using ctrl_t = decltype(std::declval<DMAC_BTCTRL_Type>().reg);        
        ctrl_t clr_msk{0}, set_msk{0};

        if constexpr (cfg.beatsize != -1) {
          constexpr uint32_t bs_reg = std::distance(beatsize_map.begin(), 
              std::find(beatsize_map.begin(), beatsize_map.end(), cfg.beatsize));  
          static_assert(bs_reg < beatsize_map.size(), err_desc_cfg_invalid);

          clr_msk |= (DMAC_BTCTRL_BEATSIZE_Msk);
          set_msk |= (bs_reg << DMAC_BTCTRL_BEATSIZE_Pos);
        }
        if constexpr (std::max(cfg.srcinc, cfg.dstinc) > 1) {
          constexpr uint32_t ssize_r = std::distance(stepsize_map.begin(), std::find
              (stepsize_map.begin(), stepsize_map.end(), std::max(cfg.srcinc, cfg.dstinc)));
          static_assert(ssize_r < stepsize_map.size(), err_desc_cfg_invalid);

          clr_msk |= (DMAC_BTCTRL_STEPSIZE_Msk);
          set_msk |= (ssize_r << DMAC_BTCTRL_STEPSIZE_Pos);
        }
        if constexpr (cfg.srcinc != -1) {
          clr_msk |= (DMAC_BTCTRL_SRCINC);
          set_msk |= (static_cast<bool>(cfg.srcinc) << DMAC_BTCTRL_SRCINC_Pos);
        }
        if constexpr (cfg.dstinc != -1) {
          clr_msk |= (DMAC_BTCTRL_DSTINC);
          set_msk |= (static_cast<bool>(cfg.dstinc) << DMAC_BTCTRL_DSTINC_Pos);
        }
        if constexpr (cfg.srcinc > 1) {
          static_assert(cfg.dstinc <= 1, err_desc_cfg_invalid); 
          clr_msk |= (DMAC_BTCTRL_STEPSEL);
          set_msk |= (DMAC_BTCTRL_STEPSEL_SRC);
        }
        if constexpr (cfg.dstinc > 1) {
          static_assert(cfg.srcinc <= 1, err_desc_cfg_invalid);
          clr_msk |= (DMAC_BTCTRL_STEPSEL);
          set_msk |= (DMAC_BTCTRL_STEPSEL_DST);
        }
        if constexpr (cfg.blockact != blockact_t::null) {
          clr_msk |= (DMAC_BTCTRL_BLOCKACT_Msk);
          set_msk |= (static_cast<ctrl_t>(cfg.blockact) << DMAC_BTCTRL_BLOCKACT_Pos);
        }
        return std::pair(clr_msk, set_msk); 
      }();
      _descPtr_->BTCTRL.reg &= ~btctrl_masks.first; 
      _descPtr_->BTCTRL.reg |= btctrl_masks.second;

      // Set source address or re-calc previous if beatsize changed
      if constexpr (!std::is_same_v<std::remove_pointer_t<decltype(cfg.src)>, detail::dumm_t>) {
        _descPtr_->SRCADDR.reg = cfg.src ? (uintptr_t)cfg.src 
            + detail::addr_mod(*_descPtr_, true) : 0;
      } else if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        _descPtr_->SRCADDR.reg = prev_srcaddr + detail::addr_mod(*_descPtr_, true);
      }
      // Set destination address or re-calc prev if beatsize changedd
      if constexpr (!std::is_same_v<std::remove_pointer_t<decltype(cfg.dst)>, detail::dumm_t>) {
        _descPtr_->DSTADDR.reg = cfg.dst ? (uintptr_t)cfg.dst 
            + detail::addr_mod(*_descPtr_, false) : 0;
      } else if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        _descPtr_->DSTADDR.reg = prev_dstaddr + detail::addr_mod(*_descPtr_, false);
      }
      detail::update_valid(*_descPtr_);
    }

    /// @b TESTED_V1
    void set_length(const uint32_t &length, const bool &in_bytes) {
      _descPtr_->BTCTRL.bit.VALID = 0;
      uintptr_t s_addr = _descPtr_->SRCADDR.reg - detail::addr_mod(*_descPtr_, true);
      uintptr_t d_addr = _descPtr_->DSTADDR.reg - detail::addr_mod(*_descPtr_, false);

      uint32_t cnt_reg = length;
      if (in_bytes && length) {
        uint32_t bs_val = ref::beatsize_map[_descPtr_->BTCTRL.bit.BEATSIZE];
        if (length % bs_val) { /* #WARN */ }
        cnt_reg /= bs_val;
      }
      // Set new beatcount & re-calc src/dst (mod changes w/ cnt)
      _descPtr_->BTCNT.reg = cnt_reg;
      _descPtr_->SRCADDR.reg = s_addr + detail::addr_mod(*_descPtr_, true);
      _descPtr_->DSTADDR.reg = d_addr + detail::addr_mod(*_descPtr_, false);
      detail::update_valid(*_descPtr_);
    }

    /// @b TESTED_V1
    uint32_t length(const bool &in_bytes) const {
      uint32_t length = _descPtr_->BTCNT.reg;
      if (in_bytes) {
        length /= ref::beatsize_map[_descPtr_->BTCTRL.bit.BEATSIZE];
      }
      return length;
    }

    /// @b COMPLETE_V2
    inline TransferDescriptor *linked_descriptor() const {
      return next;
    }

    /// @b COMPLETE_V2
    inline auto assigned_channel() const {
      return std::make_pair(assig.inst, assig.id);
    }

    /// @b COMPLETE_V2
    inline ~TransferDescriptor() {
      unlink();
    }

    DmacDescriptor *_descPtr_{&_desc_};
    DmacDescriptor _desc_{};
    void (*remove_fn)(TransferDescriptor*);
    TransferDescriptor *next{nullptr};
  };


  
  struct BasePeripheral
  {

    /// @a FINAL
    explicit BasePeripheral(const int &inst_num) {
      if (inst_num >= 0 && inst_num < ref::inst_num && !bp_arr[inst_num]) {
        bp_arr[inst_num] == this;
        inst = inst_num;
      }
    }

    ///@a FINAL
    explicit BasePeripheral(BasePeripheral &&other) {
      if (&other != this) {
        if (!other.invalid()) {
          bp_arr[other.inst] = this;
        }
        inst = std::exchange(other.inst, -1);
      }
    } 

    /// @a FINAL
    BasePeripheral &operator = (const BasePeripheral &other) {
      set_config(other);      
    }

    /// @a FINAL
    BasePeripheral &operator = (BasePeripheral &&other) {
      if (&other != this) {
        if (!other.invalid()) {
          bp_arr[other.inst] = this;
        }
        if (!invalid()) {
          bp_arr[inst] = &other;
        }
        std::swap(inst, other.inst);
      }
      return *this;
    }    

     /// @a FINAL
    void init() {
      if (invalid() || is_init()) { return; }
      exit();
      // Enable all irq interrupts
      int irq_off = ref::irq_array.size() * inst;
      for (int i = 0; i < ref::irq_array.size(); i++) {
        NVIC_EnableIRQ(static_cast<IRQn_Type>(irq_off + ref::irq_array[i]));
      }
      // Set base/writeback mem sections & enable dma/clock
      DMAC[inst].BASEADDR.reg = reinterpret_cast<uintptr_t>(bdesc);
      DMAC[inst].WRBADDR.reg = reinterpret_cast<uintptr_t>(wbdesc);
      MCLK->AHBMASK.reg |= (1 << (DMAC_CLK_AHB_ID + inst));
      DMAC[inst].CTRL.bit.DMAENABLE = 1;
    }

    /// @a FINAL
    void exit() {
      if (invalid() || !is_init()) { return; }
      // Disable dma/ahbbus & reset all registers
      DMAC[inst].CTRL.bit.DMAENABLE = 0;
      while(DMAC[inst].CTRL.bit.DMAENABLE);
      DMAC[inst].CTRL.bit.SWRST = 1;
      while(DMAC[inst].CTRL.bit.SWRST);
      MCLK->AHBMASK.reg &= ~(1 << (MCLK_AHBMASK_DMAC_Pos + inst));

      // Disable & reset all irq interrupts
      int irq_off = ref::irq_array.size() * inst;
      for (int i = 0; i < ref::irq_array.size(); i++) {
        auto curr_irq = static_cast<IRQn_Type>(ref::irq_array[i] + irq_off);
        NVIC_DisableIRQ(curr_irq);
        NVIC_ClearPendingIRQ(curr_irq);
        NVIC_SetPriority(curr_irq, ref::def_irq_prio);
      }
      // Reset fields
      memset(const_cast<DmacDescriptor*>(wbdesc), 0U, sizeof(wbdesc));
      memset(bdesc, 0U, sizeof(bdesc));

    }

    /// @a FINAL
    inline bool is_init() const {
      if (invalid()) { return false; }
      auto b_addr = reinterpret_cast<uintptr_t>(bdesc); 
      return DMAC[inst].CTRL.bit.DMAENABLE && DMAC[inst].BASEADDR.reg == b_addr;
    }

    /// @a FINAL
    struct Config
    {
      std::array<int32_t, DMAC_LVL_NUM> prilvl_en{-1, -1, -1, -1};             // Priority lvl enabled
      std::array<int32_t, DMAC_LVL_NUM> prilvl_rr{-1, -1, -1, -1};             // Priority lvl round robin arbitration
      std::array<int32_t, DMAC_LVL_NUM> prilvl_squal{-1, -1, -1, -1};          // Priority lvl service quality
      std::array<int32_t, ref::irq_array.size()> irq_prio{-1, -1, -1, -1, -1}; // Irq priority lvl
    };

    /// @a FINAL
    template<Config cfg>
    void set_config() {
      if (invalid() || !is_init()) { return; } // #WARN
      using namespace ref;

      // Compute reg values for irq priority
      constexpr auto irq_prio_arr = []() consteval {
        using prio_t = decltype(NVIC_GetPriority(DMAC_0_IRQn));
        return ([]<uint32_t... Is>(std::integer_sequence<uint32_t, Is...>) consteval {
          std::array<prio_t, ref::irq_array.size()> prio_arr{};

          // Get reg value for individual priority lvl
          constexpr auto irq_prio_i = []<uint32_t i>() consteval {
            constexpr auto irq_prio_cfg = cfg.irq_prio.at(i);
            prio_t prio_reg_f{0};
            
            if constexpr (irq_prio_cfg != -1) {
              constexpr uint32_t prio_reg = std::distance(irqpri_map.begin(),
                  std::find(irqpri_map.begin(), irqpri_map.end(), irq_prio_cfg));
              static_assert(irq_prio_cfg < irqpri_map.size(), 
                  "SIO ERROR (DMA): irq_prio config invalid.");
              prio_reg_f = prio_reg;
            }
            return prio_reg_f;
          }; 
          // Build up & return array of cfg reg values (fold expr.)
          return ((prio_arr.at(Is) = irq_prio_i.template operator()<Is>(), prio_arr), ...); 
        }.operator()<>(std::make_integer_sequence<uint32_t, ref::irq_array.size()>()));
      }();
      // Set priority lvl config
      constexpr uint32_t irq_off = irq_array.size() * inst;
      for (uint32_t i = 0; i < ref::irq_array.size(); i++) {
        auto irq = static_cast<IRQn_Type>(irq_off + ref::irq_array[i]);
        NVIC_SetPriority(irq, irq_prio_arr[i]); 
      }
      // Compute masks for and set priority lvl cfg
      constexpr auto prio_masks = []() consteval {
        using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
        using ctrl_t = decltype(std::declval<DMAC_CTRL_Type>().reg);
        return ([]<uint32_t... Is>(std::integer_sequence<uint32_t, Is...>) consteval {
          std::tuple<prictrl_t, prictrl_t, ctrl_t, ctrl_t> masks{}, temp_masks{};

          // Generate mask for specific priority lvl cfg
          constexpr auto prictrl_msk_i = []<uint32_t i>() consteval {
            constexpr auto squal_val = cfg.prilvl_squal.at(i);
            constexpr auto rr_val = cfg.prilvl_rr.at(i);
            constexpr auto en_val = cfg.prilvl_en.at(i);
            prictrl_t set_msk_prio{0}, clr_msk_prio{0};
            ctrl_t clr_msk_ctrl{0}, set_msk_ctrl{0};

            if constexpr (squal_val != -1) {
              constexpr auto squal_reg = std::distance(squal_map.begin(), 
                  std::find(squal_map.begin(), squal_map.end(), squal_val));
              static_assert(squal_reg < squal_map.size(), 
                  "SIO ERROR (DMA): prilvl_squal config invalid");
              
              uint32_t off = i * (DMAC_PRICTRL0_QOS1_Pos - DMAC_PRICTRL0_QOS0_Pos);
              clr_msk_prio |= (DMAC_PRICTRL0_QOS0_Msk << off);
              set_msk_prio |= (squal_reg << (off + DMAC_PRICTRL0_QOS0_Pos));
            }
            if constexpr (rr_val != -1) {
              static_assert(rr_val == 0 || rr_val == 1, 
                  "SIO ERROR (DMA): prilvl_rr config invalid");
              uint32_t off = i * (DMAC_PRICTRL0_RRLVLEN1_Pos - DMAC_PRICTRL0_RRLVLEN0_Pos);
              clr_msk_prio |= (1U << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
              set_msk_prio |= (static_cast<bool>(rr_val) << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
            }
            if constexpr (en_val != -1) {
              static_assert(en_val == 0 || en_val == 1, 
                  "SIO ERROR (DMA): prilvl_en config invalid");
              uint32_t off = i * (DMAC_CTRL_LVLEN1_Pos - DMAC_CTRL_LVLEN0_Pos);
              clr_msk_ctrl |= (1U << (off + DMAC_CTRL_LVLEN0_Pos));
              set_msk_ctrl |= (static_cast<bool>(en_val) << (off + DMAC_CTRL_LVLEN0_Pos));
            }
            return std::make_tuple(clr_msk_prio, set_msk_prio, clr_msk_ctrl, set_msk_ctrl);
          };
          // Iterate over priority lvls & generate masks
          return ((temp_masks = prictrl_msk_i.template operator()<Is>(),
              std::get<0>(masks) |= std::get<0>(temp_masks),
              std::get<1>(masks) |= std::get<1>(temp_masks),
              std::get<2>(masks) |= std::get<2>(temp_masks),
              std::get<3>(masks) |= std::get<3>(temp_masks), 
              masks), ...); 
        }.operator()<>(std::make_integer_sequence<uint32_t, DMAC_LVL_NUM>()));
      }();
      // Structured binding for masks, set config
      auto [clr_prictrl, set_prictrl, clr_ctrl, set_ctrl] = prio_masks;
      DMAC[inst].PRICTRL0.reg &= ~clr_prictrl;
      DMAC[inst].PRICTRL0.reg |= set_prictrl;
      DMAC[inst].CTRL.reg &= ~clr_ctrl;
      DMAC[inst].CTRL.reg |= set_ctrl;
    }

    void set_config(const BasePeripheral &other) {
      if (invalid() || other.invalid() || &other == this) [[unlikely]] { return; }
      // Copy config from registers
      DMAC[inst].CTRL.vec.LVLEN = DMAC[other.inst].CTRL.vec.LVLEN;
      DMAC[inst].PRICTRL0.reg = DMAC[other.inst].PRICTRL0.reg;

      // Copy/set irq priority level config
      unsigned int t_irq_off = ref::irq_array.size() * inst;
      unsigned int o_irq_off = ref::irq_array.size() * other.inst;

      for (int i = 0; i < ref::irq_array.size(); i++) {
        auto other_prio = NVIC_GetPriority(static_cast<IRQn_Type>(ref::irq_array[i] + o_irq_off));
        NVIC_SetPriority(static_cast<IRQn_Type>(ref::irq_array[i] + t_irq_off), other_prio);
      }
    }

    unsigned int active_channel_count() const {
      unsigned int count{0};
      for (int i = 0; i < ref::ch_num; i++) {
        if (DMAC[inst].Channel[i].CHSTATUS.bit.BUSY && Channel<3>)

      }

    }    




    protected:
      friend Channel;

      int inst{-1};
      DmacDescriptor bdesc[ref::ch_num]{};
      volatile DmacDescriptor wbdesc[ref::ch_num]{};

      std::array<Channel*, ref::ch_num> ch_arr{nullptr};
      static inline std::array<BasePeripheral*, ref::inst_num> bp_arr{nullptr};

      bool invalid() const {
        return inst < 0 || inst >= ref::inst_num;
      }
  };




  struct Channel
  {

    /// @a COMPLETE_V2
    Channel() = default;

    /// @a COMPLETE_V2
    template<unsigned int other_inst, unsigned int other_id>
      requires requires {
        other_inst < ref::inst_num;
        other_id < ref::ch_num;
      }
    explicit Channel(const Channel<other_inst, other_id> &other) {
      this->operator = (other);
    }

    /// @a COMPLETE_V2
    template<unsigned int other_inst, unsigned int other_id>
      requires requires {
        other_inst < ref::inst_num;
        other_id < ref::ch_num;
      }
    explicit Channel(Channel<other_inst, other_id> &&other) {
      this->operator = (std::move(other));
    }

    /// @a COMPLETE_V2
    template<unsigned int other_inst, unsigned int other_id>
      requires requires {
        other_inst < ref::inst_num;
        other_id < ref::ch_num;
      }
    Channel &operator = (const Channel<other_inst, other_id> &other) {
      if (&other != this) [[likely]] {
        set_channel_config(other);
        set_interrupt_config(other);
      }
      return *this;
    }

    /// @a TODO
    template<int other_inst, int other_id>
    Channel &operator = (Channel<other_inst, other_id> &&other) {
      if (&other != this) [[likely]] {

      }
      return *this;
    }

    /// @a COMPLETE_V2
    struct TransferConfig 
    {
      int32_t burstlen   = -1; // > Numeric 
      trigsrc_t trigsrc  = trigsrc_t::null;
      trigact_t trigact  = trigact_t::null;
      int32_t prilvl     = -1; 
      int32_t threshold  = -1; // > Numeric
      int32_t runstdby   = -1; // > Boolean cond
    };

    /// @a COMPLETE_V2
    template<ChannelConfig cfg>
    void set_transfer_config() {
      using namespace ref;

      bool prev_en = DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE; // Capture prev enable state
      DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;            // Disable channel
      while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);         // Wait for sync/abort

      // Compute masks for config in ctrl register
      constexpr auto chctrl_masks = [&]() consteval {
        using ctrl_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
        ctrl_t clr_msk{0}, set_msk{0};

        if constexpr (cfg.burstlen != -1) {
          constexpr uint32_t bl_reg = std::distance(burstlen_map.begin(),
              std::find(burstlen_map.begin(), burstlen_map.end(), cfg.burstlen));
          static_assert(bl_reg < burstlen_map.size(), 
              "SIO ERROR (DMA): burstlen config invalid.");

          clr_msk |= (DMAC_CHCTRLA_BURSTLEN_Msk);
          set_msk |= (bl_reg << DMAC_CHCTRLA_BURSTLEN_Pos);
        }
        if constexpr (cfg.threshold != -1) {
          constexpr uint32_t th_reg = std::distance(threshold_map.begin(),
              std::find(threshold_map.begin(), threshold_map.end(), cfg.threshold));
          static_assert(th_reg < threshold_map.size(), 
              "SIO ERROR (DMA): threshold config invalid.");

          clr_msk |= (DMAC_CHCTRLA_THRESHOLD_Msk);
          set_msk |= (th_reg << DMAC_CHCTRLA_THRESHOLD_Pos);
        }
        if constexpr (cfg.trigsrc != trigsrc_t::null) {
          clr_msk |= (DMAC_CHCTRLA_TRIGSRC_Msk);
          set_msk |= (static_cast<uint32_t>(cfg.trigsrc) << DMAC_CHCTRLA_TRIGSRC_Pos);
        }
        if constexpr (cfg.trigact != trigact_t::null) {
          clr_msk |= (DMAC_CHCTRLA_TRIGACT_Msk);
          set_msk |= (static_cast<uint32_t>(cfg.trigact) << DMAC_CHCTRLA_TRIGACT_Pos);
        }
        if constexpr (cfg.runstdby != -1) {
          static_assert(cfg.runstdby == 0 || cfg.runstdby == 1, 
              "SIO ERROR (DMA): runstdby config invalid");
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

        if constexpr (cfg.prilvl != -1) {
          constexpr uint32_t plvl_reg = std::distance(prilvl_map.begin(),
              std::find(prilvl_map.begin(), prilvl_map.end(), cfg.prilvl));
          static_assert(plvl_reg < prilvl_map.size(), 
              "SIO ERROR (DMA): ch prilvl config invalid.");

          clr_msk |= (DMAC_CHPRILVL_PRILVL_Msk);
          set_msk |= (plvl_reg << DMAC_CHPRILVL_PRILVL_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      DMAC[inst].Channel[id].CHPRILVL.reg &= ~chprilvl_masks.first;
      DMAC[inst].Channel[id].CHPRILVL.reg |= chprilvl_masks.second;
      DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = prev_en; // > Re-enable channel 
    }

    /// @a COMPLETE_V2
    template<unsigned int other_inst, unsigned int other_id>
      requires requires {
        other_inst < ref::inst_num;
        other_id < ref::ch_num;
      }
    void set_channel_config(const Channel<other_inst, other_id> &other) {
      if (&other != this) {
        // Capture enable state & disable channel
        const bool prev_en = DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE;
        DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;
        while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);
        
        // Clear and set (with other) -> prilvl & ctrl registers
        DMAC[inst].Channel[id].CHPRILVL.reg = DMAC[other_inst].Channel[other_id].CHPRILVL.reg;
        DMAC[inst].Channel[id].CHCTRLA.reg = (DMAC[other_inst].Channel[other_id].CHCTRLA.reg 
            & ~(DMAC_CHCTRLA_ENABLE | DMAC_CHCTRLA_SWRST));
        DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = prev_en; // > Re-enable channel
      }
    }
    
    /// @a COMPLETE_V2
    struct InterruptConfig
    {
      callback_t cb      = &detail::dummy_cb;
      int32_t susp_int   = -1; // > Boolean cond
      int32_t err_int    = -1; // > Boolean cond
      int32_t tcmpl_int  = -1; // > Boolean cond
    };

    /// @a COMPLETE_V2
    template<InterruptConfig cfg>
    void set_interrupt_config() {
      // Compute and set masks for interrupt cfg reg
      constexpr auto chint_masks = [&]() consteval {
        using inten_t = decltype(std::declval<DMAC_CHINTENSET_Type>().reg);
        inten_t clr_msk{0}, set_msk{0};

        if constexpr (cfg.susp_int != -1) {
          static_assert(cfg.susp_int == 0 || cfg.susp_int == 1, 
              "SIO ERROR (DMA): ch susp_int config invalid");
          clr_msk |= (DMAC_CHINTENCLR_SUSP);
          set_msk |= (static_cast<bool>(cfg.susp_int) << DMAC_CHINTENSET_SUSP_Pos);
        }
        if constexpr (cfg.err_int != -1) {
          static_assert(cfg.err_int == 0 || cfg.err_int == 1, 
              "SIO ERROR (DMA): ch err_int config invalid");
          clr_msk |= (DMAC_CHINTENCLR_TERR);
          set_msk |= (static_cast<bool>(cfg.err_int) << DMAC_CHINTENSET_TERR_Pos);
        }
        if constexpr (cfg.tcmpl_int != -1) {
          static_assert(cfg.tcmpl_int == 0 || cfg.tcmpl_int == 1, 
              "SIO ERROR (DMA): ch tcmpl_int config invalid");
          clr_msk |= (DMAC_CHINTENCLR_TCMPL);
          set_msk |= (static_cast<bool>(cfg.tcmpl_int) << DMAC_CHINTENSET_TCMPL_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      DMAC[inst].Channel[id].CHINTENSET.reg &= ~chint_masks.first;
      DMAC[inst].Channel[id].CHINTENSET.reg |= chint_masks.second;
      
      // Save callback function
      if constexpr (cfg.cb != &detail::dummy_cb) { 
        cbarr[id] = cfg.cb;
      }
    }

    /// @a COMPLETE_V2
    template<int other_inst, int other_id>
    void set_interrupt_config(const Channel<other_inst, other_id> &other) {
      if (&other != this) {
        DMAC[inst].Channel[id].CHINTENSET.reg |= DMAC[other_inst].Channel[other_id].CHINTENSET.reg;
        cbarr[id] = other.cbarr[other_id];
      }
    }

    /// @a TODO
    static void reset(const bool &clear_transfer = true) {
      if (clear_transfer) {
        Channel<inst, id>::descriptor_list.clear_transfer();
      }
      // Reset registers
      DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;     // Disable DMA channel
      while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);  // Wait for abort
      DMAC[inst].Channel[id].CHCTRLA.bit.SWRST = 1;      // Software reset ch registers
      while(DMAC[inst].Channel[id].CHCTRLA.bit.SWRST);   // Wait for sw reset
      
      // Reset "fields" in common mem
      memset(const_cast<DmacDescriptor*>(&wbdesc[id]), 0U, detail::_dsize_); 
      memset(&bdesc[id], 0U, detail::_dsize_);
      cbarr[id] = nullptr;
      suspf[id] = false;
    }

    /// @a TODO
    static bool set_state(const channel_state_t &state) {
      channel_state_t i_state = Channel<inst, id>::state();
      if (state != i_state) {
        // Disable/enable channel (wait on disable -> abort transfer)
        if (state == channel_state_t::disabled) {
          DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;
          while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);
        } else {
          if (!wbdesc[id].BTCNT.reg && !wbdesc[id].DESCADDR.reg && 
            Channel<inst, id>::state() == channel_state_t::suspended) { 
            DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;
          }
          DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 1;
        }
        // Set flag and suspend/resume channel (wait on susp only)
        if (state == channel_state_t::suspended) {
          suspf[id] = true;
          DMAC[inst].Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
          while(DMAC[inst].Channel[id].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
        } else if (Channel<inst, id>::state() == channel_state_t::suspended) {
          suspf[id] = false;
          DMAC[inst].Channel[id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
          DMAC[inst].Channel[id].CHINTFLAG.bit.SUSP = 1;
        }
      }
      return state == Channel<inst, id>::state();
    }

    /// @b TODO
    static inline channel_state_t state() {
      if (!DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE) {
        return channel_state_t::disabled;
      } else if (DMAC[inst].Channel[id].CHINTFLAG.bit.SUSP || suspf[id]) {
        return channel_state_t::suspended;
      } 
      return channel_state_t::enabled;
    }

    /// @b TODO
    static inline void trigger() {
      DMAC[inst].SWTRIGCTRL.reg |= (1 << (id + DMAC_SWTRIGCTRL_SWTRIG0_Pos));
    }

    /// @b TODO
    inline bool trigger_pending() {
      return DMAC[inst].Channel[id].CHSTATUS.bit.PEND;
    }

    /// @b TODO
    static inline bool transfer_busy() {
      return DMAC[inst].Channel[id].CHSTATUS.bit.BUSY
          && state() != channel_state_t::suspended;
    }

    /// descriptor_list
    /// @b TODO
    static struct { 

      /// @b TODO
      void clear() {
        if (btd[id]) {
          // Disable channel & clear writeback
          DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 0;
          while(DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE);
          memset(const_cast<DmacDescriptor*>(&wbdesc[id]), 0U, detail::_dsize_);

          // Copy descriptor to obj (td) mem
          TransferDescriptor<inst> *curr = btd[id];
          memcpy(&curr->_desc_, &bdesc[id], detail::_dsize_);
          memset(&bdesc[id], 0U, detail::_dsize_);
          curr->_descPtr_ = &curr->_desc_;

          do { // unlink descriptors/td obj
            curr->assig_id = -1;
            curr->_descPtr_->DESCADDR.reg = 0;
            curr = std::exchange(curr->next, nullptr);
          } while(curr && curr != btd[id]);
        }
      }

      /// @b TODO
      void set(std::initializer_list<TransferDescriptor*> desclist, const bool &looped) {
        if (desclist.size() > 0) {
          bool prev_en = DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE;
          TransferDescriptor *prev = nullptr;
          clear();

          for (auto curr : desclist) {
            if (curr) [[likely]] {
              if (curr->assig.inst != -1 && curr->assig.id != -1) {
                curr->unlink();
                Channel<*curr->assig_inst, *curr->assig_id>
              }
              if (!prev) { // Copy base into base mem section
                memcpy(&comm.bdesc[id], &curr->_desc_, detail::_dsize_);
                curr->_descPtr_ = &comm.bdesc[id];
                comm.btd[id] = curr;
              
              } else { // Link prev -> curr
                auto l_addr = reinterpret_cast<uintptr_t>(curr->_descPtr_);
                prev->_descPtr_->DESCADDR.reg = l_addr;
                prev->next = curr;
              }
              curr->assig.inst = inst;
              curr->assig.id = id;
              prev = curr;
            }
          } 
          if (looped && prev) { // If loop: link last -> first
            auto l_addr = reinterpret_cast<uintptr_t>(comm.btd[id]->_descPtr_);
            prev->_descPtr_->DESCADDR.reg = l_addr;
            prev->next = comm.btd[id];
          }
          if (prev_en) {
            DMAC[inst].Channel[id].CHCTRLA.bit.ENABLE = 1;
          }
        }
      }

      /// @b TODO
      void set(TransferDescriptor *td, const bool &looped) {
        if (!valid_channel()) { return; }
        if (td) {
          set_transfer({td}, looped);
        } else {
          static constexpr std::array<TransferDescriptor*, 0> descarr{};
          set_transfer({}, false);
        }
      }

      uint32_t add(TransferDescriptor &td, const uint32_t &index) {
        if (!valid_channel()) { return -1; }
        auto crit = detail::SuspendSection(inst, id);
        auto &comm = detail::InstCommon[inst];
        TransferDescriptor *prev = comm.btd[id];
        uint32_t add_index{0};

        if (td.assig.id != -1) { 
          td.unlink();
        }
        while(prev && prev->next && prev->next != comm.btd[id] && add_index < index) {
          prev = prev->next;
          add_index++;
        }
        if (!add_index) { // (if base) Move base td out of base mem and added into base mem
          if (comm.btd[id]) {
            memcpy(&comm.btd[id]->_desc_, &comm.bdesc[id], detail::_dsize_);
            comm.btd[id]->_descPtr_ = &comm.btd[id]->_desc_;
            td.next = comm.btd[id];
          }
          memcpy(&comm.bdesc[id], td._descPtr_, detail::_dsize_);
          td._descPtr_ = &td._desc_;
          comm.btd[id] = &td;
        }
        if (prev->next == &td) { // Relink prev & writeback desc to td & td -> prev's next
          auto l_addr = reinterpret_cast<uintptr_t>(td._descPtr_);
          prev->_descPtr_->DESCADDR.reg = l_addr;
          td.next = std::exchange(prev->next, &td);

          if (comm.wbdesc[id].DESCADDR.reg == prev->_descPtr_->DESCADDR.reg) {
            comm.wbdesc[id].DESCADDR.reg = l_addr;
          }
        }
        td.assig.id = id;
        td.assig.inst = inst;
        return add_index;
      }

      /// @b TODO
      void remove(TransferDescriptor &td) {
        if (!valid_channel()) { return; }
        auto &comm = detail::InstCommon[inst];
        auto crit = detail::SuspendSection(inst, id);

        if (comm.btd[id] == &td) { // Copy desc out of base mem
          memcpy(&td._desc_, &comm.bdesc[id], detail::_dsize_);
          td._descPtr_ = &td._desc_;

          if (td.next && td.next != &td) { // Move next into base mem
            memcpy(&comm.bdesc[id], &td.next->_desc_, detail::_dsize_);
            td.next->_descPtr_ = &comm.bdesc[id];
            comm.btd[id] = td.next;

          } else { // Disable channel if this = last descriptor 
            memset(&comm.bdesc[id], 0U, detail::_dsize_);
            memset(const_cast<DmacDescriptor*>(&comm.wbdesc[id]), 0U, detail::_dsize_);
            set_state(channel_state_t::disabled);
            comm.btd[id] = nullptr; 
          }
        } else { // Relink previous & wb (if base -> prev linked to next) 
          TransferDescriptor *prev = comm.btd[id];
          while(prev->next != &td) { prev = prev->next; }
          prev->_descPtr_->DESCADDR.reg = td._descPtr_->DESCADDR.reg;
          prev->next = td.next;

          if (comm.wbdesc[id].DESCADDR.reg = prev->_descPtr_->DESCADDR.reg) {
            comm.wbdesc[id].DESCADDR.reg = td._descPtr_->DESCADDR.reg;
          }
        }
        td._descPtr_->DESCADDR.reg = 0;
        td.next = nullptr;
        td.assig.inst = -1;
        td.assig.id = -1;
      }

      /// @b TODO
      uint32_t index_of(TransferDescriptor &td) {
        uint32_t index{-1};
        if (valid_channel()) {
          auto &comm = detail::InstCommon[inst];
          if (comm.btd[id]) {
            TransferDescriptor *curr = comm.btd[id];
            index = 0;
            do {
              if (curr == &td) { 
                return index; 
              }
              curr = curr->next;
              index++;
            } while(curr && curr != comm.btd[id]);
          }
        } 
        return -1;
      }

      /// @a TODO
      TransferDescriptor *get(const uint32_t &index) {
        if (!valid_channel()) { return; }
        auto &comm = detail::InstCommon[inst];
        TransferDescriptor *curr = comm.btd[id];
        if (curr) {
          for (uint32_t i = 0; i < index; i++) {
            if (!curr->next || curr->next == comm.btd[id]) {
              return nullptr;
            }
            curr = curr->next;
          }
        }
        return curr;
      }

      TransferDescriptor *operator [] (const uint32_t index) {
        return get(index);
      }

    }descriptor_list;

    
    /// @b TODO
    /// active_transfer
    static struct {

      /// @b TODO
      void sync(const bool &sync_in_prog = false, 
        const bool &keep_length = false) {
        if (!valid_channel()) { return; }
        auto crit = detail::SuspendSection(inst, id);
        auto &comm = detail::InstCommon[inst];
        
        if (comm.btd[id] && comm.wbdesc[id].SRCADDR.reg && 
            (sync_in_prog || !DMAC[inst].Channel[id].CHSTATUS.bit.BUSY)) {
          auto *wb = const_cast<DmacDescriptor*>(&comm.wbdesc[id]);
          DmacDescriptor *dm = &comm.bdesc[id];

          while(dm->DESCADDR.reg != wb->DESCADDR.reg) {
            if (!dm->DESCADDR.reg) { return; }
            dm = reinterpret_cast<DmacDescriptor*>(dm->DESCADDR.reg);
          }
          if (keep_length) {
            auto prev_cnt = wb->BTCNT.reg;
            memcpy(wb, dm, detail::_dsize_);
            wb->BTCNT.reg = prev_cnt;
          } else {
            memcpy(wb, dm, detail::_dsize_);
          }
          if (!wb->BTCTRL.bit.VALID || (!wb->DESCADDR.reg && !wb->BTCNT.reg)) {
            set_state(channel_state_t::disabled);
          }
        }
      }

      /// @b TODO
      void set(uint32_t td_index) {
        if (!valid_channel()) { return; }
        auto &comm = detail::InstCommon[inst];

        if (comm.btd[id]) {
          auto crit = detail::SuspendSection(inst, id);
          TransferDescriptor *curr = comm.btd[id];
          for (uint32_t i = 0; i < td_index; i++) {
            curr = curr->next;
            if (!curr->next || curr->next == comm.btd[id]) {
              return;
            }
          } // Copy descriptor @ index into writeback descriptor
          auto wb = const_cast<DmacDescriptor*>(&comm.wbdesc[id]);
          memcpy(static_cast<void*>(wb), curr->_descPtr_, detail::_dsize_);
        }
      }

      /// @b TODO
      void set(TransferDescriptor *td, const bool &keep_link = true) {
        if (!valid_channel()) { return; }
        auto &comm = detail::InstCommon[inst];

        if (comm.btd[id]) {
          auto crit = detail::SuspendSection(inst, id);
          auto *wb = const_cast<DmacDescriptor*>(&comm.wbdesc[id]);
          auto prev_link = wb->DESCADDR.reg;
          if (td) {
            memcpy(static_cast<void*>(wb), td->_descPtr_, detail::_dsize_);
          } else {
            memset(static_cast<void*>(wb), 0U, detail::_dsize_);
          }
          if (keep_link) {
            wb->DESCADDR.reg = prev_link;
          } else if (td->assig.id != id) {
            wb->DESCADDR.reg = 0;
          }
          if (!wb->BTCTRL.bit.VALID || (!wb->BTCNT.reg && !wb->DESCADDR.reg)) {
            set_state(channel_state_t::disabled);
          }
        }
      }
      /// @b TODO
      TransferDescriptor *source_td() const {
        if (!valid_channel()) { return nullptr; }
        auto &comm = detail::InstCommon[inst];
        auto crit = detail::SuspendSection(inst, id);

        TransferDescriptor *active = comm.btd[id];
        if (active && comm.wbdesc[id].SRCADDR.reg) {
          do { 
            if (active->_descPtr_->DESCADDR.reg 
                == comm.wbdesc[id].DESCADDR.reg) {
              break;
            }
            active = active->next;
          } while(active && active != comm.btd[id]);
        }
        return active;
      }

      /// @b TODO
      int32_t index() const {
        if (!valid_channel()) { return -1; }
        auto &comm = detail::InstCommon[inst];
        auto crit = detail::SuspendSection(inst, id);

        TransferDescriptor *curr = comm.btd[id];
        uint32_t index = 0;
        if (curr && comm.wbdesc[id].SRCADDR.reg) {
          do {
            if (curr->_descPtr_->DESCADDR.reg 
                == comm.wbdesc[id].DESCADDR.reg) {
              return index;
            }
            curr = curr->next;
            index++;
          } while(curr && curr != comm.btd[id]);
        }
      }

      /// @b TODO
      void length(const uint32_t &length, const bool &in_bytes) {
        if (!valid_channel()) { return; }
        auto &comm = detail::InstCommon[inst];
        auto crit = detail::SuspendSection(inst, id);

        if (comm.btd[id] && comm.wbdesc[id].SRCADDR.reg) {
          DmacDescriptor &wb = const_cast<DmacDescriptor&>(comm.wbdesc[id]);

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

      /// @b TODO
      int32_t set_length(const bool &in_bytes) const {
        if (!valid_channel()) { return -1; }
        auto &comm = detail::InstCommon[inst];

        int32_t length = -1;
        if(comm.btd[id] && comm.wbdesc[id].SRCADDR.reg) {
          length = comm.wbdesc[id].BTCNT.reg;
          if (in_bytes) {
            length *= ref::beatsize_map[comm.wbdesc[id].BTCTRL.bit.BEATSIZE];
          } 
        } 
        return length;
      }
      
    }active_transfer;
  

    /// @b COMPLETE_V1
    ~Channel() {
      reset(true);
      if (!valid_channel()) { return; }
      auto &comm = detail::InstCommon[inst];
      comm.ch_msk &= ~(1 << id);

      if (!comm.ch_msk) { // Disable & reset DMA if no allocated channels
        DMAC[inst].CTRL.bit.DMAENABLE = 0;
        while(DMAC[inst].CTRL.bit.DMAENABLE);
        DMAC[inst].CTRL.bit.SWRST = 1;
        while(DMAC[inst].CTRL.bit.SWRST);
        MCLK->AHBMASK.reg &= ~(1 << (DMAC_CLK_AHB_ID + inst)); 

        // Reset all interrupts -> offset of irq num = instance num * irq count
        uint32_t inst_irq_off = ref::irq_array.size() * inst;
        for (uint32_t i = 0; i < ref::irq_array.size(); i++) {
          IRQn_Type curr_irq = static_cast<IRQn_Type>(inst_irq_off + ref::irq_array[i]);
          NVIC_DisableIRQ(curr_irq);
          NVIC_ClearPendingIRQ(curr_irq);
          NVIC_SetPriority(curr_irq, 0);
        }
        // Reset common section
        memset(const_cast<DmacDescriptor*>(comm.wbdesc), 0U, sizeof(comm.wbdesc));
        memset(comm.bdesc, 0U, sizeof(comm.bdesc));
        std::fill(std::begin(comm.btd), std::end(comm.btd), nullptr);
        std::fill(std::begin(comm.suspf), std::end(comm.suspf), false);
        std::fill(std::begin(comm.cbarr), std::end(comm.cbarr), nullptr);
        comm.ch_msk = 0;
      }
    }

    void test() {
      memcpy(bdesc[id], 0U, detail::_dsize_);

    }

    protected:
      friend BasePeripheral;
      BasePeripheral &bp;
      callback_t cbarr{nullptr};
      TransferDescriptor *btd{nullptr};
      volatile bool suspf[ref::ch_num]{false};
  };



  /// @b TODO
  uint32_t free_channel_count() {
    uint32_t count = 0;
    for (uint32_t i = 0; i < ref::inst_num; i++) {
      count += ref::max_ch - std::popcount(detail::InstCommon[i].ch_msk);
    }
    return count;
  } 

  /// @b COMPLETE_V1
  consteval std::pair<int32_t, int32_t> alloc_channel() {
    constexpr uint32_t cnt = __COUNTER__;
    if constexpr (cnt / ref::max_ch < ref::inst_num) {
      return std::make_pair(cnt / ref::max_ch, cnt % ref::max_ch);
    } 
    return std::make_pair(-1, -1); // > Alloc fail
  } 



    

  /// @b COMPLETE_V2
  template<uint32_t inst>
    requires (inst < DMAC_INST_NUM)
  void master_irq() {
    uint32_t id = DMAC[inst].INTPEND.bit.ID;
    auto &comm = detail::InstCommon[inst];

    if (id < ref::max_ch) {
      callback_flag_t cbf = callback_flag_t::null;
      
      // Suspend cmd interrupt
      if (DMAC[inst].INTPEND.bit.SUSP) {
        DMAC[inst].INTPEND.bit.SUSP = 1;
        if (!comm.suspf[id]) {
          comm.suspf[id] = true;
          cbf = callback_flag_t::susp;
        }
      // Transfer complete interrupt
      } else if (DMAC[inst].INTPEND.bit.TCMPL) {
        DMAC[inst].INTPEND.bit.TCMPL = 1;
        cbf = callback_flag_t::tcmpl;
      
      // Transfer error interrupt
      } else if (DMAC[inst].INTPEND.bit.TERR) {
        DMAC[inst].INTPEND.bit.TERR = 1;

        if (DMAC[inst].INTPEND.bit.FERR) {
          cbf = callback_flag_t::ferr;
        } else {
          cbf = callback_flag_t::terr;
        }
      }
      if (comm.cbarr[id] && cbf != callback_flag_t::null) {
        comm.cbarr[id](id, cbf);
      }
    } 
  }

} // END OF NAMESPACE SIOC::DMA

/// @b TODO->MOVE_OUT_OF_FILE
void DMAC_0_Handler() { sioc::dma::master_irq<0>(); }
void DMAC_1_Handler() { sioc::dma::master_irq<0>(); }
void DMAC_2_Handler() { sioc::dma::master_irq<0>(); }
void DMAC_3_Handler() { sioc::dma::master_irq<0>(); }
void DMAC_4_Handler() { sioc::dma::master_irq<0>(); }


