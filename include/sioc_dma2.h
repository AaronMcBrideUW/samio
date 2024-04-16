
#pragma once
#include <sam.h>
#include <sioc_dma_defs.h>
#include <sioc_dma_ref.h>
#include <sioc_meta.h>
#include <utility>
#include <algorithm>
#include <iterator>
#include <functional>
#include <bit>
#include <memory>
#include <string.h>
#include <sioc_temp.h>
#include <variant>


namespace sioc::dma 
{

  struct TransferDescriptor;  
  struct Peripheral;
  struct Peripheral::Channel;

  namespace detail 
  {
    
    /// @a FINAL_V1
    inline void update_valid(DmacDescriptor &desc) {
      desc.BTCTRL.bit.VALID = (desc.SRCADDR.reg && desc.DSTADDR.reg && desc.BTCNT.reg);
    }

    /// @a FINAL_V1
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
    
    /// @a FINAL_V2
    struct SuspendSection 
    { 
      SuspendSection(Channel *ch) : ch(ch) {
        if (ch && DMAC[ch->periph.inst_num].Channel[ch->ch_id].CHCTRLA.bit.ENABLE && 
            !(ch->suspf || DMAC[ch->periph.inst_num].Channel[ch->ch_id].CHINTFLAG.bit.SUSP)) {
          Dmac &dma = DMAC[ch->periph.inst_num];

          // (Channel not suspended) -> suspend channel & set flags true
          dma.Channel[ch->ch_id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;         
          while(dma.Channel[ch->ch_id].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
          susp_flag = true;
          ch->suspf = true;
        }
      }

      ~SuspendSection() {
        if (susp_flag) {
          Dmac &dma = DMAC[ch->periph.inst_num];
          auto &wb = ch->periph.wbdesc[ch->ch_id];

          // If invalid descriptor -> disable (clear pend)
          if (!wb.BTCNT.reg && !wb.DESCADDR.reg) {
            dma.Channel[ch->ch_id].CHCTRLA.bit.ENABLE = 0;
            dma.Channel[ch->ch_id].CHCTRLA.bit.ENABLE = 1;
          }
          // Unsuspend channel
          dma.Channel[ch->ch_id].CHINTFLAG.bit.SUSP = 1;
          dma.Channel[ch->ch_id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
          ch->suspf = false;
        }
      }

      private:
        Channel *ch;
        bool susp_flag = false;
    }; 

    /// Dummy objects
    inline void dummy_cb(const unsigned int &a, const callback_flag_t &b) { }
    inline struct {}dummy_loc;
  

    /// Accessing objects

  //   /// @a FINAL_V1
  //   template<typename T, int uid>
  //   struct static_inst { static inline T inst; };

  //   /// @a FINAL_V1
  //   template<typename T, unsigned int max_inst, long uid_const = -1>
  //   void accessor(const unsigned int &uid_runt) {
  //     static inline std::array<T*, max_inst> rsrc_arr;
      
  //     if (uid_const >= 0 && uid_const < max_inst) {
  //       unsigned int free_index{0};
  //       unsigned int free_index = std::distance(rsrc_arr.begin(), 
  //           std::find(rsrc_arr.begin(), rsrc_arr.end(), nullptr));
        
  //       if (free_index < rsrc_arr.size()) { // Instantiate new type @ index
  //         static_inst<T, uid_const> new_inst;
  //         rsrc_arr.at(free_index) = std::addressof(new_inst);
  //       } else {
  //         return rsrc_arr.at(uid_const);
  //       }
  //     } else if (uid_runt >= 0 && uid_runt < max_inst) {
  //       return rsrc_arr[uid_runt];
  //     }
  //   }
  // }

  }




  /// @a FINAL_V2
  struct TransferDescriptor
  {
    
    /// @a FINAL_V2
    TransferDescriptor() = default;

    /// @a FINAL_V2
    TransferDescriptor(const TransferDescriptor &other) {
      this->operator = (other);
    }

    /// @a FINAL_V2 -> NEEDS TEST/LOOKOVER
    TransferDescriptor(TransferDescriptor &&other) {
      if (&other != this) [[likely]] {
        auto crit = detail::SuspendSection(assig_ch);

        // Get other's descriptor or ptr (if base only)
        if (other.descPtr != &other.desc) {
          descPtr = std::exchange(other.descPtr, &other.desc);
        } else {
          memcpy(&desc, &other.desc, detail::dsize);
        }
        // Relink prev/writeback if other is linked
        if (other.assig_ch) {
          TransferDescriptor *prev = other.assig_ch->btd;
          do {
            if (prev->next == &other) {
              auto l_addr = reinterpret_cast<uintptr_t>(descPtr); 
              if (assig_ch->periph.wbdesc[other.assig_ch->ch_id]
                  .DESCADDR.reg == prev->descPtr->DESCADDR.reg) {
                assig_ch->periph.wbdesc[other.assig_ch->ch_id].DESCADDR.reg = l_addr;
              }
              prev->descPtr->DESCADDR.reg = l_addr;
              prev->next = this;
            }
          } while(prev && prev != other.assig_ch->btd);
          if (descPtr != &desc) {
            other.assig_ch->btd = this;
          }
        } // Exchange fields
        next = std::exchange(other.next, nullptr);
        assig_ch = std::exchange(other.assig_ch, nullptr);
      }
    }

    /// @a FINAL_V2
    template<valid_loc_t src_t = detail::dumm_t, 
             valid_loc_t dst_t = detail::dumm_t>
    struct Config 
    {
      const uint32_t beatsize   = -1;                 // > numeric
      const src_t *src          = &detail::dummy_loc; // > ptr
      const int32_t srcinc      = -1;                 // > numeric
      const dst_t *dst          = &detail::dummy_loc; // > ptr
      const int32_t dstinc      = -1;                 // > numeric
      const blockact_t blockact = blockact_t::null;   // > enum (blockact_t)
    };

    /// @a FINAL_V2
    template<typename src_t, typename dst_t, Config &cfg>
    void set_config(const Config<src_t, dst_t> &_cfg_) {
      if (&cfg != &_cfg_) { return; } /// #WARN ? 
      using namespace ref;

      descPtr->BTCTRL.bit.VALID = 0;
      auto prev_bs = beatsize_map[descPtr->BTCTRL.bit.BEATSIZE];
      uintptr_t prev_srcaddr{0}, prev_dstaddr{0};

      // Capture previous, non-modified address if beatsize changed
      if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        if constexpr (std::is_same_v<src_t, detail::dumm_t>) {
          prev_srcaddr = descPtr->SRCADDR.reg - detail::addr_mod(*descPtr, true);
        }
        if constexpr (std::is_same_v<dst_t, detail::dumm_t>) {
          prev_dstaddr = descPtr->DSTADDR.reg - detail::addr_mod(*descPtr, false);
        }
      }
      // Compute ctrl register mask
      constexpr auto btctrl_masks = []() consteval {
        constexpr unsigned int max_inc = std::max(cfg.srcinc, cfg.dstinc);
        using ctrl_t = decltype(std::declval<DMAC_BTCTRL_Type>().reg);
        ctrl_t clr_msk{0}, set_msk{0};

        if constexpr (cfg.beatsize != -1) {
          constexpr uint32_t bs_reg = std::distance(beatsize_map.begin(), 
              std::find(beatsize_map.begin(), beatsize_map.end(), cfg.beatsize));  
          static_assert(bs_reg < beatsize_map.size(), 
              "SIO ERROR (DMA): descriptor beatsize invalid.");

          clr_msk |= (DMAC_BTCTRL_BEATSIZE_Msk);
          set_msk |= (bs_reg << DMAC_BTCTRL_BEATSIZE_Pos);
        }
        if constexpr (max_inc > 1) {
          constexpr uint32_t ssize_r = std::distance(stepsize_map.begin(), 
              std::find(stepsize_map.begin(), stepsize_map.end(), max_inc));
          static_assert(ssize_r < stepsize_map.size(), 
              "SIO ERROR (DMA): descriptor stepsize invalid.");

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
          static_assert(cfg.dstinc <= 1, "SIO ERROR (DMA): descriptor" + 
           " increment cfg invalid."); 
          clr_msk |= (DMAC_BTCTRL_STEPSEL);
          set_msk |= (DMAC_BTCTRL_STEPSEL_SRC);
        }
        if constexpr (cfg.dstinc > 1) {
          static_assert(cfg.srcinc <= 1, "SIO ERROR (DMA): descriptor" +
              " increment cfg invalid.");
          clr_msk |= (DMAC_BTCTRL_STEPSEL);
          set_msk |= (DMAC_BTCTRL_STEPSEL_DST);
        }
        if constexpr (cfg.blockact != blockact_t::null) {
          clr_msk |= (DMAC_BTCTRL_BLOCKACT_Msk);
          set_msk |= (static_cast<ctrl_t>(cfg.blockact) << DMAC_BTCTRL_BLOCKACT_Pos);
        }
        return std::pair(clr_msk, set_msk); 
      }();
      descPtr->BTCTRL.reg &= ~btctrl_masks.first; 
      descPtr->BTCTRL.reg |= btctrl_masks.second;

      // Set source address or re-calc previous if beatsize changed
      if constexpr (!std::is_same_v<src_t, detail::dumm_t>) {
        descPtr->SRCADDR.reg = cfg.src ? static_cast<uintptr_t>((void*)cfg.src)
            + detail::addr_mod(*descPtr, true) : 0;
      } else if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        descPtr->SRCADDR.reg = prev_srcaddr + detail::addr_mod(*descPtr, true);
      }
      // Set destination address or re-calc prev if beatsize changedd
      if constexpr (!std::is_same_v<dst_t, detail::dumm_t>) {
        descPtr->DSTADDR.reg = cfg.dst ? static_cast<uintptr_t>((void*)cfg.dst)
            + detail::addr_mod(*descPtr, false) : 0;
      } else if (cfg.beatsize != -1 && prev_bs != cfg.beatsize) {
        descPtr->DSTADDR.reg = prev_dstaddr + detail::addr_mod(*descPtr, false);
      }
      detail::update_valid(*descPtr);
    }

    /// @a FINAL_V2
    void set_config(const TransferDescriptor &other) {
      if (&other == this) [[unlikely]] { return; }
      descPtr->BTCTRL.bit.VALID = 0;
      auto prev_link = descPtr->DESCADDR.reg;
      
      // Copy descriptor & reset link (to prev)
      memcpy(descPtr, other.descPtr, detail::dsize);
      descPtr->DESCADDR.reg == prev_link;
      detail::update_valid(*descPtr);
      detail::update_valid(*other.descPtr);
    }

    /// @a FINAL_V2
    TransferDescriptor &operator = (const TransferDescriptor &other) {
      set_config(other);
      return *this;
    }

    /// @a FINAL_V2
    void swap(TransferDescriptor &other, const bool &swap_descriptors) {
      auto t_crit = detail::SuspendSection(assig_ch);
      auto o_crit = detail::SuspendSection(other.assig_ch);

      if (swap_descriptors) { // Swap descriptors or ptrs (if base only)
        DmacDescriptor *this_ptr{descPtr}; 
        DmacDescriptor this_desc{};
        memcpy(&this_desc, descPtr, detail::dsize);

        if (other.descPtr != &other.desc) {
          descPtr = other.descPtr;
        } else {
          memcpy(&desc, &other.desc, detail::dsize);
          descPtr = &desc;
        }
        if (this_ptr != &desc) {
          other.descPtr = this_ptr;
        } else {
          memcpy(&other.desc, &this_desc, detail::dsize);
          other.descPtr = &other.desc;
        }
      } else { // If this/other = base -> move this/other into & out of base mem
        if (other.descPtr != &other.desc && descPtr != &desc) {
          DmacDescriptor this_temp{};
          memcpy(&this_temp, descPtr, detail::dsize);
          memcpy(descPtr, other.descPtr, detail::dsize);
          memcpy(other.descPtr, &this_temp, detail::dsize);

        } else if (other.descPtr != &other.desc) {
          memcpy(&other.desc, other.descPtr, detail::dsize);
          memcpy(&other.descPtr, &desc, detail::dsize);
          descPtr = std::exchange(other.descPtr, &other.desc);
        
        } else if (descPtr != &desc) {
          memcpy(&desc, descPtr, detail::dsize);
          memcpy(descPtr, &other.desc, detail::dsize);
          other.descPtr = std::exchange(descPtr, &desc);
        }
        std::swap(descPtr->DESCADDR.reg, other.descPtr->DESCADDR.reg);
      }
      // Relink other's prev -> this & this prev -> other
      auto relink_prev = [&](TransferDescriptor &old, TransferDescriptor &targ) {
        if (old.assig_ch) {
          auto &periph = old.assig_ch->periph;
          auto &wb = periph.wbdesc[targ.assig_ch->ch_id];
          TransferDescriptor *prev = old.assig_ch->btd;
          do {
            if (prev->next == &old) {
              auto l_addr = reinterpret_cast<uintptr_t>(targ.descPtr);
              if (wb.DESCADDR.reg == prev->descPtr->DESCADDR.reg && wb.SRCADDR.reg) {
                wb.DESCADDR.reg = l_addr;
              }
              prev->descPtr->DESCADDR.reg = l_addr;
              prev->next = &targ;
            }
            prev = prev->next;
          } while(prev && prev != old.assig_ch->btd);
        }
      };
      relink_prev(other, *this);
      relink_prev(*this, other);

      // Reassign base td & swap fields
      if (descPtr != &desc) { other.assig_ch->btd = this; }
      if (other.descPtr != &other.desc) { assig_ch->btd = &other; }
      std::swap(assig_ch, other.assig_ch);
      std::swap(next, other.next);
      detail::update_valid(*descPtr);
      detail::update_valid(*other.descPtr);
    }

    /// @a FINAL_V2
    TransferDescriptor &operator = (TransferDescriptor &&other) {
      swap(other, true);
      return *this;
    }

    /// @a FINAL_V2
    void set_length(const uint32_t &length, const bool &in_bytes) {
      descPtr->BTCTRL.bit.VALID = 0;
      uintptr_t s_addr = descPtr->SRCADDR.reg - detail::addr_mod(*descPtr, true);
      uintptr_t d_addr = descPtr->DSTADDR.reg - detail::addr_mod(*descPtr, false);

      unsigned int new_cnt = length;
      if (in_bytes && length) {
        new_cnt /= ref::beatsize_map[descPtr->BTCTRL.bit.BEATSIZE];
      }
      // Set new beatcount & re-calc src/dst (mod changes w/ cnt)
      descPtr->BTCNT.reg = new_cnt;
      descPtr->SRCADDR.reg = s_addr + detail::addr_mod(*descPtr, true);
      descPtr->DSTADDR.reg = d_addr + detail::addr_mod(*descPtr, false);
      detail::update_valid(*descPtr);
    }

    /// @a FINAL_V2
    unsigned int length(const bool &in_bytes) const {
      auto curr_len = descPtr->BTCNT.reg;
      if (in_bytes) {
        auto bs_reg = descPtr->BTCTRL.bit.BEATSIZE;
        curr_len /= ref::beatsize_map[bs_reg];
      }
      return curr_len;
    }

    /// @a FINAL_V2
    bool valid() const {
      detail::update_valid(*descPtr);
      return descPtr->BTCTRL.bit.VALID;
    }

    /// @a FINAL_V2
    operator bool() const {
      return valid();
    }

    /// @a FINAL_V2
    inline Channel *assigned_channel() const {
      return assig_ch;
    }

    /// @a FINAL_V2
    inline TransferDescriptor *linked_descriptor() const {
      return next;
    }

    /// @a FINAL_V2
    void unlink() { 
      if (assig_ch) {
        assig_ch->descriptor_list.remove(*this);
      }
    }

    DmacDescriptor desc{};
    DmacDescriptor *descPtr{nullptr};
    Channel *assig_ch{nullptr};
    TransferDescriptor *next{nullptr};
  };

  /// @a FINAL_V1
  template<typename T>
  concept valid_loc_t = requires {
    std::is_trivially_copyable_v<std::conditional_t
        <std::is_same_v<T, detail::dummy_t>, int, T>>;
    !std::is_void_v<T>;
    !std::is_pointer_v<T>;
  };






  /// @a TODO
  struct Peripheral
  {
    
    /// @a FINAL_V2
    Peripheral(const unsigned int &inst_num) 
      : inst_num(inst_num) {}

    /// @a TODO
    Peripheral(const Peripheral&) = delete;
    Peripheral(Peripheral&&) = delete;
    ~Peripheral() = delete;

    /// @a FINAL_V2
    void init() {
      if (is_init()) { return; }
      exit();
      // Enable all irq interrupts
      int irq_off = ref::irq_array.size() * inst_num;
      for (int i = 0; i < ref::irq_array.size(); i++) {
        NVIC_EnableIRQ(static_cast<IRQn_Type>(irq_off + ref::irq_array[i]));
      }
      // Set base/writeback mem sections & enable dma/clock
      DMAC[inst_num].BASEADDR.reg = reinterpret_cast<uintptr_t>(bdesc);
      DMAC[inst_num].WRBADDR.reg = reinterpret_cast<uintptr_t>(wbdesc);
      MCLK->AHBMASK.reg |= (1 << (DMAC_CLK_AHB_ID + inst_num));
      DMAC[inst_num].CTRL.bit.DMAENABLE = 1;
    }

    /// @a FINAL_V2
    void exit() {
      if (!is_init()) { return; }
      // Disable dma/ahbbus & reset all registers
      DMAC[inst_num].CTRL.bit.DMAENABLE = 0;
      while(DMAC[inst_num].CTRL.bit.DMAENABLE);
      DMAC[inst_num].CTRL.bit.SWRST = 1;
      while(DMAC[inst_num].CTRL.bit.SWRST);
      MCLK->AHBMASK.reg &= ~(1 << (MCLK_AHBMASK_DMAC_Pos + inst_num));

      // Disable & reset all irq interrupts
      int irq_off = ref::irq_array.size() * inst_num;
      for (int i = 0; i < ref::irq_array.size(); i++) {
        auto curr_irq = static_cast<IRQn_Type>(ref::irq_array[i] + irq_off);
        NVIC_DisableIRQ(curr_irq);
        NVIC_ClearPendingIRQ(curr_irq);
        NVIC_SetPriority(curr_irq, ref::def_irq_prio);
      }
      // Reset resources
      memset(const_cast<DmacDescriptor*>(wbdesc), 0U, sizeof(wbdesc));
      memset(bdesc, 0U, sizeof(bdesc));

      /// @todo @a FIGURE_OUT_HOW_TO_CLEAR_CHANNELS...
    }

    /// @a FINAL_V2
    bool is_init() const {
      auto b_addr = reinterpret_cast<uintptr_t>(bdesc); 
      return DMAC[inst_num].BASEADDR.reg && DMAC[inst_num].CTRL.bit.DMAENABLE;
    }

    /// @a FINAL_V2
    operator bool() const { 
      return is_init(); 
    }

    /// @a FINAL_V2
    struct Config
    {
      const std::array<int32_t, DMAC_LVL_NUM> prilvl_en{-1, -1, -1, -1};             // Priority lvl enabled
      const std::array<int32_t, DMAC_LVL_NUM> prilvl_rr{-1, -1, -1, -1};             // Priority lvl round robin arbitration
      const std::array<int32_t, DMAC_LVL_NUM> prilvl_squal{-1, -1, -1, -1};          // Priority lvl service quality
      const std::array<int32_t, ref::irq_array.size()> irq_prio{-1, -1, -1, -1, -1}; // Irq priority lvl
    };

    /// @a FINAL_V2
    template<Config &cfg>
    void set_config() {
      using namespace ref;
      if (is_init()) [[unlikely]] { return; }

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
      uint32_t irq_off = irq_array.size() * inst_num;
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
      DMAC[inst_num].PRICTRL0.reg &= ~clr_prictrl;
      DMAC[inst_num].PRICTRL0.reg |= set_prictrl;
      DMAC[inst_num].CTRL.reg &= ~clr_ctrl;
      DMAC[inst_num].CTRL.reg |= set_ctrl;
    };

    /// @a FINAL_V2
    void set_config(const Peripheral &other) {
      if (&other == this) [[unlikely]] { return; }

      // Copy config from registers
      DMAC[inst_num].CTRL.vec.LVLEN = DMAC[other.inst_num].CTRL.vec.LVLEN;
      DMAC[inst_num].PRICTRL0.reg = DMAC[other.inst_num].PRICTRL0.reg;

      // Copy/set irq priority level config
      int t_irq_off = ref::irq_array.size() * inst_num;
      int o_irq_off = ref::irq_array.size() * other.inst_num;

      for (int i = 0; i < ref::irq_array.size(); i++) {
        auto other_prio = NVIC_GetPriority(static_cast<IRQn_Type>(ref::irq_array[i] + o_irq_off));
        NVIC_SetPriority(static_cast<IRQn_Type>(ref::irq_array[i] + t_irq_off), other_prio);
      }
    }

    /// @a FINAL_V2
    Peripheral &operator = (const Peripheral &other) {
      set_config(other);
    }

    /// @a TODO
    void active_channel() {

    }

    /// @a TODO
    void pending_channel_count() {

    }


    /// FINAL_V2
    struct Channel 
    {

      /// @a FINAL_V2
      Channel(Peripheral &periph, const unsigned int &ch_id)
        : periph(periph), ch_id(ch_id) {}

      /// @a FINAL_V2
      Channel(const Channel&) = delete;
      Channel(Channel&&) = delete;
      ~Channel() = delete;

      /// @a FINAL_V2
      struct TransferConfig
      {
        const int32_t burstlen   = -1; // > Numeric 
        const trigsrc_t trigsrc  = trigsrc_t::null;
        const trigact_t trigact  = trigact_t::null;
        const int32_t prilvl     = -1; 
        const int32_t threshold  = -1; // > Numeric
        const int32_t runstdby   = -1; // > Boolean cond
      };

      /// @a FINAL_V2
      template<TransferConfig &cfg>
      void set_transfer_config() {
        using namespace ref;

        // Record enable status & disable channel
        bool prev_en = DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE;
        DMAC[periph.inst_num].Channel[id].CHCTRLA.bit.ENABLE = 0;
        while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE);

        // Compute masks for config in ctrl register
        constexpr auto chctrl_masks = [&]() consteval {
          using ctrl_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
          ctrl_t clr_msk{0}, set_msk{0};

          if constexpr (cfg.burstlen != -1) {
            constexpr uint32_t bl_reg = std::distance(burstlen_map.begin(),
                std::find(burstlen_map.begin(), burstlen_map.end(), cfg.burstlen));
            static_assert(bl_reg < burstlen_map.size(), 
                "SIO ERROR (DMA): channel burstlen config invalid.");

            clr_msk |= (DMAC_CHCTRLA_BURSTLEN_Msk);
            set_msk |= (bl_reg << DMAC_CHCTRLA_BURSTLEN_Pos);
          }
          if constexpr (cfg.threshold != -1) {
            constexpr uint32_t th_reg = std::distance(threshold_map.begin(),
                std::find(threshold_map.begin(), threshold_map.end(), cfg.threshold));
            static_assert(th_reg < threshold_map.size(), 
                "SIO ERROR (DMA): channel threshold config invalid.");

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
                "SIO ERROR (DMA): channel runstdby config invalid");
            clr_msk |= (DMAC_CHCTRLA_RUNSTDBY);
            set_msk |= (static_cast<bool>(cfg.runstdby) << DMAC_CHCTRLA_RUNSTDBY_Pos);
          }
          return std::make_pair(clr_msk, set_msk);
        }(); 
        DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.reg &= ~chctrl_masks.first; 
        DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.reg |= chctrl_masks.second; 

        // Compute masks for config in prictrl register & set them
        constexpr auto chprilvl_masks = [&]() consteval {
          using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
          prictrl_t clr_msk{0}, set_msk{0};

          if constexpr (cfg.prilvl != -1) {
            constexpr uint32_t plvl_reg = std::distance(prilvl_map.begin(),
                std::find(prilvl_map.begin(), prilvl_map.end(), cfg.prilvl));
            static_assert(plvl_reg < prilvl_map.size(), 
                "SIO ERROR (DMA): channel prilvl config invalid.");

            clr_msk |= (DMAC_CHPRILVL_PRILVL_Msk);
            set_msk |= (plvl_reg << DMAC_CHPRILVL_PRILVL_Pos);
          }
          return std::make_pair(clr_msk, set_msk);
        }();
        DMAC[periph.inst_num].Channel[ch_id].CHPRILVL.reg &= ~chprilvl_masks.first;
        DMAC[periph.inst_num].Channel[ch_id].CHPRILVL.reg |= chprilvl_masks.second;
        DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = prev_en; // > Re-enable channel 
      }

      /// @a FINAL_V2
      void set_transfer_config(const Channel &other) {
        if (&other != this) {
          // Capture enable state & disable channel
          const bool prev_en = DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE;
          DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = 0;
          while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE);
          
          // Clear and set (with other) -> prilvl & ctrl registers
          auto &o_inst = other.periph.inst_num;
          DMAC[periph.inst_num].Channel[ch_id].CHPRILVL.reg = DMAC[o_inst].Channel[other.ch_id].CHPRILVL.reg;
          DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.reg = (DMAC[o_inst].Channel[other.ch_id].CHCTRLA.reg 
              & ~(DMAC_CHCTRLA_ENABLE | DMAC_CHCTRLA_SWRST));
          DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = prev_en; // > Re-enable channel
        }
      }

      /// @a FINAL_V2
      struct InterruptConfig
      {
        const callback_t cb      = detail::dummy_cb; // > Callable (std::function - callback_t)
        const int32_t susp_int   = -1; // > Boolean cond
        const int32_t err_int    = -1; // > Boolean cond
        const int32_t tcmpl_int  = -1; // > Boolean cond
      };

      /// @a FINAL_V2
      template<InterruptConfig &cfg>
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
        DMAC[periph.inst_num].Channel[ch_id].CHINTENSET.reg &= ~chint_masks.first;
        DMAC[periph.inst_num].Channel[ch_id].CHINTENSET.reg |= chint_masks.second;
        
        // Register channel callback & callback function
        if constexpr (cfg.cb != detail::dummy_cb) { 
          callback = cfg.cb;
        }
      }

      /// @a FINAL_V2
      void set_interrupt_config(const Channel &other, const bool &copy_cb = true) {
        if (&other != this) [[likely]] {
          // Copy interrupt set register & callback
          auto &o_chint = DMAC[other.periph.inst_num].Channel[other.ch_id].CHINTENSET.reg;
          DMAC[periph.inst_num].Channel[ch_id].CHINTENSET.reg |= o_chint;
          if (copy_cb) { callback = other.callback; }
        }
      }

      /// @a FINAL_V2
      Channel &operator = (const Channel &other) {
        if (&other != this) [[likely]] {
          set_transfer_config(other);
          set_interrupt_config(other, true);
        }
        return *this;
      }

      /// @a FINAL_V2
      void reset(const bool &clear_descriptors = true) {
        if (clear_descriptors) [[likely]] { 
          descriptor_list.clear(); 
        }
        DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = 0;     // Disable DMA channel
        while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE);  // Wait for abort
        DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.SWRST = 1;      // Software reset ch registers
        while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.SWRST);   // Wait for sw reset
        
        auto wb_ptr = const_cast<DmacDescriptor*>(&periph.wbdesc[ch_id]);
        auto b_ptr = &periph.bdesc[ch_id];

        // Reset "fields" in common mem
        memset(wb_ptr, 0U, detail::dsize); 
        memset(b_ptr, 0U, detail::dsize);
        callback = nullptr;
        suspf = false;
      }


      /// @a FINAL_V2
      bool set_state(const channel_state_t &targ_state) {
        auto atomic = temp::atomic_section();
        auto init_state = state();
        if (targ_state != init_state) [[likely]] {

          // Disable channel
          if (targ_state == channel_state_t::disabled) [[unlikely]] {
            DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = 0;
            while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE);
          
          } else { // Enable/reset channel (if writeback invalid/end)
            if (!periph.wbdesc[ch_id].BTCNT.reg && !periph.wbdesc[ch_id].DESCADDR.reg && 
                init_state == channel_state_t::suspended) { 
              DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = 0;
              while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE);
            }
            DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE = 1;
          }
          // Suspend channel
          if (targ_state == channel_state_t::suspended) [[unlikely]] {
            DMAC[periph.inst_num].Channel[ch_id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
            while(DMAC[periph.inst_num].Channel[ch_id].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
            suspf = true;

          // Resume channel (if susp ONLY) -> otherwise skip next susp
          } else if (init_state == channel_state_t::suspended) {
            DMAC[periph.inst_num].Channel[ch_id].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
            DMAC[periph.inst_num].Channel[ch_id].CHINTFLAG.bit.SUSP = 1;
            suspf = false;
          }
        }
        return (targ_state == state());
      }

      /// @a FINAL_V2
      inline channel_state_t state() {
        if (!DMAC[periph.inst_num].Channel[ch_id].CHCTRLA.bit.ENABLE) {
          return channel_state_t::disabled;

        } else if (DMAC[periph.inst_num].Channel[ch_id].CHINTFLAG.bit.SUSP || suspf) {
          return channel_state_t::suspended;
        }
        return channel_state_t::enabled;
      }

      /// @a FINAL_V2
      inline void trigger() {
        using reg_t = decltype(std::declval<DMAC_SWTRIGCTRL_Type>().reg);
        reg_t trig_msk = (1 << (ch_id + DMAC_SWTRIGCTRL_SWTRIG0_Pos)); 
        DMAC[periph.inst_num].SWTRIGCTRL.reg |= trig_msk;
      }

      /// @a FINAL_V2
      inline bool trigger_pending() const {
        return DMAC[periph.inst_num].Channel[ch_id].CHSTATUS.bit.PEND;
      }

      /// @a FINAL_V2
      inline bool transfer_busy() const {
        return DMAC[periph.inst_num].Channel[ch_id].CHSTATUS.bit.BUSY &&
            !(DMAC[periph.inst_num].Channel[ch_id].CHINTFLAG.bit.SUSP || suspf);
      }


      /// @a FINAL_V2
      struct DescriptorList_t 
      {

        /// @a FINAL_V2
        void clear() {
          if (_super_.btd) [[likely]] { // Set channel state -> disabled
            DMAC[_super_.periph.inst_num].Channel[_super_.ch_id].CHCTRLA.bit.ENABLE = 0;
            while(DMAC[_super_.periph.inst_num].Channel[_super_.ch_id].CHCTRLA.bit.ENABLE);

            // Return base descriptor to base td's owned memory
            TransferDescriptor *curr = _super_.btd;
            memcpy(&curr->desc, curr->descPtr, detail::dsize);
            curr->descPtr = &curr->desc;

            do { // Iter through & unlink/unassign descritors
              curr->assig_ch = nullptr;
              curr->descPtr->DESCADDR.reg = 0;
              curr = std::exchange(curr->next, nullptr);
            } while(curr && curr != _super_.btd);

            // Clear state variables/writeback & disable channel
            memset((void*)&_super_.periph.wbdesc[_super_.ch_id], 0U, detail::dsize);
            memset(&_super_.periph.bdesc[_super_.ch_id], 0U, detail::dsize);
            _super_.btd = nullptr;
          }
        }

        /// @a FINAL_V2
        template<TransferDescriptor&... tdlist, bool looped>
        void set() {
          if (sizeof...(tdlist)) [[likely]] {
            Dmac &dma = DMAC[_super_.periph.inst_num];
            bool prev_en = dma.Channel[_super_.ch_id].CHCTRLA.bit.ENABLE;
            (tdlist.unlink(), ...);
            clear();
            // get first td & move desc to base memory
            TransferDescriptor *base = ((tdlist, true) || ...);
            memcpy(&_super_.periph.bdesc[_super_.ch_id], &base->desc, detail::dsize);
            base->descPtr = &_super_.periph.bdesc[_super_.ch_id];
            _super_.btd = base;

            // Find initial previous descriptor (if looped -> last)
            static constexpr detail::TdescResource dumm_rsrc{};
            TransferDescriptor *prev{&dumm_rsrc};
            if constexpr (looped) { prev = &(tdlist, ...); }

            // Iterate (fold) through descriptors -> link together
            ((prev->descPtr->DESCADDR.reg = reinterpret_cast
                  <uintptr_t>(tdlist.descPtr),
              tdlist.assig_ch = &_super_,
              prev->next = &tdlist,
              prev = &tdlist
            ), ...);
            // Re-enable channel if disabled
            dma.Channel[ch_id].CHCTRLA.bit.ENABLE = prev_en;
          }
        }

        /// @a FINAL_V2
        bool add(TransferDescriptor &td, const unsigned int &index) {
          auto crit = detail::SuspendSection(&_super_);

          // Get resources & unlink channel
          TransferDescriptor *ori_base = _super_.btd;
          TransferDescriptor *prev{ori_base};
          td.unlink();

          // Set as new base descriptor 
          if (!_super_.btd || !index) {
            if (_super_.btd) {
              memcpy(&_super_.btd->desc, _super_.btd->descPtr, detail::dsize);
              _super_.btd->descPtr = &_super_.btd->desc;

              // Link td to prev base...
              auto l_addr = reinterpret_cast<uintptr_t>(_super_.btd); 
              td.descPtr->DESCADDR.reg = l_addr;
              td.next = _super_.btd;
            }
            // Move td descriptor into base memory section
            auto *base_desc = &_super_.periph.bdesc[_super_.ch_id];
            memcpy(base_desc, &td.desc, detail::dsize);
            td.descPtr = &td.desc;
            _super_.btd = &td;

            // If looped -> Find and relink last descriptor
            while(prev->next && prev->next != ori_base) { prev = prev->next; }
            if (prev->next == ori_base) {
              auto l_addr = reinterpret_cast<uintptr_t>(base_desc);
              prev->descPtr->DESCADDR.reg = l_addr;
              prev->next = &td;
            }
          } else { // Get previous descriptor
            for (int i = 0; i < index; i++) {
              if (!prev->next || prev->next == ori_base) { break; }
              prev = prev->next;
            }
            // Link td -> prev (old) next
            td.descPtr->DESCADDR.reg = prev->descPtr->DESCADDR.reg;
            td.next = prev->next;

            // Link prev -> new td
            auto l_addr = reinterpret_cast<uintptr_t>(td.descPtr);
            prev->descPtr->DESCADDR.reg = l_addr;
            prev->next = &td;
          }
          // Relink writeback descriptor
          auto &wb = _super_.periph.wbdesc[_super_.ch_id];
          if (wb.DESCADDR.reg == td.descPtr->DESCADDR.reg) {
            auto l_addr = reinterpret_cast<uintptr_t>(td.descPtr);
            wb.DESCADDR.reg = l_addr;
          }
          td.assig_ch = &_super_;
          return true;
        }

        /// @a FINAL_V2
        void remove(TransferDescriptor &td) {
          if (td.assig_ch) [[likely]] {
            // If this = base descriptor: move desc to owned mem
            if (td.descPtr != &td.desc) {
              auto *base_desc = &td.assig_ch->periph.bdesc[td.assig_ch->ch_id];
              memcpy(&td.desc, base_desc, detail::dsize);
              td.descPtr = &td.desc;

              if (td.next) { // Move next down into base
                memcpy(base_desc, &td.next->desc, detail::dsize);
                td.next->descPtr = &td.next->desc;
                td.assig_ch->btd = td.next;

                // (If looped) -> link prev (last) -> next
                TransferDescriptor *prev = td.assig_ch->btd;
                while(prev->next && prev->next != td.assig_ch->btd) { prev = prev->next; }
                if (prev->next == &td) {
                  prev->descPtr->DESCADDR.reg = td.descPtr->DESCADDR.reg;
                  prev->next = td.next;
                }
              } else { // Clear base descriptor
                td.assig_ch->btd = nullptr;
                auto *wb_desc = &td.assig_ch->periph.wbdesc[td.assig_ch->ch_id];
                memset(base_desc, 0U, detail::dsize);
                memset((void*)wb_desc, 0U, detail::dsize);

                // Disable DMA
                auto &dma = DMAC[td.assig_ch->periph.inst_num]; 
                dma.Channel[td.assig_ch->ch_id].CHCTRLA.bit.ENABLE = 0;
                while(dma.Channel[td.assig_ch->ch_id].CHCTRLA.bit.ENABLE);
              }
            } else { // Remove from middle of list... link prev -> next
              TransferDescriptor *prev = td.assig_ch->btd;
              while(prev->next != &td) { prev = prev->next; }
              prev->descPtr->DESCADDR.reg = td.descPtr->DESCADDR.reg;
              prev->next = td.next;
            }
            // Update writeback descriptor
            auto *wb = &td.assig_ch->periph.wbdesc[td.assig_ch->ch_id];
            if (wb->DESCADDR.reg == reinterpret_cast<uintptr_t>(td.descPtr)) {
              wb->DESCADDR.reg = td.descPtr->DESCADDR.reg;

              // If writeback now invalid -> disable channel
              if (!wb->DESCADDR.reg && !wb->BTCNT.reg) {
                auto &dma = DMAC[td.assig_ch->periph.inst_num];
                dma.Channel[td.assig_ch->ch_id].CHCTRLA.bit.ENABLE = 0;
                while(dma.Channel[td.assig_ch->ch_id].CHCTRLA.bit.ENABLE);
              }
            }
            td.descPtr->DESCADDR.reg = 0;
            td.assig_ch = nullptr;
            td.next = nullptr;
          }
        }

        /// @a FINAL_V2
        inline TransferDescriptor *get(const unsigned int &index) {
          if (!_super_.btd) [[unlikely]] { return nullptr; }
          TransferDescriptor *curr{_super_.btd};
          for (int i = 0; i < index; i++) {
            if (!curr->next || curr->next == _super_.btd) { return nullptr; }
            curr = curr->next;
          }
          return curr;
        }

        /// @a FINAL_V2
        long index_of(TransferDescriptor &td) const {
          if (_super_.btd) [[likely]] {
            TransferDescriptor *curr{_super_.btd};
            unsigned int curr_index{0};
            do {
              if (curr->next == &td) { return curr_index; }
              curr = curr->next; 
              curr_index++;
            } while(curr && curr != _super_.btd);
          }
          return -1;
        }

        /// @a FINAL_V2
        inline unsigned int size() const {
          unsigned int curr_index{0};
          if (_super_.btd) [[likely]] {
            TransferDescriptor *curr{_super_.btd};
            curr_index++;
            do { 
              curr = curr->next;
              curr_index++;
            } while(curr && curr != _super_.btd);
          }
          return curr_index;
        }

        Channel &_super_;
      }descriptor_list{*this};


      /// FINAL_V2
      struct {

        /// @a FINAL_V2
        void sync(const bool &sync_in_prog = false, const bool &keep_length = false) {
          auto crit = detail::SuspendSection(&_super_);
          Dmac &dma = DMAC[_super_.periph.inst_num];
          auto &wb = _super_.periph.wbdesc[_super_.ch_id];

          // Require descriptor list & not in progress (conditional)
          if (_super_.btd && wb.SRCADDR.reg && (sync_in_prog || 
              !dma.Channel[_super_.ch_id].CHSTATUS.bit.BUSY)) {
            TransferDescriptor *curr = _super_.btd;

            // Find td corresponding to writeback descriptor
            while(curr->descPtr->DESCADDR.reg != wb.DESCADDR.reg) {
              if (!curr->next || curr->next == _super_.btd) { return; }
              curr = curr->next;
            }
            // Update writeback
            if (keep_length) {
              auto prev_cnt = wb.BTCNT.reg;
              memcpy((void*)&wb, curr->descPtr, detail::dsize);
              wb.BTCNT.reg = prev_cnt;
            } else {
              memcpy((void*)&wb, curr->descPtr, detail::dsize);
            }
            // Disable channel if wbdesc made invalid
            if (!wb.DESCADDR.reg && !wb.BTCNT.reg) {
              dma.Channel[_super_.ch_id].CHCTRLA.bit.ENABLE = 0;
              while(dma.Channel[_super_.ch_id].CHCTRLA.bit.ENABLE);
            }
          }
        }

        /// @a FINAL_V2
        bool set(const unsigned int &td_index) {
          if (!_super_.btd) { return false; }
          auto crit = detail::SuspendSection(&_super_);
          TransferDescriptor *curr = _super_.btd;

          // Get descriptor at index
          for (int i = 0; i < td_index; i++) {
            if (!curr->next || curr->next == _super_.btd) { return false; }
            curr = curr->next;
          } 
          // Copy descriptor @ index into writeback descriptor
          auto *wb = &_super_.periph.wbdesc[_super_.ch_id];
          memcpy((void*)wb, curr->descPtr, detail::dsize);
        }

        /// @a FINAL_V2
        bool set(TransferDescriptor &td, const bool &keep_link = true) {
          if (!_super_.btd) { return false; }
          auto crit = detail::SuspendSection(&_super_);
          auto &wb = _super_.periph.wbdesc[_super_.ch_id];
          auto prev_link = wb.DESCADDR.reg;

          // Copy specified td descriptor into writeback mem
          memcpy((void*)&wb, td.descPtr, detail::dsize);
          wb.DESCADDR.reg = keep_link ? prev_link : 0;

          // Disable channel if writeback made invalid
          if (!wb.BTCNT.reg && !wb.DESCADDR.reg) {
            Dmac &dma = DMAC[_super_.periph.inst_num];
            dma.Channel[_super_.ch_id].CHCTRLA.bit.ENABLE = 0;
            while(dma.Channel[_super_.ch_id].CHCTRLA.bit.ENABLE);
          }
        }

        /// @a FINAL_V2
        TransferDescriptor *source_td() const {
          auto crit = detail::SuspendSection(&_super_);
          auto &wb = _super_.periph.wbdesc[_super_.ch_id];
          if (!_super_.btd || !wb.SRCADDR.reg) { return nullptr; }

          // Iterate through until iter link = wb link
          TransferDescriptor *curr = _super_.btd;
          while(curr->descPtr->DESCADDR.reg != wb.DESCADDR.reg) {
            if (!curr->next || curr->next == _super_.btd) { return nullptr; }
            curr = curr->next;
          }
          return curr;
        }

        /// @a FINAL_V2
        long index() const {
          auto crit = detail::SuspendSection(&_super_);
          auto &wb = _super_.periph.wbdesc[_super_.ch_id];
          
          // Iterate through until writeback link = td @ index link
          if (!_super_.btd || !wb.SRCADDR.reg) {
            TransferDescriptor *curr = _super_.btd;
            for (int i = 0; i < ref::max_td_inst; i++) {
              if (curr->descPtr->DESCADDR.reg == wb.DESCADDR.reg) { return i; }
              curr = curr->next;
            }
          }
          return -1;
        }

        /// @a FINAL_V2
        bool set_length(const length_t &length, const bool &in_bytes) {
          auto crit = detail::SuspendSection(&_super_);
          auto &wb = const_cast<DmacDescriptor&>(_super_.periph.wbdesc[_super_.ch_id]);
          if (!_super_.btd || !wb.SRCADDR.reg) { return false; }

          // Save initial (non mod) addresses -> must update with new mod 
          uintptr_t s_addr = wb.SRCADDR.reg - detail::addr_mod(wb, true);
          uintptr_t d_addr = wb.DSTADDR.reg - detail::addr_mod(wb, false);

          // Update btcnt reg & src/dst addr
          unsigned int new_cnt = length;
          if (in_bytes && length) {
            new_cnt /= ref::beatsize_map[wb.BTCTRL.bit.BEATSIZE];
          }
          wb.BTCNT.reg = new_cnt;
          wb.SRCADDR.reg = s_addr + detail::addr_mod(wb, true);
          wb.DSTADDR.reg = d_addr + detail::addr_mod(wb, false);
        }

        /// @a FINAL_V2
        length_t length(const bool &in_bytes) const {
          auto crit = detail::SuspendSection(&_super_);
          auto &wb = _super_.periph.wbdesc[_super_.ch_id];
          if (!_super_.btd || !wb.SRCADDR.reg) { return 0; }
      
          // Find length & augment by beatsize (if in bytes)
          unsigned int length = wb.BTCNT.reg;
          if (in_bytes) { 
            length *= ref::beatsize_map[wb.BTCTRL.bit.BEATSIZE];
          } 
          return length;
        }

        Channel &_super_;
      }active_transfer{*this};


      private:
        const unsigned int ch_id;
        Peripheral &periph;
        volatile bool suspf{false};
        callback_t callback{nullptr};
        TransferDescriptor *btd{nullptr};
    };


    private:
      const unsigned int inst_num;
      DmacDescriptor bdesc[ref::ch_num]{};
      volatile DmacDescriptor wbdesc[ref::ch_num]{};
      std::array<Channel*, ref::ch_num> ch_arr{};
  };



 






  /// @b TODO -> ACCESS CHANNEL
  template<uint32_t inst>
    requires (inst < DMAC_INST_NUM)
  void inst_irq_handler() {
    auto id = DMAC[inst].INTPEND.bit.ID;
    
    if (id < ref::max_ch) {
      callback_flag_t cbf = callback_flag_t::null;
      
      // Suspend cmd interrupt
      if (DMAC[inst].INTPEND.bit.SUSP) {
        DMAC[inst].INTPEND.bit.SUSP = 1;
        // if (!comm.suspf[id]) {
        //   comm.suspf[id] = true;
        //   cbf = callback_flag_t::susp;
        // }
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

    }







} // End of sioc::dma namespace
