
#include <sioc_dma_defs.h>
#include <sioc_dma_ref.h>
#include <sioc_util.h>
#include <sio_error.h>

#include <sam.h>
#include <utility>
#include <algorithm>
#include <iterator>
#include <array>
#include <tuple>
#include <string.h>
#include <bit>

namespace sioc::dma {

  namespace detail {

    typedef struct {
      DMAC_BTCTRL_Type    BTCTRL;
      DMAC_BTCNT_Type     BTCNT;
      DMAC_SRCADDR_Type   SRCADDR;
      DMAC_DSTADDR_Type   DSTADDR;
      DMAC_DESCADDR_Type  DESCADDR;
    } DmacDescriptor_NV;

    constexpr inline DmacDescriptor_NV bdesc_arr[ref::ch_num];
    const inline uintptr_t bdesc_addr = (uintptr_t)&bdesc_arr;

    // FINAL_V1
    DmacDescriptor *&get_wbdesc_arr(const int &inst_index) {
      reinterpret_cast<DmacDescriptor*>(DMAC[inst_index].WRBADDR.reg);
    }

  }

  namespace channel {

    // // FINAL_V1
    // struct Config {
    //   int burstlen   = -1; // TODO -> add descriptions... 
    //   trigsrc_t trigsrc  = trigsrc_t::null; // Enum (trigsrc_t)
    //   trigact_t trigact  = trigact_t::null; // Enum (trigact_t)
    //   int prilvl     = -1; // Numeric - (constrained)
    //   int threshold  = -1; // Numeric - (constrained)
    //   int runstdby   = -1; // Bool
    // };
    
    // // FINAL_V1
    // template<Config cfg>
    // void set_transfer_config(const int &inst_index, const int &ch_index) {
    //   using namespace ref;

    //   // Record enable status & disable channel
    //   const bool prev_en = (DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE == 1);
    //   DMAC[inst_index].Channel[id].CHCTRLA.bit.ENABLE = 0;
    //   while(DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE == 1);

    //   // Compute masks for config in ctrl register
    //   constexpr auto chctrl_masks = [&]() consteval {
    //     using ctrl_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
    //     ctrl_t clr_msk{0}, set_msk{0};

    //     if constexpr (cfg.burstlen != -1) {
    //       constexpr uint32_t bl_reg = std::distance(burstlen_map.begin(),
    //           std::find(burstlen_map.begin(), burstlen_map.end(), cfg.burstlen));
    //       static_assert(bl_reg < burstlen_map.size(), 
    //           "SIO ERROR (DMA): channel burstlen config invalid.");

    //       clr_msk |= (DMAC_CHCTRLA_BURSTLEN_Msk);
    //       set_msk |= (bl_reg << DMAC_CHCTRLA_BURSTLEN_Pos);
    //     }
    //     if constexpr (cfg.threshold != -1) {
    //       constexpr uint32_t th_reg = std::distance(threshold_map.begin(),
    //           std::find(threshold_map.begin(), threshold_map.end(), cfg.threshold));
    //       static_assert(th_reg < threshold_map.size(), 
    //           "SIO ERROR (DMA): channel threshold config invalid.");

    //       clr_msk |= (DMAC_CHCTRLA_THRESHOLD_Msk);
    //       set_msk |= (th_reg << DMAC_CHCTRLA_THRESHOLD_Pos);
    //     }
    //     if constexpr (cfg.trigsrc != trigsrc_t::null) {
    //       clr_msk |= (DMAC_CHCTRLA_TRIGSRC_Msk);
    //       set_msk |= (static_cast<uint32_t>(cfg.trigsrc) << DMAC_CHCTRLA_TRIGSRC_Pos);
    //     }
    //     if constexpr (cfg.trigact != trigact_t::null) {
    //       clr_msk |= (DMAC_CHCTRLA_TRIGACT_Msk);
    //       set_msk |= (static_cast<uint32_t>(cfg.trigact) << DMAC_CHCTRLA_TRIGACT_Pos);
    //     }
    //     if constexpr (cfg.runstdby != -1) {
    //       static_assert(cfg.runstdby == 0 || cfg.runstdby == 1, 
    //           "SIO ERROR (DMA): channel runstdby config invalid");
    //       clr_msk |= (DMAC_CHCTRLA_RUNSTDBY);
    //       set_msk |= (static_cast<bool>(cfg.runstdby) << DMAC_CHCTRLA_RUNSTDBY_Pos);
    //     }
    //     return std::make_pair(clr_msk, set_msk);
    //   }(); 
    //   DMAC[inst_index].Channel[ch_index].CHCTRLA.reg &= ~chctrl_masks.first; 
    //   DMAC[inst_index].Channel[ch_index].CHCTRLA.reg |= chctrl_masks.second; 

    //   // Compute masks for config in prictrl register & set them
    //   constexpr auto chprilvl_masks = [&]() consteval {
    //     using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
    //     prictrl_t clr_msk{0}, set_msk{0};

    //     if constexpr (cfg.prilvl != -1) {
    //       constexpr uint32_t plvl_reg = std::distance(prilvl_map.begin(),
    //           std::find(prilvl_map.begin(), prilvl_map.end(), cfg.prilvl));
    //       static_assert(plvl_reg < prilvl_map.size(), 
    //           "SIO ERROR (DMA): channel prilvl config invalid.");

    //       clr_msk |= (DMAC_CHPRILVL_PRILVL_Msk);
    //       set_msk |= (plvl_reg << DMAC_CHPRILVL_PRILVL_Pos);
    //     }
    //     return std::make_pair(clr_msk, set_msk);
    //   }();
    //   DMAC[inst_index].Channel[ch_index].CHPRILVL.reg &= ~chprilvl_masks.first;
    //   DMAC[inst_index].Channel[ch_index].CHPRILVL.reg |= chprilvl_masks.second;
    //   DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = prev_en; // Re-enable channel 
    // }

    // // FINAL_V1
    // constexpr bool valid_index(const int &inst_index, const int &ch_index) {
    //   return periph::valid_index(inst_index) && ch_index >= 0 && ch_index < ref::ch_num;
    // }

    // // FINAL_V1
    // template<channel_state_t targ_state>
    // void set_state(const int &inst_index, const int &ch_index) {
    //   const auto atom = util::atomic_section();
    //   const auto init_state = state(inst_index, ch_index);
    //   auto wb = (detail::get_wbdesc_arr(inst_index) + ch_index);

    //   // Enable dma channel
    //   if constexpr (targ_state == channel_state_t::enabled) {
    //     if (wb->BTCNT.reg == 0 && wb->DESCADDR.reg == 0 && init_state == 
    //         channel_state_t::suspended) 
    //     { 
    //       // If invalid active desc & suspend - disable ch to prevent fetch error... 
    //       DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = 0;
    //       while(DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE == 1);
    //     }
    //     DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = 1;

    //   } else {  // Disable channel
    //     DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = 0;
    //     while(DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE == 1);
    //   }
    //   // Suspend channel
    //   if constexpr (targ_state == channel_state_t::suspended) {
    //     DMAC[inst_index].Channel[ch_index].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
    //     while(DMAC[inst_index].Channel[ch_index].CHCTRLB.bit.CMD != DMAC_CHCTRLB_CMD_SUSPEND_Val);

    //   // Resume channel if suspended
    //   } else if (init_state == channel_state_t::suspended) { 
    //     DMAC[inst_index].Channel[ch_index].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
    //     DMAC[inst_index].Channel[ch_index].CHINTFLAG.bit.SUSP = 1;
    //     DMAC[inst_index].Channel[ch_index].CHINTENSET.bit.SUSP = 1;
    //   }
    // }

    // // FINAL_V1
    // channel_state_t state(const int &inst_index, const int &ch_index) {
    //   if (DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE == 0) {
    //     return channel_state_t::disabled;
    //   } else if (DMAC[inst_index].Channel[ch_index].CHINTFLAG.bit.SUSP == 1) {
    //     return channel_state_t::enabled;
    //   } else {
    //     return channel_state_t::enabled;
    //   }
    // } 

    // // FINAL_V1
    // void trigger_channel(const int &inst_index, const int &ch_index) {
    //   DMAC[inst_index].SWTRIGCTRL.reg |= (1 << (ch_index + DMAC_SWTRIGCTRL_SWTRIG0_Pos));
    // }

    // // FINAL_V1
    // bool transfer_busy(const int &inst_index, const int &ch_index) {
    //   return DMAC[inst_index].Channel[ch_index].CHSTATUS.bit.BUSY == 1 &&
    //       DMAC[inst_index].Channel[ch_index].CHINTFLAG.bit.SUSP == 0;
    // }

    // // FINAL_V1
    // bool trigger_pending(const int &inst_index, const int &ch_index) {
    //   return DMAC[inst_index].Channel[ch_index].CHSTATUS.bit.PEND == 1;
    // }

  }



  // namespace periph {

  //   // void init(const int &inst_index) {
  //   //   if (is_init(inst_index)) { return; }
  //   //   exit(inst_index);
  //   //   // Enable all irq interrupts
  //   //   int irq_off = ref::irq_array.size() * inst_index;
  //   //   for (int i = 0; i < ref::irq_array.size(); i++) {
  //   //     NVIC_EnableIRQ(static_cast<IRQn_Type>(irq_off + ref::irq_array[i]));
  //   //   }
  //   //   // Set base/writeback mem sections & enable dma/clock
  //   //   DmacDescriptor *wb = new DmacDescriptor[ref::ch_num]{};
  //   //   DMAC[inst_index].BASEADDR.reg = reinterpret_cast<uintptr_t>(detail::bdesc_arr);
  //   //   DMAC[inst_index].WRBADDR.reg = reinterpret_cast<uintptr_t>();
  //   //   MCLK->AHBMASK.reg |= (1 << (DMAC_CLK_AHB_ID + inst_index));
  //   //   DMAC[inst_index].CTRL.bit.DMAENABLE = 1;
  //   // }



  //   // TODO
  //   struct Config {
  //     const std::array<int, DMAC_LVL_NUM> prilvl_en{-1, -1, -1, -1};             // Channel priority lvl enabled
  //     const std::array<int, DMAC_LVL_NUM> prilvl_rr{-1, -1, -1, -1};             // Round robin arbitration enabled for priority lvl
  //     const std::array<int, DMAC_LVL_NUM> prilvl_squal{-1, -1, -1, -1};          // Transfer service quality (higher = less latency)
  //   };

  //   // TODO
  //   template<Config cfg>
  //   void set_config() {
  //     using namespace ref;
  //     if (is_init()) [[unlikely]] { return; }

  //     // Compute masks for and set priority lvl cfg
  //     constexpr auto prio_masks = []() consteval {
  //       using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
  //       using ctrl_t = decltype(std::declval<DMAC_CTRL_Type>().reg);
  //       return ([]<uint32_t... Is>(std::integer_sequence<uint32_t, Is...>) consteval {
  //         std::tuple<prictrl_t, prictrl_t, ctrl_t, ctrl_t> masks{}, temp_masks{};

  //         // Generate mask for specific priority lvl cfg
  //         constexpr auto prictrl_msk_i = []<uint32_t i>() consteval {
  //           constexpr auto squal_val = cfg.prilvl_squal.at(i);
  //           constexpr auto rr_val = cfg.prilvl_rr.at(i);
  //           constexpr auto en_val = cfg.prilvl_en.at(i);
  //           prictrl_t set_msk_prio{0}, clr_msk_prio{0};
  //           ctrl_t clr_msk_ctrl{0}, set_msk_ctrl{0};

  //           if constexpr (squal_val != -1) {
  //             constexpr auto squal_reg = std::distance(squal_map.begin(), 
  //                 std::find(squal_map.begin(), squal_map.end(), squal_val));
  //             static_assert(squal_reg < squal_map.size(), 
  //                 "SIO ERROR (DMA): prilvl_squal config invalid");
              
  //             uint32_t off = i * (DMAC_PRICTRL0_QOS1_Pos - DMAC_PRICTRL0_QOS0_Pos);
  //             clr_msk_prio |= (DMAC_PRICTRL0_QOS0_Msk << off);
  //             set_msk_prio |= (squal_reg << (off + DMAC_PRICTRL0_QOS0_Pos));
  //           }
  //           if constexpr (rr_val != -1) {
  //             static_assert(rr_val == 0 || rr_val == 1, 
  //                 "SIO ERROR (DMA): prilvl_rr config invalid");
  //             uint32_t off = i * (DMAC_PRICTRL0_RRLVLEN1_Pos - DMAC_PRICTRL0_RRLVLEN0_Pos);
  //             clr_msk_prio |= (1U << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
  //             set_msk_prio |= (static_cast<bool>(rr_val) << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
  //           }
  //           if constexpr (en_val != -1) {
  //             static_assert(en_val == 0 || en_val == 1, 
  //                 "SIO ERROR (DMA): prilvl_en config invalid");
  //             uint32_t off = i * (DMAC_CTRL_LVLEN1_Pos - DMAC_CTRL_LVLEN0_Pos);
  //             clr_msk_ctrl |= (1U << (off + DMAC_CTRL_LVLEN0_Pos));
  //             set_msk_ctrl |= (static_cast<bool>(en_val) << (off + DMAC_CTRL_LVLEN0_Pos));
  //           }
  //           return std::make_tuple(clr_msk_prio, set_msk_prio, clr_msk_ctrl, set_msk_ctrl);
  //         };
  //         // Iterate over priority lvls & generate masks
  //         return ((temp_masks = prictrl_msk_i.template operator()<Is>(),
  //             std::get<0>(masks) |= std::get<0>(temp_masks),
  //             std::get<1>(masks) |= std::get<1>(temp_masks),
  //             std::get<2>(masks) |= std::get<2>(temp_masks),
  //             std::get<3>(masks) |= std::get<3>(temp_masks), 
  //             masks), ...); 
  //       }.operator()<>(std::make_integer_sequence<uint32_t, DMAC_LVL_NUM>()));
  //     }();
  //     // Structured binding for masks, set config
  //     auto [clr_prictrl, set_prictrl, clr_ctrl, set_ctrl] = prio_masks;
  //     DMAC[inst_index].PRICTRL0.reg &= ~clr_prictrl;
  //     DMAC[inst_index].PRICTRL0.reg |= set_prictrl;
  //     DMAC[inst_index].CTRL.reg &= ~clr_ctrl;
  //     DMAC[inst_index].CTRL.reg |= set_ctrl;
  //   }

  //   /// FINAL_V1
  //   bool copy_config(const int &inst_index, const int &other_index) {
  //     if (inst_index == other_index) [[likely]] { return false; }
  //     auto &o_chint = DMAC[inst_index].Channel[other_index].CHINTENSET.reg;
  //     DMAC[inst_index].Channel[other_index].CHINTENSET.reg |= o_chint;
  //   }

  //   // FINAL_V1
  //   constexpr bool valid_index(const int &inst_index) {
  //     return inst_index >= 0 && inst_index < ref::inst_num;
  //   }

  // }


  // namespace irq {

  //   // FINAL_V1
  //   struct Config {
  //     std::array<int, ref::irq_array.size()> priority_lvl = {-1, -1, -1, -1, -1};
  //   };

  //   // FINAL_V1
  //   template<Config cfg>
  //   void set_config(const int &inst_index) {
  //     constexpr auto irq_prio_arr = []() consteval {
  //       using prio_t = decltype(NVIC_GetPriority(DMAC_0_IRQn));
  //       return ([]<uint32_t... Is>(std::integer_sequence<uint32_t, Is...>) consteval {
  //         std::array<prio_t, ref::irq_array.size()> result_arr{};

  //         // Get reg value for individual priority lvl
  //         constexpr auto calc_index = []<int i>() consteval {
  //           constexpr auto prio_cfg = cfg.priority_lvl.at(i);
  //           prio_t result_prio{0};
            
  //           if constexpr (prio_cfg != -1) {
  //             constexpr unsigned int prio_found = std::distance(irqpri_map.begin(),
  //                 std::find(irqpri_map.begin(), irqpri_map.end(), prio_cfg));
  //             static_assert(prio_cfg < irqpri_map.size(), 
  //                 "SIO ERROR (DMA): irq_prio config invalid.");
  //             result_prio = prio_found;
  //           }
  //           return result_prio;
  //         }; 
  //         // Build up & return array of cfg reg values (fold expr.)
  //         return ((result_arr.at(Is) = calc_index.template operator()<Is>(), result_arr), ...); 
  //       }.operator()<>(std::make_integer_sequence<uint32_t, ref::irq_array.size()>()));
  //     }();
  //     // Set priority lvl config
  //     uint32_t irq_off = irq_array.size() * inst_index;
  //     for (uint32_t i = 0; i < ref::irq_array.size(); i++) {
  //       auto irq = static_cast<IRQn_Type>(irq_off + ref::irq_array[i]);
  //       NVIC_SetPriority(irq, irq_prio_arr[i]); 
  //     }
  //   }

  //   // FINAL_V1
  //   bool copy_config(const int &inst_index, const int &other_index) {
  //     if (inst_index == other_index) { return false; }
  //     int irq_off = ref::irq_array.size() * inst_index;
  //     int o_irq_off = ref::irq_array.size() * other_index;

  //     for (int i = 0; i < ref::irq_array.size(); i++) {
  //       auto targ_prio = NVIC_GetPriority(static_cast<IRQn_Type>(ref::irq_array[i] + o_irq_off));
  //       NVIC_SetPriority(static_cast<IRQn_Type>(ref::irq_array[i] + irq_off), targ_prio);
  //     }
  //   }





  // static constexpr channel_state_t targ_state = channel_state_t::disabled; // TO REMOVE
  // // template<const channel_state_t _state_>
  // bool set_state(const bool &blocking) {
  //   auto &wb = DMAC[inst_index] 
  //   auto atomic = temp::atomic_section();
  //   auto init_state = state();
    
  //   // Disable channel
  //   if constexpr (targ_state == channel_state_t::disabled) [[unlikely]] {
  //     DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = 0;
  //     while(DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE);
    
  //   } else { // Enable/reset channel (if writeback invalid/end)
  //     if (!inst[inst_index].wbdesc[ch_index].BTCNT.reg && !wbdesc[ch_index].DESCADDR.reg && 
  //         init_state == channel_state_t::suspended) { 
  //       DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = 0;
  //       while(DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE);
  //     }
  //     DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE = 1;
  //   }
  //   // Suspend channel
  //   if constexpr (targ_state == channel_state_t::suspended) [[unlikely]] {
  //     DMAC[inst_index].Channel[ch_index].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
  //     while(DMAC[inst_index].Channel[ch_index].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);

  //   // Resume channel (if susp ONLY) -> otherwise skip next susp
  //   } else if (init_state == channel_state_t::suspended) {
  //     DMAC[inst_index].Channel[ch_index].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
  //     DMAC[inst_index].Channel[ch_index].CHINTFLAG.bit.SUSP = 1;
  //   }
  // }


  // // FINAL_V1
  // inline channel_state_t channel_state() {
  //   if (!DMAC[inst_index].Channel[ch_index].CHCTRLA.bit.ENABLE) {
  //     return channel_state_t::disabled;

  //   } else if (DMAC[inst_index].Channel[ch_index].CHINTFLAG.bit.SUSP) {
  //     return channel_state_t::suspended;
  //   }
  //   return channel_state_t::enabled;
  // }

}
