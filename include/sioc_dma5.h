
#pragma once

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
#include <bit>
#include <memory>

namespace sioc::dma {

  namespace detail {

    typedef struct {
      DMAC_BTCTRL_Type    BTCTRL;
      DMAC_BTCNT_Type     BTCNT;
      DMAC_SRCADDR_Type   SRCADDR;
      DMAC_DSTADDR_Type   DSTADDR;
      DMAC_DESCADDR_Type  DESCADDR;
    } DmacDescriptor_NV;

    constexpr struct {}dummy;
    const auto dummy_addr = std::bit_cast<uintptr_t>(&dummy);

    void dummy_cb(const int&, const callback_flag_t&) {}

    constexpr auto ctrl_vmsk = (DMAC_CTRL_DMAENABLE | DMAC_CTRL_SWRST);
    constexpr auto chctrla_vmsk = (DMAC_CHCTRLA_ENABLE | DMAC_CHCTRLA_SWRST);

    // FINAL_V1
    __always_inline bool inst_valid(const int &inst) 
    {
      return inst >= 0 && inst < ref::inst_num;
    }

    __always_inline bool ch_valid(const int &inst, const int &channel) 
    {
      return inst_valid(inst) && channel >= 0 && channel < ref::ch_num;
    }

    // DmacDescriptor* get_wbdesc(const int &inst, const int &ch_index) {
    //   auto wb_base = reinterpret_cast<DmacDescriptor*>(DMAC[inst].WRBADDR.reg);
    //   return wb_base + ch_index;
    // }
  }

  struct {
    const int inst{0};
    Dmac &ireg = DMAC[inst];

    // FINAL_V1
    void init() 
    {
      if (is_init() == true) [[unlikely]] { return; }
      exit();
      // Enable interrupts through NVIC
      int irq_off = ref::irq_array.size() * inst;
      for (int i = 0; i < ref::irq_array.size(); i++) {
        NVIC_EnableIRQ(static_cast<IRQn_Type>(ref::irq_array[i] + irq_off));
      }
      // Set base mem sections to dummy & enable DMA
      ireg.BASEADDR.reg = detail::dummy_addr;
      ireg.WRBADDR.reg = detail::dummy_addr;
      MCLK->AHBMASK.reg |= (1 << (MCLK_AHBMASK_DMAC_Pos + inst));
      ireg.CTRL.bit.DMAENABLE = 1;
    } 

    // FINAL_V1
    void exit() 
    {
      if (is_init() == true) [[likely]] {
        using namespace ref;
        // Free allocated base descriptor memory
        if (DMAC[inst].BASEADDR.reg != detail::dummy_addr) {
          auto base_ptr = std::bit_cast<DmacDescriptor*>(ireg.BASEADDR.reg);
          auto wb_ptr = std::bit_cast<DmacDescriptor*>(ireg.WRBADDR.reg);
          delete[] base_ptr;
          delete[] wb_ptr;
        }
        // Disable dma & clear registers
        ireg.CTRL.bit.DMAENABLE = 0;
        while(ireg.CTRL.bit.DMAENABLE == 1);
        ireg.CTRL.bit.SWRST = 1;
        while(ireg.CTRL.bit.SWRST == 1);
        MCLK->AHBMASK.reg &= ~(1 << (MCLK_AHBMASK_DMAC_Pos + inst));
          
        // Disable, clear pending & reset priority for all interrupts
        int irq_off = irq_array.size() * inst;
        for (int i = 0; i < irq_array.size(); i++) {
          auto curr_irq = static_cast<IRQn_Type>(irq_array[i] + irq_off);
          NVIC_DisableIRQ(curr_irq);
          NVIC_ClearPendingIRQ(curr_irq);
          NVIC_SetPriority(curr_irq, def_irq_prio);
        }
      }
    }

    // FINAL_V1
    bool is_init() 
    {
      return (ireg.CTRL.bit.DMAENABLE == 1);
    }

    // FINAL_V1
    typedef struct Config 
    {
      std::array<int8_t, ref::prilvl_num> prilvl_en{-1, -1, -1, -1};
      std::array<int8_t, ref::prilvl_num> prilvl_rr{-1, -1, -1, -1};
      std::array<int8_t, ref::prilvl_num> prilvl_squal{-1, -1, -1, -1};
      std::array<int8_t, ref::irq_array.size()> prilvl_irq_prio{-1, -1, -1, -1, -1};
    };

    // FINAL_V1
    template<Config cfg>
    void set_config() 
    {
      using namespace ref;
      // Generate masks for setting cfg in registers @ compile time
      constexpr auto &[prictrl_clr, prictrl_set, ctrl_clr, ctrl_set] = []() consteval {
        using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
        using ctrl_t = decltype(std::declval<DMAC_CTRL_Type>().reg);

        return ([]<int... Is>(std::integer_sequence<int, Is...>) consteval {
          std::tuple<prictrl_t, prictrl_t, ctrl_t, ctrl_t> masks{}, temp_masks{};

          // Generate mask for specific priority lvl cfg
          auto prictrl_msk_i = []<int i>() consteval {
            constexpr auto squal_cfg = cfg.prilvl_squal.at(i);
            constexpr auto rr_cfg = cfg.prilvl_rr.at(i);
            constexpr auto en_cfg = cfg.prilvl_en.at(i);
            prictrl_t set_msk_prio{0}, clr_msk_prio{0};
            ctrl_t clr_msk_ctrl{0}, set_msk_ctrl{0};

            if constexpr (squal_cfg != -1) {
              constexpr int squal_conv = std::distance(squal_map.begin(), 
                  std::find(squal_map.begin(), squal_map.end(), squal_cfg));
              static_assert(squal_conv < squal_map.size(), 
                  "SIO ERROR (DMA): sys prilvl_squal config invalid.");
              
              int off = i * (DMAC_PRICTRL0_QOS1_Pos - DMAC_PRICTRL0_QOS0_Pos);
              clr_msk_prio |= (DMAC_PRICTRL0_QOS0_Msk << off);
              set_msk_prio |= (squal_conv << (off + DMAC_PRICTRL0_QOS0_Pos));
            }
            if constexpr (rr_cfg != -1) {
              static_assert(rr_cfg == 0 || rr_cfg == 1, 
                  "SIO ERROR (DMA): sys prilvl_rr config invalid.");
              int off = i * (DMAC_PRICTRL0_RRLVLEN1_Pos - DMAC_PRICTRL0_RRLVLEN0_Pos);
              clr_msk_prio |= (1U << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
              set_msk_prio |= (rr_cfg << (off + DMAC_PRICTRL0_RRLVLEN0_Pos));
            }
            if constexpr (en_cfg != -1) {
              static_assert(en_cfg == 0 || en_cfg == 1, 
                  "SIO ERROR (DMA): sys prilvl_en config invalid.");
              int off = i * (DMAC_CTRL_LVLEN1_Pos - DMAC_CTRL_LVLEN0_Pos);
              clr_msk_ctrl |= (1U << (off + DMAC_CTRL_LVLEN0_Pos));
              set_msk_ctrl |= (en_cfg << (off + DMAC_CTRL_LVLEN0_Pos));
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
        }.operator()<>(std::make_integer_sequence<int, ref::prilvl_num>()));
      }();
      // Apply masks from structured binding to registers...
      ireg.PRICTRL0.reg &= ~prictrl_clr;
      ireg.PRICTRL0.reg |= prictrl_set;
      ireg.CTRL.reg &= ~ctrl_clr;
      ireg.CTRL.reg |= ctrl_set;

      // Compute reg values for irq priority
      constexpr auto irq_prio_arr = []() consteval {
        using prio_t = decltype(NVIC_GetPriority(DMAC_0_IRQn));
        return ([]<int... Is>(std::integer_sequence<int, Is...>) consteval {
          std::array<prio_t, irq_array.size()> prio_arr{};

          // Get reg value for individual priority lvl
          auto calc_index = []<int i>() consteval {
            constexpr auto irq_prio_cfg = cfg.prilvl_irq_prio.at(i);
            prio_t prio_result{0};
            
            if constexpr (irq_prio_cfg != -1) {
              constexpr auto prio_reg = std::distance(irqpri_map.begin(),
                  std::find(irqpri_map.begin(), irqpri_map.end(), irq_prio_cfg));
              static_assert(irq_prio_cfg < irqpri_map.size(), 
                  "SIO ERROR (DMA): sys prilvl_irq_prio config invalid."); 
              prio_result = prio_reg;
            }
            return prio_result;
          }; 
          // Build up & return array of cfg reg values (fold expr.)
          return ((prio_arr.at(Is) = calc_index.template operator()<Is>(), prio_arr), ...); 
        }.operator()<>(std::make_integer_sequence<int, irq_array.size()>()));
      }();
      // Set priority lvl config
      int irq_off = inst * irq_array.size();
      for (int i = 0; i < irq_array.size(); i++) {
        auto irq = static_cast<IRQn_Type>(irq_off + irq_array[i]);
        NVIC_SetPriority(irq, irq_prio_arr[i]); 
      }
    }

    // FINAL_V3
    bool copy_config(const int &other_inst)
    {
      ireg.CTRL.reg &= ~(DMAC_CTRL_MASK | detail::ctrl_vmsk);
      ireg.CTRL.reg |= (DMAC[other_inst].CTRL.reg & ~detail::ctrl_vmsk);
      ireg.PRICTRL0.reg = DMAC[other_inst].PRICTRL0.reg;

      // Copy irq priority level
      const int t_irq_off = inst * ref::irq_array.size(); 
      const int o_irq_off = other_inst * ref::irq_array.size();
      
      for (int i = 0; i < ref::irq_array.size(); i++) {
        const auto this_irq = static_cast<IRQn_Type>(ref::irq_array[i] + t_irq_off);
        const auto other_irq = static_cast<IRQn_Type>(ref::irq_array[i] + o_irq_off);
        NVIC_SetPriority(this_irq, NVIC_GetPriority(other_irq)); 
      }
    }

    // FINAL_V3
    void clear_config()
    {
      ireg.CTRL.reg &= ~(DMAC_CTRL_MASK | detail::ctrl_vmsk);
      ireg.PRICTRL0.reg &= ~(DMAC_PRICTRL0_MASK);

      const int irq_off = inst * ref::irq_array.size();
      for (int i = 0; i < ref::irq_array.size(); i++) {
        const auto curr_irq = static_cast<IRQn_Type>(ref::irq_array[i] + irq_off);
        NVIC_SetPriority(curr_irq, ref::def_irq_prio);
      }
    }

    // FINAL_V3
    Config get_config() 
    {
      Config cfg{};
      for (int i = 0; i < ref::prilvl_num; i++) {
        const int en_off = i * (DMAC_CTRL_LVLEN1_Pos - DMAC_CTRL_LVLEN0_Pos);
        cfg.prilvl_en[i] = static_cast<int8_t>(ireg.CTRL.reg & 
            (1U << (en_off + DMAC_CTRL_LVLEN0_Pos)));

        const int rr_off = i * (DMAC_PRICTRL0_RRLVLEN1_Pos - DMAC_PRICTRL0_RRLVLEN0_Pos);
        cfg.prilvl_rr[i] = static_cast<int8_t>(ireg.PRICTRL0.reg & 
            (1U << (rr_off + DMAC_PRICTRL0_RRLVLEN0_Pos)));

        const int sq_off = i * (DMAC_PRICTRL0_QOS1_Pos - DMAC_PRICTRL0_QOS0_Pos); 
        cfg.prilvl_squal[i] = static_cast<int8_t>(ref::squal_map[
            ireg.PRICTRL0.reg & (DMAC_PRICTRL0_QOS0_Msk << sq_off)]);
      }                                                                                   
      // Number of priority levels & interrupts do not match...
      const int irq_off = inst * ref::irq_array.size();
      for (int i = 0; i < ref::irq_array.size(); i++) {
        const auto curr_irq = static_cast<IRQn_Type>(ref::irq_array[i] + irq_off);
        cfg.prilvl_irq_prio[i] = ref::irqpri_map[NVIC_GetPriority(curr_irq)];
      }
      return cfg;
    }


    struct {
      const int inst{0};
      const int ch{0};
      DmacChannel &chreg = DMAC[inst].Channel[ch];
      

        // FINAL_V1
        bool get_init() const {
          return chreg.CHINTENSET.bit.TERR == 1;
        }

        // FINAL_V1
        typedef struct Config 
        {
          int8_t burstlen   = -1;  
          trigsrc_t trigsrc  = trigsrc_t::null;
          trigact_t trigact  = trigact_t::null;
          int8_t prilvl     = -1; 
          int8_t threshold  = -1;
          int8_t runstdby   = -1;
        };

        // FINAL_V1
        template<Config cfg>
        void set_config() 
        {
          using namespace ref;
          // Record enable status & disable channel
          bool prev_en = chreg.CHCTRLA.bit.ENABLE == 1;
          chreg.CHCTRLA.bit.ENABLE = 1;
          while(chreg.CHCTRLA.bit.ENABLE == 1);

          // Compute masks for config in ctrl register
          constexpr auto &[ctrl_clr, ctrl_set] = [&]() consteval {
            using ctrl_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
            ctrl_t clr_msk{0}, set_msk{0};

            if constexpr (cfg.burstlen != -1) {
              constexpr int bl_reg = std::distance(burstlen_map.begin(),
                  std::find(burstlen_map.begin(), burstlen_map.end(), cfg.burstlen));
              static_assert(bl_reg < burstlen_map.size(), 
                  "SIO ERROR (DMA): channel burstlen config invalid.");

              clr_msk |= (DMAC_CHCTRLA_BURSTLEN_Msk);
              set_msk |= (bl_reg << DMAC_CHCTRLA_BURSTLEN_Pos);
            }
            if constexpr (cfg.threshold != -1) {
              constexpr int th_reg = std::distance(threshold_map.begin(),
                  std::find(threshold_map.begin(), threshold_map.end(), cfg.threshold));
              static_assert(th_reg < threshold_map.size(), 
                  "SIO ERROR (DMA): channel threshold config invalid.");

              clr_msk |= (DMAC_CHCTRLA_THRESHOLD_Msk);
              set_msk |= (th_reg << DMAC_CHCTRLA_THRESHOLD_Pos);
            }
            if constexpr (cfg.trigsrc != trigsrc_t::null) {
              clr_msk |= (DMAC_CHCTRLA_TRIGSRC_Msk);
              set_msk |= (static_cast<ctrl_t>(cfg.trigsrc) << DMAC_CHCTRLA_TRIGSRC_Pos);
            }
            if constexpr (cfg.trigact != trigact_t::null) {
              clr_msk |= (DMAC_CHCTRLA_TRIGACT_Msk);
              set_msk |= (static_cast<ctrl_t>(cfg.trigact) << DMAC_CHCTRLA_TRIGACT_Pos);
            }
            if constexpr (cfg.runstdby != -1) {
              static_assert(cfg.runstdby == 0 || cfg.runstdby == 1, 
                  "SIO ERROR (DMA): channel runstdby config invalid.");
              clr_msk |= (DMAC_CHCTRLA_RUNSTDBY);
              set_msk |= (static_cast<bool>(cfg.runstdby) << DMAC_CHCTRLA_RUNSTDBY_Pos);
            }
            return std::make_pair(clr_msk, set_msk);
          }(); 
          chreg.CHCTRLA.reg &= ~ctrl_clr; 
          chreg.CHCTRLA.reg |= ctrl_set; 

          // Compute masks for config in prictrl register & set them
          constexpr auto &[chprilvl_clr, chprilvl_set] = [&]() consteval {
            using prictrl_t = decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
            prictrl_t clr_msk{0}, set_msk{0};

            if constexpr (cfg.prilvl != -1) {
              constexpr int plvl_reg = std::distance(prilvl_map.begin(),
                  std::find(prilvl_map.begin(), prilvl_map.end(), cfg.prilvl));
              static_assert(plvl_reg < prilvl_map.size(), 
                  "SIO ERROR (DMA): channel prilvl config invalid.");

              clr_msk |= (DMAC_CHPRILVL_PRILVL_Msk);
              set_msk |= (plvl_reg << DMAC_CHPRILVL_PRILVL_Pos);
            }
            return std::make_pair(clr_msk, set_msk);
          }();
          chreg.CHPRILVL.reg &= ~chprilvl_clr;
          chreg.CHPRILVL.reg |= chprilvl_set;
          chreg.CHCTRLA.bit.ENABLE = prev_en; // > Re-enable channel 
        }

        // FINAL_V2
        void clear_config() 
        {
          chreg.CHCTRLA.reg &= ~(DMAC_CHCTRLA_MASK | detail::chctrla_vmsk);
          chreg.CHPRILVL.reg &= ~(DMAC_CHPRILVL_MASK);
        }

        // FINAL_V2
        bool copy_config(const int &other_inst, const int &other_ch)  
        {
          DmacChannel &o_chreg = DMAC[other_inst].Channel[other_ch];
          chreg.CHCTRLA.reg &= ~(DMAC_CHCTRLA_MASK | detail::chctrla_vmsk);
          chreg.CHCTRLA.reg |= (o_chreg.CHCTRLA.reg & ~(detail::chctrla_vmsk)); 
          chreg.CHPRILVL.reg = o_chreg.CHPRILVL.reg;
        }

        // FINAL_V1
        Config get_config() 
        {
          auto burstlen_r = DMAC[inst].Channel[ch].CHCTRLA.bit.BURSTLEN;
          auto trigsrc_r = DMAC[inst].Channel[ch].CHCTRLA.bit.TRIGSRC;
          auto trigact_r = DMAC[inst].Channel[ch].CHCTRLA.bit.TRIGACT;
          auto prilvl_r = DMAC[inst].Channel[ch].CHPRILVL.bit.PRILVL;
          auto threshold_r = DMAC[inst].Channel[ch].CHCTRLA.bit.THRESHOLD;
          auto runstdby_r = DMAC[inst].Channel[ch].CHCTRLA.bit.RUNSTDBY;

          return Config{
            .burstlen = static_cast<int8_t>(ref::burstlen_map[burstlen_r]),
            .trigsrc = static_cast<trigsrc_t>(trigsrc_r),
            .trigact = static_cast<trigact_t>(trigact_r),
            .prilvl = static_cast<int8_t>(ref::prilvl_map[prilvl_r]),
            .threshold = static_cast<int8_t>(ref::threshold_map[threshold_r]),
            .runstdby = static_cast<int8_t>(runstdby_r)
          };
        }

    }channel[ref::ch_num]{inst, INIT_SEQ};


    struct {
      const int inst{0};
      const int ch{0};
      

      typedef struct Config 
      {
        int8_t susp_int   = -1;
        int8_t err_int    = -1;
        int8_t tcmpl_int  = -1;
      };


      
    }irq[ref::ch_num]{INIT_SEQ};


  }inst[ref::inst_num]{INIT_SEQ};


  namespace irq 
  {

    // FINAL_V1


    // FINAL_V1
    template<Config cfg>
    bool set_config(const int &inst, const int &ch) 
    {
      if (!channel::is_valid(inst, ch)) { return false; }
      using namespace ref;
      // Compute masks for interrupt enable/disable cfg registers
      constexpr auto &[chint_clr, chint_set] = [&]() consteval {
        using inten_t = decltype(std::declval<DMAC_CHINTENSET_Type>().reg);
        inten_t clr_msk{0}, set_msk{0};

        if constexpr (cfg.susp_int != -1) {
          static_assert(cfg.susp_int == 0 || cfg.susp_int == 1, 
              "SIO ERROR (DMA): irq susp_int config invalid.");
          clr_msk |= (DMAC_CHINTENCLR_SUSP);
          set_msk |= (static_cast<bool>(cfg.susp_int) << DMAC_CHINTENSET_SUSP_Pos);
        }
        if constexpr (cfg.err_int != -1) {
          static_assert(cfg.err_int == 0 || cfg.err_int == 1, 
              "SIO ERROR (DMA): irq err_int config invalid.");
          clr_msk |= (DMAC_CHINTENCLR_TERR);
          set_msk |= (static_cast<bool>(cfg.err_int) << DMAC_CHINTENSET_TERR_Pos);
        }
        if constexpr (cfg.tcmpl_int != -1) {
          static_assert(cfg.tcmpl_int == 0 || cfg.tcmpl_int == 1, 
              "SIO ERROR (DMA): irq tcmpl_int config invalid.");
          clr_msk |= (DMAC_CHINTENCLR_TCMPL);
          set_msk |= (static_cast<bool>(cfg.tcmpl_int) << DMAC_CHINTENSET_TCMPL_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      // Set registers with masks
      DMAC[inst].Channel[ch].CHINTENSET.reg &= ~chint_clr;
      DMAC[inst].Channel[ch].CHINTENSET.reg |= chint_set;
      return false;
    }

    // FINAL_V1
    bool clear_config(const int &inst, const int &ch) 
    {
      if (!channel::is_valid(inst, ch)) { return false; }
      DMAC[inst].Channel[ch].CHINTENCLR.reg = DMAC_CHINTENCLR_MASK;
      return true;
    }

    // FINAL_V1
    bool copy_config(const int &targ_inst, const int &targ_ch, const int &other_inst,
        const int &other_ch)
    {
      auto other_reg = DMAC[other_inst].Channel[other_ch].CHINTENSET.reg; 
      DMAC[targ_inst].Channel[targ_ch].CHINTENSET.reg = other_reg; 
    }

    // FINAL_V1
    Config get_config(const int &inst, const int &ch)
    {
      return Config{
        .susp_int = static_cast<bool>(DMAC[inst].Channel[ch].CHINTENSET.bit.SUSP),
        .err_int = static_cast<bool>(DMAC[inst].Channel[ch].CHINTENSET.bit.TERR),
        .tcmpl_int = static_cast<bool>(DMAC[inst].Channel[ch].CHINTENSET.bit.TCMPL)
      };
    }

  }



  // namespace ch {



    // // FINAL_V1
    // bool get_init(const int &inst) {
    //   return DMAC[inst].CTRL.reg != DMAC_CTRL_RESETVALUE;
    // }

    // void init(const int &inst, const int &ch_index) {
    //   if (!get_init(inst, ch_index) && sys::get_init(inst)) [[unlikly]] {
    //     int max_index{-1};
    //     for (int i = 0; i < ref::ch_num; i++) { // Find current ch w/ max index...
    //       if (DMAC[inst].Channel[i].CHINTFLAG.bit.TCMPL == 1) { max_index = i; }
    //     }
    //     if (max_index < ch_index) { // If wb mem section < index -> reallocate it...
    //       auto new_wb = new(ref::desc_align) DmacDescriptor[ch_index];
    //       if (DMAC[inst].WRBADDR.reg != std::bit_cast<uintptr_t>(&detail::bdesc)) {
    //         auto old_wb = reinterpret_cast<DmacDescriptor*>(DMAC[inst].WRBADDR.reg);
    //         memcpy(new_wb, old_wb, sizeof(DmacDescriptor) * max_index);
    //       }
    //       DMAC[inst].WRBADDR.reg = std::bit_cast<uintptr_t>(&new_wb);
    //     }                                                                                                                                       
    //   }
    // }


  //   void init(const int &inst, const int &ch_index) {
  //     if (!sys::get_init(inst)) [[unlikely]] { return; } 
  //     if (get_init(inst, ch_index)) [[unlikely]] { return; }
  //     auto wb_addr = DMAC[inst].WRBADDR.reg; 

  //     // Init writeback desc mem section (first init)
  //     if (wb_addr == std::bit_cast<uintptr_t>(&detail::bdesc)) {
  //       void *wbmem = aligned_alloc(ref::desc_align, sizeof(DmacDescriptor) * ch_index);
  //       new(static_cast<DmacDescriptor*>(wbmem) + ch_index) DmacDescriptor;

  //     } else { // Possibly realloc (expand) writeback mem section
  //       for (int i = ref::ch_num - 1; i >= 0 ; i--) {
  //         if (DMAC[inst].Channel[i].CHINTENSET.bit.TERR == 1 && i < ch_index) { 
  //           void *newmem = aligned_alloc(ref::desc_align, sizeof(DmacDescriptor) * ch_index);
  //           void *oldmem = std::bit_cast<void*>(DMAC[inst].WRBADDR.reg);
  //           memcpy(newmem, oldmem, sizeof(DmacDescriptor) * i);
  //           new(static_cast<DmacDescriptor*>(newmem) + ch_index) DmacDescriptor;
          
  //         } else if(i == 0) { // Construct descriptor at give location                         
  //           void *wbmem = std::bit_cast<void*>(DMAC[inst].WRBADDR.reg);
  //           new(static_cast<DmacDescriptor*>(wbmem) + ch_index) DmacDescriptor;
  //         }
  //       }

  //     }

  //     if (l_ch == -1) {
  //     } else if (l_ch < ch_index) {
  //       auto wbdesc = (DmacDescriptor*)DMAC[inst].WRBADDR.reg;
        
  //     }
  //     auto wb_arr = reinterpret_cast<DmacDescriptor*>(DMAC[inst].WRBADDR.reg);
  //     return std::make_pair(wb_arr, last_chi);


  //     DMAC[inst].Channel[ch_index].CHINTFLAG.bit.TERR = 1;
      
  //   }

  //   void reset(const int &inst, const int &ch_index) {

  //   }

  //   // FINAL_V1
  //   bool get_init(const int &inst, const int &ch_index) {
  //     return DMAC[inst].Channel[ch_index].CHINTFLAG.bit.TERR == 1 ||
  //         DMAC[inst].Channel[ch_index].CHCTRLA.reg != DMAC_CHCTRLA_RESETVALUE;
  //   }

  // } 

  namespace irq {


  } 



    






}