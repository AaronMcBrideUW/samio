
#pragma once
#include <sioc_dma_defs.h>
#include <sioc_dma_ref.h>
#include <sam.h>
#include <array>
#include <iterator>
#include <algorithm>
#include <utility>

namespace sioc::dma {

  struct Channel;
  struct TransferDescriptor;


  namespace detail
  {
    uint32_t _chmsk_;
    TransferDescriptor *_btd_[ref::ch_num]{nullptr};
    SECTION_DMAC_DESCRIPTOR __ALIGNED(16) DmacDescriptor _bdesc_[ref::ch_num]{};
    SECTION_DMAC_DESCRIPTOR __ALIGNED(16) DmacDescriptor _wbdesc_[ref::ch_num]{};

    inline struct {}dummy_loc;
  }

  
  typedef struct Channel 
  {

    const int id;

    typedef struct Config
    {
      int32_t burstlen   = -1;
      trigsrc_t trigsrc  = trigsrc_t::null;
      trigact_t trigact  = trigact_t::null;
      int32_t prilvl     = -1;
      int32_t threshold  = -1;
      int32_t runstdby   = -1; // Boolean cond
      int32_t susp_cb    = -1; // Boolean cond
    };

    template<Config cfg>
    void set_config() {
      using namespace ref;

      // Compute masks for config in ctrl register
      auto [ctrl_clear_msk, ctrl_set_msk] = [&]() consteval {
        using ctrl_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);
        ctrl_t clr_msk = 0, set_msk = 0;

        // For numeric settings -> use maps to find correspondign reg value
        if (cfg.burstlen != -1) {
          constexpr uint32_t bl_reg = std::distance(burstlen_map.begin(),
              std::find(burstlen_map.begin(), burstlen_map.end(), cfg.burstlen));
          static_assert(bl_reg < burstlen_map.size(), "SIO ERROR: DMA channel config");

          clr_msk |= (DMAC_CHCTRLA_BURSTLEN_Msk);
          set_msk |= (bl_reg << DMAC_CHCTRLA_BURSTLEN_Pos);
        }
        if (cfg.threshold != -1) {
          constexpr uint32_t th_reg = std::distance(threshold_map.begin(),
              std::find(threshold_map.begin(), threshold_map.end(), cfg.threshold));
          static_assert(th_reg < threshold_map.size(), "SIO ERROR: DMA channel config");

          clr_msk |= (DMAC_CHCTRLA_THRESHOLD_Msk);
          set_msk |= (th_reg << DMAC_CHCTRLA_THRESHOLD_Pos);
        }

        // Cast enum/"bool" types, underlying val corresponds to reg value
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
      // Use masks to mask/clear & set specified cfg in ctrl reg
      DMAC->Channel[id].CHCTRLA.reg &= ~ctrl_clear_msk;
      DMAC->Channel[id].CHCTRLA.reg |= ctrl_set_msk;

      // Compute masks for config in prictrl register
      auto [prictrl_clr_msk, prictrl_set_msk] = [&]() consteval {
        using prictrl_t decltype(std::declval<DMAC_PRICTRL0_Type>().reg);
        prictrl_t clr_msk = 0, set_msk = 0;

        if (cfg.prilvl != -1) {
          constexpr uint32_t plvl_reg = std::distance(prilvl_map.begin(),
              std::find(prilvl_map.begin(), prilvl_map.end(), cfg.prilvl));
          static_assert(plvl_reg < prilvl_map.size(), "SIO ERROR: DMA channel config");

          clr_msk |= (DMAC_CHPRILVL_PRILVL_Msk);
          set_msk |= (plvl_reg << DMAC_CHPRILVL_PRILVL_Pos);
        }
        return std::make_pair(clr_msk, set_msk);
      }();
      // Use masks to mask/clear & set specified config in prictrl reg
      DMAC->Channel[id].CHPRILVL.reg &= ~prictrl_clr_msk;
      DMAC->Channel[id].CHPRILVL.reg |= prictrl_set_msk;
    }

  };



}

