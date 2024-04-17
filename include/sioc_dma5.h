
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

  void allocate_channel(const int &inst_index, const int &ch_index) {
    if (DMAC[inst_index].Channel[ch_index].CHINTENSET.bit.TERR == 1) { return; }
    if (DMAC[inst_index].CTRL.bit.DMAENABLE == 0) {
      
    }
    


  }



}