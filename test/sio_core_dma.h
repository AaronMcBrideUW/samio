/*
///////////////////////////////////////////////////////////////////////////////////////////////////
//// HEADER: DIRECT MEMORY ACCESS CORE METHODS
///////////////////////////////////////////////////////////////////////////////////////////////////

/// @b IMPORT
#pragma once
#include <sio_config.h>
#include <sam.h>
#include <utility>
#include <algorithm>
#include <array>
#include <string.h>

//// PRIMARY NAMESPACE
namespace sio::dma {

  class TransferDescriptor;


  /// @brief
  ///   Denotes the state of a DMA channel
  enum class channel_state {
    unknown,
    disabled,
    idle,
    suspended,
    active
  };

  /// @brief
  ///   Denotes an error flag set for a DMA channel.
  enum class channel_error {
    none,
    transfer_error,
    task_error,
    crc_error,
    invalid_index
  };

  /// @brief
  ///   Denotes what is transfered each time a DMA channel
  ///   is set to it's active state before it returns to an
  ///   "idle" state.
  enum class transfer_mode : int {
    packet  = DMAC_CHCTRLA_TRIGACT_BURST_Val,
    task    = DMAC_CHCTRLA_TRIGACT_BLOCK_Val,
    all     = DMAC_CHCTRLA_TRIGACT_TRANSACTION_Val
  };

  /// @brief 
  ///   Denotes peripheral "triggers" that can be linked
  ///   to DMA channels.
  enum class channel_peripheral : int {
    none    = DMAC_CHCTRLA_TRIGSRC_DISABLE_Val,
    /// TO-DO
  };  

  /// @brief
  ///   Denotes the calculation type used by the CRC 
  ///   checksum generator. 
  enum class crc_type : int {
    crc16 = DMAC_CRCCTRL_CRCPOLY_CRC16_Val,
    crc32 = DMAC_CRCCTRL_CRCPOLY_CRC32_Val
  };

  /// @brief
  ///   Denotes the reason for invoking a callback.
  enum class callback_flag {
    transfer_complete,
    channel_error
  };

  /// @brief
  ///   The signiture requires for DMA channel callback functions.
  ///   -> Must take and int denoting the channel number and
  ///   and enum "callback_flag" used for denoting the callback
  ///   reason/source.
  using callback_t = void (*)(int, callback_flag);


  namespace { 
    __aligned(16)
    __section(".hsram")
    typedef struct _DmacDescriptor_ {
      DMAC_BTCTRL_Type BTCTRL;
      DMAC_BTCNT_Type BTCNT;
      DMAC_SRCADDR_Type SRCADDR;
      DMAC_DSTADDR_Type DSTADDR;
      DMAC_DESCADDR_Type DESCADDR;
    };

    constexpr std::array<int, 3> bs_ref = {1, 2, 4};  
    constexpr std::array<int, 8> ss_ref = {1, 2, 4, 8, 16, 32, 62, 128};
    constexpr std::array<int, 16> bus_ref = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    constexpr std::array<int, 4> thresh_ref = {1, 2, 4, 8};
    constexpr std::array<int, 4> chpri_ref = {1, 2, 3, 4};
    constexpr std::array<int, 4> squal_ref = {1, 2, 3, 4};
    constexpr int crcshift_ref = 20;
    constexpr int irq_pri_ref = 1;
    
    constexpr std::array<uint8_t, DMAC_LVL_NUM> prilvl_irq_priority = {1, 1, 1, 1};
    constexpr std::array<uint8_t, DMAC_LVL_NUM> prilvl_service_qual = {1, 1, 2, 2};
    constexpr std::array<bool, DMAC_LVL_NUM> prilvl_rr_mode = {false, false, false, false};

    _DmacDescriptor_ _wbdesc_[SIO_DMA_CHANNELS]{};
    _DmacDescriptor_ _bdesc_[SIO_DMA_CHANNELS]{};
    TransferDescriptor *_btask_[SIO_DMA_CHANNELS] = { nullptr };
    callback_t _cbptr_[SIO_DMA_CHANNELS] = { nullptr };
  }


///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DMA CONTROL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

  template<int _dmanum_>
  struct sys_module {

    /// @brief
    ///   This method sets the initialization state of the DMA
    ///   peripheral and it's channels.
    /// @param initialized 
    ///   If true, the dma is initialized or re-initialized, if
    ///   false, the dma is system is completely reset to it's
    ///   default un-initialized state.
    /// @note
    /// - Un-initializing the DMA peripheral will clear/reset
    ///   all channels.
    /// @warning
    ///   Calling this method with the parameter of true will
    ///   re-initialize the DMA peripheral even if it is already 
    ///   initialized!
    void set_init(const bool &initialized) {
      DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
        while(DMAC->CTRL.reg & DMAC_CTRL_DMAENABLE);
      DMAC->CTRL.reg |= DMAC_CTRL_SWRST;
        while(DMAC->CTRL.reg & DMAC_CTRL_SWRST);
      if (!initialized) {
        MCLK->AHBMASK.reg &= ~MCLK_AHBMASK_DMAC;
      }
      memset(_bdesc_, 0, sizeof(_bdesc_));
      memset(_wbdesc_, 0, sizeof(_wbdesc_));
      std::fill(std::begin(_btask_), std::end(_btask_), nullptr);

      for (int i = 0; i < DMAC_LVL_NUM; i++) {
        NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
        if (!initialized) {
          NVIC_DisableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
          NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), 
            SIO_DMA_IRQ_PRIORITY_LVL);
        } else {
          NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), 
            prilvl_irq_priority[i]);
          NVIC_EnableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));

          DMAC->PRICTRL0.reg |=
            (prilvl_rr_mode[i] << (DMAC_PRICTRL0_RRLVLEN0_Pos + i * 8)) |
            (prilvl_service_qual[i] << (DMAC_PRICTRL0_QOS0_Pos + i * 8));
        }
      }
      if (initialized) {
        DMAC->BASEADDR.reg = (uintptr_t)_bdesc_;
        DMAC->WRBADDR.reg = (uintptr_t)_wbdesc_;
        MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
      }
    }

    /// @brief
    ///   This method returns true if the DMA has been initialized
    ///   or false otherwise.
    /// @note
    /// - If the DMA was not initialized using SIO's "set_init"
    ///   method (above) this method will return false.
    inline bool get_init() const noexcept {
      return (DMAC->BASEADDR.reg == (uintptr_t)_bdesc_);
    }

    /// @brief 
    ///   This method enables/disables the DMA peripheral.
    /// @param enabled 
    ///   If set to true, the DMA peripheral is enabled, if set 
    ///   to false, the DMA peripheral is disabled.
    /// @note
    /// - While the DMA peripheral is in the "disabled" state
    ///   all channels will also be in the "disabled" state. 
    inline void set_enabled(const bool &enabled) noexcept {
      if (enabled) {
        DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
      } else {
        DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
          while(DMAC->CTRL.reg & DMAC_CTRL_DMAENABLE);
      }
    }

    /// @brief
    ///   This method returns a boolean that denotes the current
    ///   enable-state of the DMA peripheral, true = enabled,
    ///   false = disabled.
    inline bool get_enabled() const noexcept {
      return (DMAC->CTRL.reg & DMAC_CTRL_DMAENABLE);
    }


    /// @brief
    ///   This method returns a pointer to the last channel
    ///   that was given access to the DMA bus by the arbator.
    /// @note
    /// - If no channel has been given DMA bus access since the
    ///   DMA was initialized then this method will return
    ///   a nullptr.
    inline channel_fn *get_active_channel() noexcept {
      int num = (DMAC->ACTIVE.reg & DMAC_ACTIVE_ID_Msk);
      if (num >= 0 && num < SIO_DMA_CHANNELS) {
        return &channel[num];
      }
      return nullptr;
    }

  }sys;

  template<int _lvl_>
  struct prilvl_config_module {

    /// @brief
    ///   This method sets the enable state of round robin arbitration
    ///   for channels of the corresponding priority lvl.
    /// @param enabled
    ///   If set to true, round robin arbitration is enabled for
    ///   channels in this priority grouping, if set to false
    ///   then the arbator chooses channels based on whichever
    ///   has the highest priority lvl (FIFO-based). 
    inline void set_round_robin(const bool &enabled) noexcept {
      unsigned int shift = (_lvl_ + DMAC_PRICTRL0_RRLVLEN0_Pos);
      if (enabled) {
        DMAC->PRICTRL0.reg |= (1 << shift);
      } else {
        DMAC->PRICTRL0.reg &= ~(1 << shift);
      }
    }

    /// @brief 
    ///   This method returns a boolean that denotes the round robin
    ///   enable state of the associated priority lvl.
    /// @return 
    ///   If true, round robin arbitration is enabled for channels at
    ///   this priority lvl, if false then round robin arbitration is
    ///   disabled for channels at this priority lvl.
    inline bool get_round_robin() const noexcept {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_RRLVLEN1_Pos
        - DMAC_PRICTRL0_RRLVLEN0_Pos) + DMAC_PRICTRL0_RRLVLEN0_Pos);
      return (DMAC->PRICTRL0.reg & (1 << shift));
    }

    /// @brief
    ///   This method sets the transfer service quality lvl for channels
    ///   set to the corresponding priority lvl.
    /// @param quality_level
    ///   The value of this integer denotes the service quality of the
    ///   DMA transfer {1, 2, 3, 4} -> higher = greater quality.
    /// @note
    ///   In this context "service quality" is synonymous with the
    ///   latency of the transfer -> greater service quality = less
    ///   latency between the trasnfer's activation and execution.
    inline bool set_service_quality(const int &quality_lvl) noexcept {
      unsigned int squal_r = std::distance(squal_ref.begin(), 
        std::find(squal_ref.begin(), squal_ref.end(), quality_lvl));

      if (squal_r < squal_ref.size()) {
        unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
          - DMAC_PRICTRL0_QOS0_Pos) + DMAC_PRICTRL0_QOS0_Pos);

        DMAC->PRICTRL0.reg &= ~(DMAC_PRICTRL0_QOS0_Msk << shift);
        DMAC->PRICTRL0.reg |= (squal_r << shift);
        return true;
      }
      return false;
    }

    /// @brief
    ///   This method gets the service quality configuration set
    ///   for channels at this priority level.
    /// @return
    ///   An integer denoting the service quality level set
    ///   for this priority level {1, 2, 3, 4} -> higher number
    ///   = greater service quality.
    inline int get_service_quality() {
      unsigned int shift = (_lvl_ * (DMAC_PRICTRL0_QOS1_Pos
        - DMAC_PRICTRL0_QOS0_Pos) + DMAC_PRICTRL0_QOS0_Pos);
      return squal_ref.at(DMAC->PRICTRL0.reg 
        & (DMAC_PRICTRL0_QOS0_Msk << shift));
    }

    /// @brief
    ///   This method sets the enable state for channels at
    ///   this priority level.
    /// @param enabled
    ///   If set to true, channels set to this priority level
    ///   are able to execute transfers, if set to false channels 
    ///   at this priority level are not able to execute transfers.
    inline void set_enabled(const bool &enabled) noexcept {
      if (enabled) {
        DMAC->CTRL.reg |= (1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
      } else {
        DMAC->CTRL.reg &= ~(1 << (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
      }
    }

    /// @brief
    ///   This method returns a boolean that denotes the enable
    ///   state of this priority level.
    /// @return 
    ///   A boolean -> If true, channels at this priority lvl have
    ///   been enabled and can execute transfers. If false, channels
    ///   at this priority level have been disabled and cannot
    ///   execute transfers.
    inline bool get_enabled() const noexcept {
      return (DMAC->CTRL.reg & (_lvl_ + DMAC_CTRL_LVLEN0_Pos));
    }
  }


  /// @b "dma"
  struct dma_fn {



    


///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: CHECKSUM GENERATOR (CRC) CONTROLLER
///////////////////////////////////////////////////////////////////////////////////////////////////

    /// @b "crc"
    inline static struct crc_fn {

      /// @brief 
      ///   Sets the type of crc that is generated by the dma.
      /// @param mode 
      ///   When set to crc16 the crc generator uses a 16 degree 
      ///   polynomial and when set to "crc32" the crc generator 
      ///   uses a 32 degree polynomial for generating the checksum.
      inline void set_type(const crc_type &type) noexcept {
        DMAC->CRCCTRL.reg &= ~DMAC_CRCCTRL_CRCPOLY_Msk;
        DMAC->CRCCTRL.reg |= (static_cast<int>(type) 
          << DMAC_CRCCTRL_CRCPOLY_Pos);
      }

      /// @brief
      ///   This method returns an enum that denotes the current 
      ///   "crc_type" set for the crc generator {"crc16", "crc32"}
      inline crc_type get_type() const noexcept {
        return static_cast<crc_type>(DMAC->CRCCTRL.reg 
          & DMAC_CRCCTRL_CRCPOLY_Msk);
      }

      /// @brief 
      ///   Sets the input for the crc generator.
      /// @param ch_num 
      ///   This is an integer that denotes the index number of the
      ///   dma channel to set as the input for the crc generator.
      /// @return 
      ///   Returns true if a valid channel number was entered and the
      ///   crc source config was successfully set or false otherwise.
      /// @note
      /// - If the crc generator is currently generating a checksum then
      ///   then the source cannot be changed.
      inline bool set_channel(const int &ch_num) noexcept {
        if (!is_busy() && ch_num >= 0 && ch_num < SIO_DMA_CHANNELS) {
          DMAC->CRCCTRL.reg &= ~DMAC_CRCCTRL_CRCSRC_Msk;
          DMAC->CRCCTRL.reg 
            |= DMAC_CRCCTRL_CRCSRC(ch_num + crcshift_ref);
          return true;
        }
        return false;
      }

      /// @brief 
      ///   This method returns an integer that denotes the index 
      ///   number of the dma channel currently set as the crc 
      ///   generator's input source.
      inline int get_channel() const noexcept {
        return (DMAC->CRCCTRL.reg & DMAC_CRCCTRL_CRCSRC_Msk); 
      }

      /// @brief 
      ///  This method returns true if the crc generator is currently
      ///  busy (i.e. generating a checksum) or false otherwise.
      inline bool is_busy() const noexcept {
        return (DMAC->CRCSTATUS.reg & DMAC_CRCSTATUS_CRCBUSY);
      }

      /// @brief
      ///   Pointer to data input register for CRC peripheral.
      static inline volatile void *input_ptr 
        = (void*)REG_DMAC_CRCDATAIN;

      /// @brief
      ///   Pointer to data output register for CRC peripheral.
      static inline volatile void *output_ptr 
        = (void*)REG_DMAC_CRCCHKSUM;

    }crc;


///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DMA CHANNEL
///////////////////////////////////////////////////////////////////////////////////////////////////
      
    /// @b "channel"
    inline static struct channel_fn {

      /// @brief
      ///   Resets progress through transfer descriptor list.
      /// @note
      /// - If the channel is current active, it's state will
      ///   be set to IDLE.
      inline bool reset_transfer(void) noexcept {
        using enum channel_state;
        channel_state state = get_state();
        if (state != disabled) {
          DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
          while(DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE);
        }
        DMAC->Channel[_index_].CHINTFLAG.reg &= ~DMAC_CHINTFLAG_RESETVALUE;
        memset(&_wbdesc_[_index_], 0, sizeof(_wbdesc_[0]));

        if (state != disabled) {
          DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
        }
        if (state == suspended) {
          DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
        } 
      }

      /// @brief
      ///   Resets the channel to it's defaut, initial state.
      /// @note
      /// - This method also removes all currently assigned transfer
      ///   descriptors from the channel.
      inline void reset_channel() noexcept {
        auto state = get_state();
        bool enabled, susp;
        
        if (state != channel_state::unknown) [[likely]] {
          susp = (state == channel_state::suspended);

          if (state != channel_state::disabled) {
            enabled = true;
            DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE);
          }
        }
        DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_SWRST;
        while(DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_SWRST);

        clear_descriptors();
        memset(&_wbdesc_[_index_], 0, sizeof(_wbdesc_[0]));

        if (enabled) {
          DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
          if (susp) {
            DMAC->Channel[_index_].CHCTRLB.reg 
              |= DMAC_CHEVCTRL_EVACT_SUSPEND;
          }
        }
      }

      /// @brief 
      ///   This method sets the current state of the channel.
      /// @param state
      ///   DISABLED:
      ///      The channel cannot be activated by
      ///      a linked peripheral and any progress through the
      ///      transfer descriptors is lost. 
      ///   IDLE: 
      ///      The channel can be triggered by a linked
      ///      peripheral and progress through transfer descriptors
      ///      is preserved.
      ///    SUSPENDED: 
      ///      The channel cannot be triggered by
      ///      a linked peripheral and progress through transfer
      ///      descriptors is preserved.
      ///    ACTIVE: 
      ///      The channel is executing a transfer.
      /// @return 
      ///   True if the state was successfully changed,
      ///   and false otherwise.
      bool set_state(const channel_state &state) noexcept {
        auto ch_disable = [this]() {
          if (DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE) {
            DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE);
          }
          DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_NOACT_Val;
        };
        auto ch_enable = [this]() {
          if (!(DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE)) {
            DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
          }
        };
        if (get_state() == state) {
          return true;
        }
        switch(state) {
          case channel_state::disabled: {
            ch_disable();
            memset(&_wbdesc_[_index_], 0, sizeof(_DmacDescriptor_));
            DMAC->Channel[_index_].CHINTFLAG.reg = DMAC_CHINTFLAG_RESETVALUE;
            break;
          }
          case channel_state::idle: {
            ch_disable();
            DMAC->Channel[_index_].CHINTFLAG.reg |= DMAC_CHINTFLAG_SUSP;
            DMAC->Channel[_index_].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
            break;
          }
          case channel_state::suspended: {
            ch_enable();
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
            while(DMAC->Channel[_index_].CHINTFLAG.reg & DMAC_CHINTFLAG_SUSP);
            break;
          }
          case channel_state::active: {
            ch_enable();
            if (DMAC->Channel[_index_].CHINTFLAG.reg & DMAC_CHINTFLAG_SUSP) {
              DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
            } else {
              DMAC->SWTRIGCTRL.reg |= (1 << _index_);
            }
            DMAC->Channel[_index_].CHINTFLAG.reg = DMAC_CHINTFLAG_RESETVALUE;              
            break;
          }
          default: {
            return false;
          }
        }
        return true;
      }

      /// @brief 
      ///   Gets the current state of the channel.
      /// @return 
      ///   The current state of the channel as the enum
      ///   "channel_state".
      inline channel_state get_state() const noexcept {
        if (!DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE) {
          return channel_state::disabled;

        } else if (DMAC->Channel[_index_].CHINTFLAG.reg & DMAC_CHINTFLAG_SUSP) {
          return channel_state::suspended;

        } else if ((DMAC->Channel[_index_].CHSTATUS.reg
          & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND)) >= 1) {
          return channel_state::active;

        } else if (DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE) {
          return channel_state::idle;
        }
        return channel_state::unknown;
      }

      /// @brief 
      ///   Returns any errors that have occured
      ///   since the channel's state was last set to
      ///   disabled or active.
      /// @return 
      ///   An enum of type "channel_error" that denotes
      ///   the most signifigant error flagged for the channel
      ///   or "none" if no errors are presently flagged.  
      inline channel_error get_error() const noexcept {
        if (DMAC->Channel[_index_].CHINTFLAG.reg & DMAC_CHINTFLAG_TERR) {
          
          return (DMAC->Channel[_index_].CHSTATUS.reg & DMAC_CHSTATUS_FERR) 
            ? channel_error::task_error : channel_error::transfer_error; 

        } else if (DMAC->Channel[_index_].CHSTATUS.reg & DMAC_CHSTATUS_CRCERR) {
          return channel_error::crc_error;
        }
        return channel_error::none;
      }

      /// @brief 
      ///   This sets the transfer descriptor list for the channel.
      /// @param td_list 
      ///   The transfer descriptors in this list will be assigned to
      ///   this channel in the order given.
      /// @param looped 
      ///   If true the transfer descriptor descriptor list is linked
      ///   circularly, if false the last descriptor in the list
      //    is left unlinked.
      /// @return 
      ///   This method returns true if the descriptors where successfully
      ///   assigned to the channel, or false otherwise.
      /// @note
      ///   Any descriptors given that are assigned to annother channel
      ///   will be unassigned & removed from that channel's descriptor
      ///   list.
      /// @note
      ///   If any of the linked descriptors go out of scope or are deleted
      ///   they will be automatically removed from their assigned
      ///   channel's descriptor list.
      template<size_t count>
      bool set_descriptors(std::array<TransferDescriptor*, count> td_array,
        const bool &looped) noexcept {
        using enum channel_state;
        auto reset_properties = [](TransferDescriptor *desc) {
          if (desc) {
            desc->_descPtr_->DESCADDR.reg &= ~DMAC_DESCADDR_MASK;
            desc->_link_ = nullptr;
            desc->_cfg_._assigCH_ = -1;
          }
        };
        if (std::unique(td_array.begin(), td_array.end())) [[likely]] {
          channel_state state = get_state();

          if (state == active || state == idle) {
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
            while((DMAC->Channel[_index_].CHINTFLAG.reg 
              & DMAC_CHINTFLAG_SUSP) == 0);
          }
          if (_wbdesc_[_index_].SRCADDR.reg != DMAC_SRCADDR_RESETVALUE) {
            memset(&_wbdesc_[_index_], 0, sizeof(_DmacDescriptor_));
          }
          if (_btask_[_index_] && _btask_[_index_] != *td_array.begin()) {
            memcpy(&_btask_[_index_]->_desc_, &_bdesc_[_index_], 
              sizeof(_DmacDescriptor_));
            _btask_[_index_]->_descPtr_ = &_btask_[_index_]->_desc_;
            _btask_[_index_] = nullptr;
          }
          TransferDescriptor *curr_td = _btask_[_index_];
          TransferDescriptor *prev_td = looped ? *td_array.end() : nullptr;

          for (auto&& new_td : td_array) {       
            TransferDescriptor *temp = curr_td ? curr_td->_link_ : nullptr;
            new_td->update_valid();

            if (new_td != curr_td) { 
              reset_properties(curr_td);
              new_td->unlink();
             
              new_td->_cfg_._assigCH_ = _index_;
              if (prev_td) {
                prev_td->_link_ = new_td;
                prev_td->_descPtr_->DESCADDR.reg
                  = (uintptr_t)(new_td->_descPtr_);
              }
            }
            prev_td = new_td;
            curr_td = temp;
          }
          while(curr_td && curr_td != _btask_[_index_]) {
            reset_properties(curr_td);
          }
          if (!_btask_[_index_]) {
            memcpy(&_bdesc_[_index_], &(*td_list.begin())->_desc_, 
              sizeof(_DmacDescriptor_));
            (*td_list.begin())->_descPtr_ = &_bdesc_[_index_];
            _btask_[_index_] = (*td_list.begin());
          }
          if (state == idle) {
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_NOACT;
          } else if (state == active) {
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
          }
          return true;
        }
        return false;
      }

      /// @brief
      ///   This clears this channels transfer descriptor list.
      /// @note
      ///   This method sets the channel's state to "disabled".
      void clear_descriptors() noexcept {
        using enum channel_state;

        if (_btask_[_index_]) [[likely]] {
          if (get_state() != disabled) {
            DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg & DMAC_CHCTRLA_ENABLE);
          }
          TransferDescriptor *curr_td = _btask_[_index_];
          bool firstFlag = true;

          while(curr_td && (firstFlag || curr_td != _btask_[_index_])) {
            TransferDescriptor *temp = curr_td->_link_;
            firstFlag = false;

            curr_td->_descPtr_->DESCADDR.reg &= ~DMAC_DESCADDR_MASK;
            curr_td->_link_ = nullptr;
            curr_td->_cfg_._assigCH_ = -1;
            curr_td = temp;
          }
          memcpy(&_btask_[_index_]->_desc_, &_bdesc_[_index_],
            sizeof(_DmacDescriptor_));
          _btask_[_index_]->_descPtr_ = &_btask_[_index_]->_desc_;
          _btask_[_index_] = nullptr;
        }
      }

      /// @brief
      ///   This method returns the descriptor at a specified
      ///   index in the transfer descriptor list.
      /// @param index 
      ///   The index of the descriptor to get.
      /// @return 
      ///   A pointer to the descriptor at the specified index
      ///   if it exists. If not, a nullptr is returned.
      inline TransferDescriptor *get_descriptor(const int &index) noexcept {
        TransferDescriptor *curr = _btask_[_index_];
        int i = 0;
        while(curr && (i == 0 || curr != _btask_[_index_])) {
          if (i == index) {
            return curr;
          }
          curr = curr->_link_;
          i++;
        }
        return nullptr;
      }

      /// @brief
      ///   Sets the next descriptor to be executed by the channel.
      /// @param index
      ///   The index of the descriptor in this channel's
      ///   transfer descriptor list that will be executed when
      ///   the channel next enters an "active" state.
      /// @return
      ///   True if the given index was valid and the corresponding
      ///   descriptor was successfully queued, false otherwise.
      /// @note
      ///   If the channel is active when this method is called
      ///   it will be forcefully set to the "idle" state.
      bool queue_descriptor(const int &index) noexcept {
        using enum channel_state;
        if (_btask_[_index_]) [[likely]] {
          TransferDescriptor *targ = get_descriptor(index);
          if (targ) {
            channel_state state = get_state();            
            if (state == active || state == idle) {
              DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
              while(DMAC->Channel[_index_].CHCTRLB.reg 
                & DMAC_CHCTRLB_CMD_SUSPEND);
            }
            memcpy(&_wbdesc_[_index_], targ, sizeof(_DmacDescriptor_));
            if (state == active || state == idle) {
              DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_NOACT;
            }
            return true;
          }
        }
        return false;
      }
      

      /// @brief 
      ///   Gets the index of the last active transfer descriptor.
      /// @return 
      ///   An integer denoting the index of the last active transfer
      ///   descriptor in the transfer descriptor list.
      int get_current_index() const noexcept {
        if (_btask_[_index_]) [[likely]] {
          TransferDescriptor *curr = _btask_[_index_];
          bool first_flag = true;
          int index = 0;

          while(curr && (first_flag || curr != _btask_[_index_])) {
            first_flag = false;
            if (curr->_descPtr_->DESCADDR.reg 
              == _wbdesc_[_index_].DESCADDR.reg) {
              return index;
            }
            index++;
          }         
        }
        return -1;
      }

      /// @brief 
      ///   This method gets the number of elements/bytes remaining
      ///   the current transfer. 
      /// @param in_bytes 
      ///   If true, then this method returns the number of remaining
      ///   bytes left in the transfer, if false, then the number of
      ///   elements are returned -> based on the alignment of the
      ///   transfer.
      inline int get_remaining_elements(const bool &in_bytes = false) 
        noexcept {
        if (_btask_[_index_]) [[likely]] {
          return _wbdesc_[_index_].BTCNT.reg * in_bytes
            ? bs_ref.at(_wbdesc_[_index_].BTCTRL.reg 
            & DMAC_BTCTRL_BEATSIZE_Msk) : 1;
        }
        return -1;
      }

      /// @brief
      ///   This method updates the writeback (active) descriptor
      ///   so that any changes that were made to it's corresponding
      ///   reference (actual) descriptor are reflected in the ongoing
      ///   transfer.
      /// @note
      ///   This method is necessary because when the transfer associated
      ///   with a descriptor begins that descriptor is "captured" and
      ///   any changes made to the descriptor will not reflected in the
      ///   ongoing transfer.
      void update_writeback() noexcept {
        using enum channel_state;
        if (_btask_[_index_]) [[likely]] {
          channel_state state = get_state();

          if (state != suspended && state != disabled) {
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
            while((DMAC->Channel[_index_].CHCTRLB.reg
              & DMAC_CHCTRLB_CMD_SUSPEND) == 0);
          }
          TransferDescriptor *curr = _btask_[_index_];
          bool first_flag = true;

          while(curr && (first_flag || curr != _btask_[_index_])) {
            first_flag = false;
            if (curr->_descPtr_->DESCADDR.reg 
              == _wbdesc_[_index_].DESCADDR.reg) {
              auto cnt = _wbdesc_[_index_].BTCNT.reg;
              
              memcpy(&_wbdesc_[_index_], curr->_descPtr_, 
                sizeof(_DmacDescriptor_));
              _wbdesc_[_index_].BTCNT.reg = cnt;
              break;
            }
            curr = curr->_link_;
          }
          if (state == active) {
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
          } else if (state == idle) {
            DMAC->Channel[_index_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_NOACT;
          }
        }
      }

      /// @brief
      ///   This method returns true if the channel is currently
      ///   executing a transfer (state == active) or false otherwise.
      inline bool is_active() const noexcept {
        return DMAC->Channel[_index_].CHSTATUS.reg
          & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND);
      }

      /// @brief
      ///   This method links a peripheral trigger to this channel.
      /// @param value
      ///   This is an enum ("channel_peripheral") that denotes the
      ///   peripheral trigger type to set for this channel.
      /// @note
      ///   To remove any currently set peripheral triggers use the
      ///   channel_peripheral value of "none".
      /// @note
      ///   Once a peripheral trigger is linked to the channel,
      ///   that peripheral will be able to set that channel's state to "active"
      ///   unless the channel's state is set to "suspend" or "disabled".
 

      /// @brief
      ///   This method returns an enum of type "channel_peripheral"
      ///   denoting the peripheral trigger type currently linked
      ///   to this channel.
      /// @note
      ///   If no peripherals have been linked this method will return
      ///   the enum channel_peripheral::none.
      inline channel_peripheral get_linked_peripheral() 
        const noexcept {
        auto reg_v = DMAC->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_TRIGSRC_Msk;
        return static_cast<channel_peripheral>(reg_v);
      }

      /// @brief
      ///   Sets the callback function to be used/invoked by this channel.
      /// @param callback
      ///   A non-member function pointer to a function with no return
      ///   (void), that takes in order: an integer and an enum of type
      ///   "callback_flag".
      /// @note
      /// - To disable the IRQ callback set the callback to a nullptr
      /// @note
      /// - The integer passed to the callback method denotes the index
      ///   of the channel sending the callback and the "callback_flag"
      ///   parameter denotes the callback source/purpose {channel_error,
      ///   transfer_complete}. 
      inline void set_irq_callback(callback_t callback) noexcept {
        _cbptr_[_index_] = callback;
      }

      /// @brief
      ///   This method set the channel's transfer mode
      ///   which determines how the channel executes transfer
      ///   descriptors.
      /// @param mode
      ///   packet -> When set to active the channel will only
      ///     transfer a single packet worth of data before returning
      ///     to an "idle" state.
      ///   descriptor -> When set to active the channel will
      ///     execute the next transfer descriptor before returning
      ///     to an "idle" state.
      ///   all -> When set to active the channel will execute all
      ///     available transfer descriptors before returning to an
      ///     idle state.
      /// @return
      ///   This method returns true if the given transfer mode was
      ///   valid and it was successfully set, or false otherwise.
      /// @warning
      ///   If the transfer descriptors assigned to this channel are
      ///   looped, setting the transfer mode to all will cause the
      ///   DMA continue executing them indefinetly.
      inline bool set_transfer_mode(const transfer_mode &mode) noexcept {
        using enum_t = std::underlying_type_t<transfer_mode>;
        DMAC->Channel[_index_].CHCTRLA.reg &= ~DMAC_CHCTRLA_TRIGACT_Msk;
        DMAC->Channel[_index_].CHCTRLA.reg |= static_cast<enum_t>(mode);
      }

      /// @brief
      ///   This method returns an enum of type "transfer_mode"
      ///   that denotes the current transfer mode set for this
      ///   channel 
      /// @note
      /// - For more information about the meaning of different transfer
      ///   modes, see the method above - "set_transfer_mode()"
      inline transfer_mode get_transfer_mode() const noexcept {
        auto reg_v = DMAC->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_TRIGACT_Msk;
        return static_cast<transfer_mode>(reg_v);
      }

      /// @brief
      ///   This method sets the packet size of the channel.
      /// @param elements
      ///   This is an integer that denotes the number of elements
      ///   to transfer at a time (in a single packet - the smallest
      ///    "transfer unit").
      /// @return
      ///   This method returns true if the given packet size was
      ///   valid and set, or false otherwise.
      /// @note
      /// - The size, in bytes of a single packet is determined
      ///   on a transfer-by-transfer basis by it's associated
      ///   transfer descriptor.
      bool set_packet_size(const int &elements) noexcept {       
        using reg_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);

        reg_t bus_r = std::distance(bus_ref.begin(), 
          std::find(bus_ref.begin(), bus_ref.end(), elements));

        if (bus_r < bus_ref.size()) [[likely]] {
          reg_t thresh_r = std::distance(thresh_ref.begin(),
            std::find_if(thresh_ref.begin(), thresh_ref.end(),
            [&bus_r](int i){ return !(bus_ref[bus_r] % i); }));

          if (thresh_r < thresh_ref.size()) {
            DMAC->Channel[_index_].CHCTRLA.reg 
              &= ~(DMAC_CHCTRLA_BURSTLEN_Msk 
              | DMAC_CHCTRLA_THRESHOLD_Msk);

            DMAC->Channel[_index_].CHCTRLA.reg 
              |= (DMAC_CHCTRLA_BURSTLEN(bus_r) 
              | DMAC_CHCTRLA_THRESHOLD(thresh_r));
            return true;
          }
        }
        return false;
      }

      /// @brief
      ///   This method returns an integer denoting the packet
      ///   size set for this channel.
      /// @note
      /// - The size, in bytes of a single packet is determined
      ///   on a transfer-by-transfer basis by it's associated
      ///   transfer descriptor.
      inline int get_packet_size() const noexcept {
        return bus_ref.at(DMAC->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_BURSTLEN_Msk);
      }

      /// @brief 
      ///   This method sets the priority level of transfers executed
      ///   by this channel.
      /// @param lvl 
      ///   This is an integer that denotes the priority lvl (1-4 
      ///   -> higher = greater priority) of transfers executed by 
      ///   this channel.
      /// @return 
      ///   True if the given priority lvl is valid and was succesfully
      ///   set, or false otherwise.
      bool set_priority_lvl(const int &lvl) noexcept {
        using reg_t = decltype(std::declval<DMAC_CHPRILVL_Type>().reg);

        reg_t lvl_r = std::distance(chpri_ref.begin(), 
          std::find(chpri_ref.begin(), chpri_ref.end(), lvl));
        
        if (lvl_r < chpri_ref.size()) [[likely]] {
          DMAC->Channel[_index_].CHPRILVL.reg &= ~DMAC_CHPRILVL_MASK;
          DMAC->Channel[_index_].CHPRILVL.reg |= DMAC_CHPRILVL_PRILVL(lvl_r);
          return true;
        }
        return false;
      }

      /// @brief
      ///   This method returns an integer that denotes this channel's
      ///   currentl priority lvl (1-4 -> higher = greater priority).
      int get_priority_lvl() const noexcept {
        return chpri_ref.at(DMAC->Channel[_index_].CHPRILVL.reg
          & DMAC_CHPRILVL_MASK);
      }

      /// @internal
      ///   Determines the index of the accessed channel
      const int _index_;
    }channel[SIO_DMA_CHANNELS]{{._index_ = []{static int counter = 0; 
      return counter++;}()}};

  }dma;
  

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER_DESCRIPTOR
///////////////////////////////////////////////////////////////////////////////////////////////////

  /// @class
  ///   This object is used to describe a single transfer 
  ///   which can be executed by a DMA channel. 
  class TransferDescriptor {
    friend struct channel_ctrl;
    public:

      /// @brief
      ///   Default constructor
      TransferDescriptor() = default;

      /// @brief
      ///   Copy constructor.
      /// @param other_descriptor
      ///   This is trasnfer descriptor that will be copied.
      /// @note
      /// - The channel that the other given transfer descriptor
      ///   is assigned to will NOT be copied.
      TransferDescriptor(const TransferDescriptor &other) {
        this->operator = (other);
      }

      /// @brief
      ///   This is the copy assignment operator
      /// @param other_descriptor
      ///   This denotes the other transfer descriptor object that
      ///   will be copied.
      /// @note
      /// - The channel that the other given transfer descriptor
      ///   is assigned to will NOT be copied.
      TransferDescriptor operator = (const TransferDescriptor &other) {
        using enum channel_state;
        channel_state state = unknown;
        if (_cfg_._assigCH_ != -1) {
          state = dma.channel[_cfg_._assigCH_].get_state();

          if (state != disabled && state != suspended) {
            DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg 
              |= DMAC_CHCTRLB_CMD_SUSPEND;
            while((DMAC->Channel[_cfg_._assigCH_].CHINTFLAG.reg 
              & DMAC_CHINTFLAG_SUSP) == 0);
          }
        }
        auto prev_l = _descPtr_->DESCADDR.reg;
        memcpy(_descPtr_, other._descPtr_, sizeof(_DmacDescriptor_));
        _descPtr_->DESCADDR.reg = prev_l;

        if (state == active) {
          DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
        } else if (state == idle) {
          DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_NOACT;
        }
      }

      /// @brief
      ///   Returns a bool that denotes whether or not the descriptor
      ///   has been set to "valid". 
      /// @note
      /// - A descriptor is considered "valid" if it has a non-zero
      ///   source, destination and transfer length. 
      bool get_valid() const noexcept {
        update_valid();
        return (bool)(_descPtr_->BTCTRL.reg & DMAC_BTCTRL_VALID);
      }
      explicit operator bool() const noexcept {
        return get_valid();
      }

      /// @brief
      ///   This method sets the source of the transfer described by
      ///   this descriptor object.
      /// @param source_pointer
      ///   This is a pointer of arbitrary type that points to the source
      ///   location for the transfer. Note if the source pointer type is
      ///   volatile, no modifier will be applied to it's address.
      /// @param increment
      ///   This is an integer that denotes how much the src_ptr should
      ///   be incremented between every time an element is transfered. 
      ///   Note that an increment of 1 denotes that the src_ptr should 
      ///   be incremented by 1x the size of the alignment set for this
      ///   transfer (by "set_alignment").
      /// @return 
      ///   This method returns true if the given src_ptr and increment 
      ///   configuration are valid and the source location for the transfer 
      ///   was successfully set. Otherwise the method returns false.
      /// @note 
      /// - Only 1 of the transfer's locations (either the source or 
      ///   destination) can have an increment size greater then 1.
      /// @note
      /// - The source range for the transfer CANNOT overlap with the
      //    destination range for the transfer (range = ptr address 
      ///   + transfer size).
      template<typename src_t>
      bool set_source(const src_t &src_ptr, const int &incr) noexcept {
        if (incr > 0) {
          _descPtr_->BTCTRL.reg |= DMAC_BTCTRL_SRCINC;
          if (incr > 1) {
            if ((_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL)
                == DMAC_BTCTRL_STEPSEL_DST_Val
              && (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSIZE_Msk)
                > DMAC_BTCTRL_STEPSIZE_X1_Val) {
              return false;
            }
            auto step_r = std::distance(ss_ref.begin(), 
              std::find(ss_ref.begin(), ss_ref.end(), incr));
            if (step_r >= ss_ref.size()) {
              return false;
            }
            _descPtr_->BTCTRL.reg |= DMAC_BTCTRL_STEPSEL;
            _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
            _descPtr_->BTCTRL.reg 
              |= (step_r << DMAC_BTCTRL_STEPSIZE_Pos);

          } else if ((_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL)
            == DMAC_BTCTRL_STEPSEL_SRC_Val) {
            _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
          }
        } else {
          _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_SRCINC;
        }
        _cfg_.src_vol = std::is_volatile_v<src_t>;
        _descPtr_->SRCADDR.reg = (uintptr_t)src_ptr + addr_mod(true);
        update_valid();
        return true;
      } 

      /// @brief
      ///   This method returns a pointer to the source of the transfer 
      ///   described by this object. If not source has been set, this 
      ///   method returns a void pointer.
      inline void *get_source() const noexcept {
        if (_descPtr_->SRCADDR.reg == DMAC_SRCADDR_RESETVALUE) {
          return nullptr;
        }
        return (void*)(_descPtr_->SRCADDR.reg - addr_mod(true));
      }

      /// @brief
      ///   This method sets the destination for the transfer described
      ///   by this descriptor object.
      /// @param destination_pointer
      ///   This is a pointer of arbitrary type that points to the destination
      ///   of the transfer. Note, if the destination ptr type is volatile, no
      ///   modifier will be applied to the address (use for registers).
      /// @param increment
      ///   This is an integer that denotes how much the dst_ptr should
      ///   be incremented every time an element is transfered. Note that
      ///   an increment of 1 denotes that the dst_ptr should be 
      ///   incremented by 1x the size of the alignment set for this
      ///   transfer (by "set_alignment").
      /// @return
      ///   This method returns true if the given dst_ptr and increment 
      ///   configuration are valid and the destination location for the 
      ///   transfer was successfully set. Otherwise the method returns false.
      /// @note
      /// - Only 1 of the transfer's locations (source/destination) can 
      ///   have an increment size greater then 1.
      /// @note
      /// - The destination range for the transfer CANNOT overlap with the
      ///   source range for the transfer (range = ptr address + transfer size).
      template<typename dst_t>
      bool set_destination(const dst_t &dst_ptr, const int &incr) noexcept {
        if (incr > 0) {
          _descPtr_->BTCTRL.reg |= DMAC_BTCTRL_DSTINC;
          if (incr > 1) {
            if ((_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL)
                == DMAC_BTCTRL_STEPSEL_SRC_Val
              && (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSIZE_Msk)
                > DMAC_BTCTRL_STEPSIZE_X1_Val) {
              return false;
            }
            auto step_r = std::distance(ss_ref.begin(), 
              std::find(ss_ref.begin(), ss_ref.end(), incr));
            if (step_r >= ss_ref.size()) {
              return false;
            }
            _descPtr_->BTCTRL.reg 
              &= ~(DMAC_BTCTRL_STEPSEL | DMAC_BTCTRL_STEPSIZE_Msk);
            _descPtr_->BTCTRL.reg 
              |= (step_r << DMAC_BTCTRL_STEPSIZE_Pos);

          } else if ((_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL)
            == DMAC_BTCTRL_STEPSEL_DST_Val) {
            _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_STEPSIZE_Msk;
          }
        } else {
          _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_SRCINC;
        }
        _cfg_.dst_vol = std::is_volatile_v<dst_t>;
        _descPtr_->DSTADDR.reg = (uintptr_t)dst_ptr + addr_mod(false);
        update_valid();
        return true;
      }

      /// @brief
      ///   This method returns a pointer to the destination of the 
      ///   transfer described by this object. If not destination has 
      ///   been set, this method returns a void pointer.
      inline void *get_destination() const noexcept {
        return (void*)(_descPtr_->DSTADDR.reg - addr_mod(false));
      }

      /// @brief
      ///   This method removes this transfer descriptor from the channel
      ///   that it is currently assigned to (if applicable).
      /// @note
      /// - This method ensures that both the transfer descriptor list of
      ///   the assigned channel and that channels progress through the descriptor
      ///   list is uneffected by this action.
      /// @note
      /// - This method is called when this object is destructed.
      void unlink() noexcept {
        if (_cfg_._assigCH_ != -1) {
          bool wbFlag = false;
          TransferDescriptor *curr = _btask_[_cfg_._assigCH_];

          if (_link_ != this) {
            wbFlag = (_wbdesc_[_cfg_._assigCH_].DESCADDR.reg 
              == (uintptr_t)_descPtr_);
            while(curr->_link_ && curr->_link_ != _btask_[_cfg_._assigCH_]) {
              curr = curr->_link_;
            }
          }
          // Handle this = base task
          if (this == _btask_[_cfg_._assigCH_]) {
            memcpy(&_desc_, &_bdesc_[_cfg_._assigCH_], 
              sizeof(_DmacDescriptor_));
            _descPtr_ = &_desc_;

            if (!_link_ || _link_ == this) {
              memset(&_bdesc_[_cfg_._assigCH_], 0, sizeof(_DmacDescriptor_));
              _btask_[_cfg_._assigCH_] = nullptr;
            } else {
              memcpy(&_bdesc_[_cfg_._assigCH_], &_link_->_desc_, 
                sizeof(_DmacDescriptor_));
              _link_->_descPtr_ = &_bdesc_[_cfg_._assigCH_];
              _btask_[_cfg_._assigCH_] = _link_;     
            }
          }
          if (curr) {
            curr->_link_ = _link_;
            curr->_descPtr_->DESCADDR.reg = _descPtr_->DESCADDR.reg;
            
            // Update writeback descriptor (if applicable)
            if (wbFlag && _wbdesc_[_cfg_._assigCH_].DESCADDR.reg 
              == (uintptr_t)_descPtr_) {
              using enum channel_state;
              channel_state state = dma.channel[_cfg_._assigCH_].get_state();

              if (state != disabled && state != suspended) {
                DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg 
                  |= DMAC_CHCTRLB_CMD_SUSPEND;
                while((DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg
                  & DMAC_CHCTRLB_CMD_SUSPEND) == 0);
              }
              auto cnt = _wbdesc_[_cfg_._assigCH_].BTCNT.reg;
              memcpy(&_wbdesc_[_cfg_._assigCH_], _descPtr_,
                sizeof(_DmacDescriptor_));
              _wbdesc_[_cfg_._assigCH_].BTCNT.reg = cnt;

              if (state == active) {
                DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg 
                  |= DMAC_CHCTRLB_CMD_RESUME;
              } else if (state == idle) {
                DMAC->Channel[_cfg_._assigCH_].CHCTRLB.reg 
                  |= DMAC_CHCTRLB_CMD_NOACT;
              }
            }
          }
          _descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
          _link_ = nullptr;
          _cfg_._assigCH_ = -1;
        }
      }

      /// @brief
      ///   This method returns a pointer to the transfer descriptor object 
      ///   that this descriptor is linked to. Note that the transfer 
      ///   descriptors are "linked" when they are assigned to a channel in 
      //    a particular order.
      inline TransferDescriptor *get_linked_descriptor() noexcept {
        return _link_;
      }

      /// @brief
      ///   This method sets the alignment of the channel in bytes.
      /// @param bytes
      ///   This is an integer that denotes the size of the elements
      ///   transfered. Valid sizes are {1, 2, 4}
      /// @note
      /// - This alignment value also determines the size of each increment
      ///   given for the source/destination locations of this transfer.
      bool set_alignment(const int &bytes) noexcept {
        auto beatsize_r = std::distance(bs_ref.begin(),
          std::find(bs_ref.begin(), bs_ref.end(), bytes));

        if (beatsize_r < bs_ref.size()) {
          uintptr_t src_addr = _descPtr_->SRCADDR.reg - addr_mod(true);
          uintptr_t dst_addr = _descPtr_->DSTADDR.reg - addr_mod(false);
          _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_BEATSIZE_Msk;
          _descPtr_->BTCTRL.reg 
            |= (beatsize_r << DMAC_BTCTRL_BEATSIZE_Pos);
          
          update_valid();
          _descPtr_->SRCADDR.reg = src_addr + addr_mod(true);
          _descPtr_->DSTADDR.reg = dst_addr + addr_mod(false);
          return true;
        }
        return false;
      }

      /// @brief 
      ///   This method returns an integer denoting the current alignment
      ///   configuration set for this transfer.
      inline int get_alignment() const noexcept {
        return (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk);
      }

      /// @brief
      ///   This method sets the overall length of the transfer.
      /// @param elements
      ///   This is an integer that denotes the overall number of 
      ///   elements to transfer.
      /// @note
      /// - The size of a single element refered to by the "length" of the
      ///   transfer is determined by the alignment config set for the
      ///   transfer.
      inline void set_length(const decltype(std::declval<DMAC_BTCNT_Type>().reg) 
        &elements) noexcept {
        uintptr_t src_addr = _descPtr_->SRCADDR.reg - addr_mod(true);
        uintptr_t dst_addr = _descPtr_->DSTADDR.reg - addr_mod(false);

        _descPtr_->BTCNT.reg = elements;        
        update_valid();
        _descPtr_->SRCADDR.reg = src_addr + addr_mod(true);
        _descPtr_->DSTADDR.reg = dst_addr + addr_mod(false);
      }

      /// @brief 
      ///   This method gets the overall length of the transfer.
      /// @return 
      ///   An integer denoting the number of elements to be transfered.
      /// @note
      /// - The size of each "element" in bytes is denoted by the
      ///   alignment configuration set for the channel.
      inline const decltype(std::declval<DMAC_BTCNT_Type>().reg) 
        &get_length() const noexcept {
        return _descPtr_->BTCNT.reg * bs_ref.at(_descPtr_->BTCTRL.reg
          & DMAC_BTCTRL_BEATSIZE_Msk);
      }

      /// @brief
      ///   Destroys transfer descriptor and unlinks
      ///   it from it's assigned channel (if applicable)
      ~TransferDescriptor() {
        unlink();
      }

    private:
      friend dma_fn;
      struct {
        int8_t _assigCH_ = -1; // Denotes assigned channel num, -1 if no assigned channel
        bool src_vol = false;  // True if the last given source was volatile (i.e. no modifier) 
        bool dst_vol = false;  // True if the last given dest was volatile (i.e. no modifier)
      }_cfg_ __PACKED __aligned(1);
      TransferDescriptor *_link_ = nullptr;  // Pointer to linked descriptor
      _DmacDescriptor_ *_descPtr_ = &_desc_; // Pointer to controlled DMAC descriptor
      _DmacDescriptor_ _desc_;               // Allocated DMAC descriptor (RAII)

      /// @internal
      /// @brief
      ///   This method gets the modifier that is applied to an
      ///   an address based on the current configuration of the transfer.
      /// @param src
      ///   True = modifier for source, false = modifier for destination.
      uintptr_t addr_mod(const bool &src) const noexcept {
        if ((src && !_cfg_.src_vol) || (!src && !_cfg_.dst_vol)) {
          bool sel = (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_STEPSEL) == (src 
            ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val);

          return _descPtr_->BTCNT.reg 
            * (_descPtr_->BTCTRL.bit.BEATSIZE + 1) 
            * (sel ? (1 << _descPtr_->BTCTRL.bit.STEPSIZE) : 1);
        }
        return 0;
      }

      /// @internal
      /// @brief
      ///   This method updates the "valid" configuration of the channel
      ///   based on whether the source, destination and size of the transfer
      ///   has been set.
      void update_valid() const noexcept {
        bool valid = _descPtr_->SRCADDR.reg && _descPtr_->DSTADDR.reg
          && _descPtr_->BTCNT.reg;
        if (valid) {
          _descPtr_->BTCTRL.reg |= DMAC_BTCTRL_VALID;
        } else {
          _descPtr_->BTCTRL.reg &= ~DMAC_BTCTRL_VALID;
        }
      }

  };

  /// @internal
  /// @brief 
  ///   Interrupt handler function for DMAC that delegates call 
  ///   to assigned callback functions & clears relevent flags.
  [[interrupt("IRQ"), nesting]]
  void DMAC_0_Handler() {
    using enum callback_flag;
    int src = (DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk);
    if (src >= 0 && src < SIO_DMA_CHANNELS) [[likely]] {
      if (_cbptr_[src]) {
        if (DMAC->INTPEND.reg & DMAC_INTPEND_TCMPL) {
          DMAC->Channel[src].CHINTFLAG.reg &= ~DMAC_CHINTFLAG_TCMPL;
          _cbptr_[src](src, transfer_complete);

        } else if (DMAC->INTPEND.reg & DMAC_INTPEND_TERR) {
          DMAC->Channel[src].CHINTFLAG.reg &= ~DMAC_CHINTFLAG_TERR;        
          _cbptr_[src](src, channel_error);
        }
      } else {
        DMAC->Channel[src].CHINTFLAG.reg &= ~(DMAC_CHINTFLAG_TCMPL
          | DMAC_CHINTFLAG_TERR);
      }
    }
  }
  void DMAC_1_Handler() [[interrupt("IRQ")]] { DMAC_0_Handler(); }
  void DMAC_2_Handler() [[interrupt("IRQ")]] { DMAC_0_Handler(); }
  void DMAC_3_Handler() [[interrupt("IRQ")]] { DMAC_0_Handler(); }
  void DMAC_4_Handler() [[interrupt("IRQ")]] { DMAC_0_Handler(); } 

} /// END OF NAMESPACE SIO::DETAIL

*/
