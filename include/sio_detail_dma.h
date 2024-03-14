
#include <sam.h>
#include <tuple>
#include <utility>
#include <algorithm>
#include <iterator>
#include <array>
#include <numeric>
#include <string.h>

#define SIO_DMA_CHANNELS 16
#define SIO_DEF_IRQ_PRIORITY 1

namespace sio::detail {

  namespace {  

    /// DESCRIPTOR TYPEDEF
    __aligned(16)
    __section(".hsram")
    typedef struct {
      DMAC_BTCTRL_Type BTCTRL;
      DMAC_BTCNT_Type BTCNT;
      DMAC_SRCADDR_Type SRCADDR;
      DMAC_DSTADDR_Type DSTADDR;
      DMAC_DESCADDR_Type DESCADDR;
    }_DmacDescriptor_;

    /// GLOBAL
    constexpr std::array<int, 3> bs_ref = {1, 2, 4};  
    constexpr std::array<int, 8> ss_ref = {1, 2, 4, 8, 16, 32, 62, 128};
    constexpr std::array<int, 16> bus_ref = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    constexpr std::array<int, 4> thresh_ref = {1, 2, 4, 8};
    constexpr std::array<int, 4> chpri_ref = {0, 1, 2, 3};

    /// GLOBAL VARIABLES
    static _DmacDescriptor_ _wbdesc_[DMAC_CH_NUM]{};
    static _DmacDescriptor_ _bdesc_[DMAC_CH_NUM]{};
    static TransferDescriptor *_btask_[DMAC_CH_NUM] = { nullptr };
    static uint32_t _allocmsk_ = 0;
    static uint32_t _loopmsk_ = 0;
    
    // Used exclusively for initialization
    static int init_index = 0;

  }

  enum class channel_state {
    unknown,
    disabled,
    idle,
    suspended,
    active
  };
  enum class channel_error {
    none,
    transfer_error,
    task_error,
    crc_error,
    invalid_index
  };
  enum class transfer_mode : int {
    packet  = DMAC_CHCTRLA_TRIGACT_BURST_Val,
    task    = DMAC_CHCTRLA_TRIGACT_BLOCK_Val,
    all     = DMAC_CHCTRLA_TRIGACT_TRANSACTION_Val
  };
  enum class channel_peripheral : int {
    none    = DMAC_CHCTRLA_TRIGSRC_DISABLE_Val,
    adc     = 1 
  };



  struct {

    class TransferDescriptor;


    /// \b TO_DO

    struct {

      struct final {
        uint8_t prilvl_irq_priority[DMAC_LVL_NUM] = { 1 }; 
        bool prilvl_rr_arb[DMAC_LVL_NUM] = { false };
        uint8_t prilvl_service_quality[DMAC_LVL_NUM] = { 2 };
      }config;

      ///\b V1_COMPLETE
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
              SIO_DEF_IRQ_PRIORITY);
          } else {
            NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), 
              config.prilvl_irq_priority[i]);
            NVIC_EnableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));

            DMAC->PRICTRL0.reg |=
              (config.prilvl_rr_arb[i] 
              << (DMAC_PRICTRL0_RRLVLEN0_Pos + i * 8)) |
              (config.prilvl_service_quality[i] 
              << (DMAC_PRICTRL0_QOS0_Pos + i * 8));
          }
        }
        if (initialized) {
          DMAC->BASEADDR.reg = (uintptr_t)_bdesc_;
          DMAC->WRBADDR.reg = (uintptr_t)_wbdesc_;
          MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
        }
      }

      ///\b V1_COMPLETE
      inline bool get_init() const noexcept {
        return (DMAC->BASEADDR.reg == (uintptr_t)_bdesc_);
      }

      ///\b V1_COMPLETE
      bool set_enabled(const bool &enabled) noexcept {
        if (enabled) {
          DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
        } else {
          DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
            while(DMAC->CTRL.reg & DMAC_CTRL_DMAENABLE);
        }
      }

      ///\b V1_COMPLETE
      inline bool get_enabled() const noexcept {
        return (DMAC->CTRL.reg & DMAC_CTRL_DMAENABLE);
      }


    }system;


      };


    /// \b TO_DO

    struct {

    }crc;


    /// \b TO_DO

    struct {

    }active_channel;


    

    static struct channel_ctrl {

      /// \b V1_COMPLETE
      inline bool reset_transfer(void) noexcept {
        auto state = get_state();
        if (state != channel_state::unknown) {
          bool susp = (state == channel_state::suspended);
          bool enabled = (state != channel_state::disabled);

          if (enabled) {
            DMAC->Channel[_index_].CHCTRLA.reg 
              &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg 
              & DMAC_CHCTRLA_ENABLE);
          }
          DMAC->Channel[_index_].CHINTFLAG.reg
            &= ~DMAC_CHINTFLAG_RESETVALUE;
          memset(&_wbdesc_[_index_], 0, sizeof(_wbdesc_[0]));

          if (enabled) {
            DMAC->Channel[_index_].CHCTRLA.reg
              |= DMAC_CHCTRLA_ENABLE;
            if (susp) {
              DMAC->Channel[_index_].CHCTRLB.reg 
                |= DMAC_CHCTRLB_CMD_SUSPEND;
            } 
          }
          return true;
        }
        return false;
      }

      /// \b V2_COMPLETE
      inline void reset_channel() noexcept {
        auto state = get_state();
        bool enabled, susp;
        
        if (state != channel_state::unknown) {
          susp = (state == channel_state::suspended);

          if (state != channel_state::disabled) {
            enabled = true;
            DMAC->Channel[_index_].CHCTRLA.reg 
              &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg 
              & DMAC_CHCTRLA_ENABLE);
          }
        }
        DMAC->Channel[_index_].CHCTRLA.reg 
          |= DMAC_CHCTRLA_SWRST;
        while(DMAC->Channel[_index_].CHCTRLA.reg
          & DMAC_CHCTRLA_SWRST);

        clear_descriptors();
        memset(&_wbdesc_[_index_], 0, sizeof(_wbdesc_[0]));

        if (enabled) {
          DMAC->Channel[_index_].CHCTRLA.reg
            |= DMAC_CHCTRLA_ENABLE;
          if (susp) {
            DMAC->Channel[_index_].CHCTRLB.reg
              |= DMAC_CHEVCTRL_EVACT_SUSPEND;
          }
        }
      }

      /// \b V1_COMPLETE -> NEEDS VALID DESCRIPTOR CHECKING!!!
      bool set_state(const channel_state &state) noexcept {
        auto ch_disable = [this]() {
          if (DMAC->Channel[_index_].CHCTRLA.reg
            & DMAC_CHCTRLA_ENABLE) {

            DMAC->Channel[_index_].CHCTRLA.reg 
              &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg
              & DMAC_CHCTRLA_ENABLE);
          }
          DMAC->Channel[_index_].CHCTRLB.reg 
            |= DMAC_CHCTRLB_CMD_NOACT_Val;
        };
        auto ch_enable = [this]() {
          if (!(DMAC->Channel[_index_].CHCTRLA.reg
            & DMAC_CHCTRLA_ENABLE)) {
            DMAC->Channel[_index_].CHCTRLA.reg
              |= DMAC_CHCTRLA_ENABLE;
          }
        };
        auto ready_descriptor = [this]() {
          /// \b TO_DO
        };
        if (get_state() == state) {
          return true;
        }
        switch(state) {
          case channel_state::disabled: {
            ch_disable();
            DMAC->Channel[_index_].CHINTFLAG.reg
              |= DMAC_CHINTFLAG_RESETVALUE;
            break;
          }
          case channel_state::idle: {
            ch_disable();
            DMAC->Channel[_index_].CHINTFLAG.reg
              |= DMAC_CHINTFLAG_SUSP;
            DMAC->Channel[_index_].CHCTRLA.reg
              |= DMAC_CHCTRLA_ENABLE;
            break;
          }
          case channel_state::suspended: {
            ch_enable();
            DMAC->Channel[_index_].CHCTRLB.bit.CMD 
              = DMAC_CHCTRLB_CMD_SUSPEND_Val;
            while(DMAC->Channel[_index_].CHINTFLAG.reg
              & DMAC_CHINTFLAG_SUSP);
            break;
          }
          case channel_state::active: {
            ch_enable();
            if (DMAC->Channel[_index_].CHINTFLAG.reg
              & DMAC_CHINTFLAG_SUSP) {
              DMAC->Channel[_index_].CHCTRLB.reg
                |= DMAC_CHCTRLB_CMD_RESUME;
            } else {
              DMAC->SWTRIGCTRL.reg |= (1 << _index_);
            }
            DMAC->Channel[_index_].CHINTFLAG.reg 
              = DMAC_CHINTFLAG_RESETVALUE;              
            break;
          }
          default: {
            return false;
          }
        } // End of switch
        return true;
      }

      /// \b V1_COMPLETE
      inline channel_state get_state() const noexcept {
        if (!DMAC->Channel[_index_].CHCTRLA.reg
          & DMAC_CHCTRLA_ENABLE) {
          return channel_state::disabled;

        } else if (DMAC->Channel[_index_].CHINTFLAG.reg
          & DMAC_CHINTFLAG_SUSP) {
          return channel_state::suspended;

        } else if ((DMAC->Channel[_index_].CHSTATUS.reg
          & (DMAC_CHSTATUS_BUSY | DMAC_CHSTATUS_PEND)) >= 1) {
          return channel_state::active;

        } else if (DMAC->Channel[_index_].CHCTRLA.reg
          & DMAC_CHCTRLA_ENABLE) {
          return channel_state::idle;
        }
        return channel_state::unknown;
      }

      /// \b V1_COMPLETE
      inline channel_error get_error() const noexcept {
        if (DMAC->Channel[_index_].CHINTFLAG.reg
          & DMAC_CHINTFLAG_TERR) {
          
          return (DMAC->Channel[_index_].CHSTATUS.reg 
            & DMAC_CHSTATUS_FERR) ? channel_error::task_error 
            : channel_error::transfer_error; 

        } else if (DMAC->Channel[_index_].CHSTATUS.reg
          & DMAC_CHSTATUS_CRCERR) {
          return channel_error::crc_error;
        }
        return channel_error::none;
      }

      ///\b V1_COMPLETE
      bool set_descriptors(std::initializer_list<TransferDescriptor*> 
        td_list, const bool &looped) noexcept {
        using enum channel_state;
        
        auto reset_properties = [&](TransferDescriptor *desc) {
          desc->_descPtr_->DESCADDR.reg &= ~DMAC_DESCADDR_MASK;
          desc->_link_ = nullptr;
          desc->_assigCH_ = -1;
        };
        // Ensure all given descriptors are unique
        if (std::unique(td_list.begin(), td_list.end())) {
          TransferDescriptor *curr_td = _btask_[_index_];   
          TransferDescriptor *saved_td = nullptr;
          channel_state state = get_state();

          // Ensure that channel can't be activated
          if (state == active || state == idle) {
            DMAC->Channel[_index_].CHCTRLB.reg 
              |= DMAC_CHCTRLB_CMD_SUSPEND;
            while(DMAC->Channel[_index_].CHINTFLAG.reg
              & DMAC_CHINTFLAG_SUSP);
          }
          // Remove old transfer descriptors
          if (curr_td != *td_list.begin()) {
            memcpy(&curr_td->_desc_, &_btask_[_index_],
              sizeof(_DmacDescriptor_));
            curr_td->_descPtr_ = &curr_td->_desc_;
            _btask_[_index_] = nullptr;
          }
          auto list_iter = td_list.begin();
          bool first_flag = true;
          curr_td = _btask_[_index_];
          saved_td = looped ? *td_list.end() : nullptr;

          while(list_iter != td_list.end()) {
            first_flag = false;
            TransferDescriptor *temp = curr_td 
              ? curr_td->_link_ : nullptr; 

            if (*list_iter != curr_td) { 
              curr_td->unlink();
              (*list_iter)->_assigCH_ = _index_;

              if (saved_td) {
                saved_td->_link_ = (*list_iter);
                saved_td->_descPtr_->DESCADDR.reg 
                  = (uintptr_t)((*list_iter)->_descPtr_);
              }
            }
            saved_td = (*list_iter);
            std::advance(list_iter, 1);
            curr_td = saved_td;
          }
          while(curr_td && curr_td != _btask_[_index_]) {
            curr_td->_assigCH_ = -1;
            curr_td->_link_ = nullptr;
            curr_td->_descPtr_->DESCADDR.reg 
              &= ~DMAC_DESCADDR_MASK;
          }
          // Set NEW base transfer descriptor
          if (!_btask_[_index_]) {
            memcpy(&_bdesc_[_index_], &(*td_list.begin())->_desc_,
              sizeof(_DmacDescriptor_));
              
            (*td_list.begin())->_descPtr_ = &_bdesc_[_index_];
            _btask_[_index_] = (*td_list.begin());
          }
          // Return channel to original state
          if (state == idle) {
            DMAC->Channel[_index_].CHCTRLB.reg
              |= DMAC_CHCTRLB_CMD_NOACT;
          } else if (state == active) {
            DMAC->Channel[_index_].CHCTRLB.reg
              |= DMAC_CHCTRLB_CMD_RESUME;
          }
          return true;
        }
        return false;
      }

      /// \b V1_COMPLETE
      void clear_descriptors() noexcept {
        using enum channel_state;
        if (_btask_[_index_]) {
          if (get_state() != disabled) {
            DMAC->Channel[_index_].CHCTRLA.reg 
              &= ~DMAC_CHCTRLA_ENABLE;
            while(DMAC->Channel[_index_].CHCTRLA.reg 
              & DMAC_CHCTRLA_ENABLE);
          }
          TransferDescriptor *curr_td = _btask_[_index_];
          bool firstFlag = true;

          while(curr_td && (firstFlag || curr_td != _btask_[_index_])) {
            TransferDescriptor *temp = curr_td->_link_;
            firstFlag = false;
            curr_td->_descPtr_->DESCADDR.reg &= ~DMAC_DESCADDR_MASK;
            curr_td->_link_ = nullptr;
            curr_td->_assigCH_ = -1;
            curr_td = temp;
          }
          memcpy(&_btask_[_index_]->_desc_, &_bdesc_[_index_],
            sizeof(_DmacDescriptor_));
          _btask_[_index_]->_descPtr_ = &_btask_[_index_]->_desc_;
          _btask_[_index_] = nullptr;
        }
      };

      ///\b V1_COMPLETE
      inline void set_linked_peripheral(const channel_peripheral &value) 
        noexcept {
        DMAC->Channel[_index_].CHCTRLA.reg 
          &= ~DMAC_CHCTRLA_TRIGSRC_Msk;
        DMAC->Channel[_index_].CHCTRLA.reg 
          |= static_cast<int>(value);
      }

      /// \b V1_COMPLETE
      inline channel_peripheral get_linked_peripheral() 
        const noexcept {
        auto reg_v = DMAC->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_TRIGSRC_Msk;
        return static_cast<channel_peripheral>(reg_v);
      };

      /// \b V1_COMPLETE
      inline bool set_transfer_mode(const transfer_mode &mode) noexcept {
        using enum_t = std::underlying_type_t<transfer_mode>;
        DMAC->Channel[_index_].CHCTRLA.reg 
          &= ~DMAC_CHCTRLA_TRIGACT_Msk;
        DMAC->Channel[_index_].CHCTRLA.reg 
          |= static_cast<enum_t>(mode);
      }

      /// \b V1_COMPLETE
      inline transfer_mode get_transfer_mode() const noexcept {
        auto reg_v = DMAC->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_TRIGACT_Msk;
        return static_cast<transfer_mode>(reg_v);
      }

      /// \b V1_COMPLETE
      bool set_packet_size(int elements) noexcept {       
        using reg_t = decltype(std::declval<DMAC_CHCTRLA_Type>().reg);

        reg_t bus_r = std::distance(bus_ref.begin(), 
          std::find(bus_ref.begin(), bus_ref.end(), elements));

        if (bus_r < bus_ref.size()) {
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

      /// \b DRAFT
      inline int get_packet_size() const noexcept {
        return bus_ref.at(DMAC->Channel[_index_].CHCTRLA.reg 
          & DMAC_CHCTRLA_BURSTLEN_Msk);
      }

      /// \b V1_COMPLETE
      bool set_priority_lvl(const int &lvl) noexcept {
        using reg_t = decltype(std::declval<DMAC_CHPRILVL_Type>().reg);

        reg_t lvl_r = std::distance(chpri_ref.begin(), 
          std::find(chpri_ref.begin(), chpri_ref.end(), lvl));
        
        if (lvl_r < chpri_ref.size()) {
          DMAC->Channel[_index_].CHPRILVL.reg 
            &= ~DMAC_CHPRILVL_MASK;
          DMAC->Channel[_index_].CHPRILVL.reg 
            |= DMAC_CHPRILVL_PRILVL(lvl_r);
          return true;
        }
        return false;
      }

      /// \b V1_COMPLETE
      int get_priority_lvl() const noexcept {
        return chpri_ref.at(DMAC->Channel[_index_].CHPRILVL.reg
          & DMAC_CHPRILVL_MASK);
      }

      const int _index_;
    }channel[DMAC_CH_NUM]{{._index_ = init_index++}}; 


    ///\b V1_COMPLETE???
    class TransferDescriptor {
      friend struct channel_ctrl;

      public:
      ///\b V1_COMPLETE
      TransferDescriptor() = default;

      ///\b V1_COMPLETE
      TransferDescriptor(const TransferDescriptor &other) {
        this->operator = (other);
      }

      ///\b DRAFT_COMPLETE -> NOTE NEED TO ADD SUSPENDING FOR OTHER CH
      TransferDescriptor operator = (const TransferDescriptor &other) {
        using enum channel_state;
        channel_state state = unknown;
        if (_assigCH_ != -1) {
          state = channel[_assigCH_].get_state();

          if (state != disabled && state != suspended) {
            DMAC->Channel[_assigCH_].CHCTRLB.reg
              |= DMAC_CHCTRLB_CMD_SUSPEND;
            while((DMAC->Channel[_assigCH_].CHINTFLAG.reg
              & DMAC_CHINTFLAG_SUSP) == 0);
          }
        }
        auto prev_l = _descPtr_->DESCADDR.reg;
        memcpy(_descPtr_, other._descPtr_, sizeof(_DmacDescriptor_));
        _descPtr_->DESCADDR.reg = prev_l;

        if (state == active) {
          DMAC->Channel[_assigCH_].CHCTRLB.reg
            |= DMAC_CHCTRLB_CMD_RESUME;
        } else if (state == idle) {
          DMAC->Channel[_assigCH_].CHCTRLB.reg
            |= DMAC_CHCTRLB_CMD_NOACT;
        }
      }

      ///\b V1_COMPLETE
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
            _descPtr_->BTCTRL.reg 
              |= DMAC_BTCTRL_STEPSEL;
            _descPtr_->BTCTRL.reg 
              &= ~DMAC_BTCTRL_STEPSIZE_Msk;
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

      /// \b V1_COMPLETE
      inline void *get_source() const noexcept {
        return (void*)(_descPtr_->SRCADDR.reg - addr_mod(true));
      }

      ///\b V1_COMPLETE
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

      ///\b V1_COMPLETE
      inline void *get_destination() const noexcept {
        return (void*)(_descPtr_->DSTADDR.reg - addr_mod(false));
      }

      ///\b V1_COMPLETE
      void unlink() noexcept {
        if (_assigCH_ != -1) {
          bool wbFlag = false;
          TransferDescriptor *curr = _btask_[_assigCH_];

          if (_link_ != this) {
            wbFlag = (_wbdesc_[_assigCH_].DESCADDR.reg 
              == (uintptr_t)_descPtr_);
            while(curr->_link_ && curr->_link_ != _btask_[_assigCH_]) {
              curr = curr->_link_;
            }
          }
          // Handle this = base task
          if (this == _btask_[_assigCH_]) {
            memcpy(&_desc_, &_bdesc_[_assigCH_], 
              sizeof(_DmacDescriptor_));
            _descPtr_ = &_desc_;

            if (!_link_ || _link_ == this) {
              memset(&_bdesc_[_assigCH_], 0, sizeof(_DmacDescriptor_));
              _btask_[_assigCH_] = nullptr;
            } else {
              memcpy(&_bdesc_[_assigCH_], &_link_->_desc_, 
                sizeof(_DmacDescriptor_));
              _link_->_descPtr_ = &_bdesc_[_assigCH_];
              _btask_[_assigCH_] = _link_;     
            }
          }
          if (curr) {
            curr->_link_ = _link_;
            curr->_descPtr_->DESCADDR.reg = _descPtr_->DESCADDR.reg;
            if (wbFlag) {
              curr->update_writeback();
            }
          }
          _descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
          _link_ = nullptr;
          _assigCH_ = -1;
        }
      }

      ///\b V1_COMPLETE
      inline TransferDescriptor *get_linked_descriptor() noexcept {
        return _link_;
      }

      ///\b V1_COMPLETE
      inline void update_writeback() noexcept {
        using enum channel_state;
        if (_assigCH_ && _wbdesc_[_assigCH_].DESCADDR.reg 
          == _descPtr_->DESCADDR.reg) {
          
          channel_state state = channel[_assigCH_].get_state();
          if (state != suspended && state != disabled) {
            DMAC->Channel[_assigCH_].CHCTRLB.reg 
              |= DMAC_CHCTRLB_CMD_SUSPEND;

            while((DMAC->Channel[_assigCH_].CHINTFLAG.reg
              & DMAC_CHINTFLAG_SUSP) == 0);
          }
          auto cnt = _wbdesc_[_assigCH_].BTCNT.reg;
          memcpy(&_wbdesc_[_assigCH_], &_descPtr_,
            sizeof(_DmacDescriptor_));

          _wbdesc_[_assigCH_].BTCNT.reg = cnt;
          if (state != suspended && state != disabled) {
            DMAC->Channel[_assigCH_].CHCTRLB.reg
              |= state == active ? DMAC_CHCTRLB_CMD_RESUME
              : DMAC_CHCTRLB_CMD_NOACT;
          }
        }
      }

      ///\b V1_COMPLETE
      bool set_alignment(const int &bytes) noexcept {
        auto beatsize_r = std::distance(bs_ref.begin(),
          std::find(bs_ref.begin(), bs_ref.end(), bytes));

        if (beatsize_r < bs_ref.size()) {
          uintptr_t src_addr = _descPtr_->SRCADDR.reg - addr_mod(true);
          uintptr_t dst_addr = _descPtr_->DSTADDR.reg - addr_mod(false);

          auto prev_cnt = _descPtr_->BTCNT.reg * bs_ref.at(_descPtr_
            ->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk);
          _descPtr_->BTCTRL.reg 
            &= ~DMAC_BTCTRL_BEATSIZE_Msk;
          _descPtr_->BTCTRL.reg 
            |= (beatsize_r << DMAC_BTCTRL_BEATSIZE_Pos);

          update_valid();
          _descPtr_->BTCNT.reg = prev_cnt ? (prev_cnt 
            / bs_ref.at(beatsize_r)) : 0; 
          _descPtr_->SRCADDR.reg = src_addr + addr_mod(true);
          _descPtr_->DSTADDR.reg = dst_addr + addr_mod(false);
          return true;
        }
        return false;
      }

      ///\b V1_COMPLETE
      inline int get_alignment() const noexcept {
        return (_descPtr_->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk);
      }

      /// \b V1_COMPLETE
      using btcnt_t = decltype(std::declval<DMAC_BTCNT_Type>().reg);
      inline void set_size(const btcnt_t &bytes) noexcept {
        uintptr_t src_addr = _descPtr_->SRCADDR.reg - addr_mod(true);
        uintptr_t dst_addr = _descPtr_->DSTADDR.reg - addr_mod(false);

        _descPtr_->BTCNT.reg = bytes ? bytes / bs_ref.at(_descPtr_
          ->BTCTRL.reg & DMAC_BTCTRL_BEATSIZE_Msk) : 0;        
        update_valid();
        _descPtr_->SRCADDR.reg = src_addr + addr_mod(true);
        _descPtr_->DSTADDR.reg = dst_addr + addr_mod(false);
      }

      ///\b V1_COMPLETE
      inline const btcnt_t &get_size() const noexcept {
        return _descPtr_->BTCNT.reg * bs_ref.at(_descPtr_->BTCTRL.reg
          & DMAC_BTCTRL_BEATSIZE_Msk);
      }

      private:

        ///\b NOTE_NEED_TO_GET_RID_OF_STRUCT
        struct {
          bool src_vol = false;
          bool dst_vol = false;
        }_cfg_ __packed __aligned(1);
        uint8_t _assigCH_;
        uint8_t _cfg1_ = 0;
        TransferDescriptor *_link_ = nullptr;
        _DmacDescriptor_ _desc_;
        _DmacDescriptor_ *_descPtr_ = &_desc_;

        ///\b V1_COMPLETE
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

        ///\b V1_COMPLETE
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


  }dma;

}

