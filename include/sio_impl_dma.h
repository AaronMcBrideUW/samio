
#pragma once
#include <sam.h>
#include <sio_impl_util.h>
#include <sio_defs.h>
#include <string.h>
#include <utility>
#include <cmath>
#include <memory>

namespace sio::impl::dma {

  /// @internal Reference   
  #define CFGMSK_SRC_POS 0
  #define CFGMSK_DST_POS 1
  #define DEFAULT_IRQ_PRIORITY 0

  /// @internal Hardware constants
  #define update_wb true
  #define DMA_DEF_CH_PRILVL 2
  #define DMA_DEF_CH_RUN_STBY false
  #define DMA_DEF_CH_LOOPED false

  static constexpr uint8_t bs_ref[] = {1, 2, 4};  
  static constexpr uint8_t ss_ref[] = {1, 2, 4, 8, 16, 32, 62, 128};

  typedef void (*error_int_t)(const int&, const channel_error&);
  typedef void (*transfer_int_t)(const int&);

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
  enum class transfer_mode {};
  enum class channel_peripheral : int {};

  /// @defgroup DMA misc config
  struct __attribute__((packed, aligned(1))) {
    static constexpr uint8_t min_obj_increment_size = 8;
    static constexpr uint8_t  prilvl_irq_priority[DMAC_LVL_NUM] = { 1 }; 
    static constexpr bool prilvl_rr_arb[DMAC_LVL_NUM] = { false };
    static constexpr uint8_t prilvl_service_quality[DMAC_LVL_NUM] = { 2 };
  }sys_config{};


  typedef struct active_channel_t {

    /// \b DONE

    static _aInline int index(void) noexcept {
      return DMAC->ACTIVE.bit.ID;
    }
    static _aInline int remaining_bytes(void) noexcept {
      return DMAC->ACTIVE.bit.BTCNT 
        * wbDescArray[index()].BTCTRL.bit.BEATSIZE;
    }
    static _aInline int is_busy(void) noexcept {  // TO DO -> Is there a better method we could have here?
      return DMAC->ACTIVE.bit.ABUSY;
    }
  };
  active_channel_t active_channel;



  class Channel {

    public:
    
      /// \b DONE


      /// @brief Default constructor.
      /// @note Uses next available DMA channel. If no channels
      ///   are available -> object is in undefined state.
      Channel() noexcept {
        static const uintptr_t baseAddr = mem_addr(baseDescArray);
        static const uintptr_t wbAddr = mem_addr(wbDescArray);

        // If first channel -> Init DMAC peripheral
        if (!_challoc_mask_) { 
          _reset_dmac_();
          for (int i = 0; i < DMAC_LVL_NUM; i++) {
            NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
            NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), 
              sys_config.prilvl_irq_priority[i]);
            NVIC_EnableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));

            DMAC->PRICTRL0.reg |=
              (sys_config.prilvl_rr_arb[i] 
              << (DMAC_PRICTRL0_RRLVLEN0_Pos + i * 8)) |
              (sys_config.prilvl_service_quality[i] 
              << (DMAC_PRICTRL0_QOS0_Pos + i * 8));
          }
          DMAC->BASEADDR.reg = baseAddr;
          DMAC->WRBADDR.reg = wbAddr;
          MCLK->AHBMASK.bit.DMAC_ = 1;
          DMAC->CTRL.bit.DMAENABLE = 1;
        }
        // "Allocate" & reset next available DMA channel
        for (int i = 0; i < DMAC_CH_NUM; i++) {
          if (_challoc_mask_ & (1 << i) == 0) {
            _challoc_mask_ |= (1 << i);
            _index_ = i;
            reset_channel();
          }
        }
      }


      /// \b DONE

      /// @brief Move constructor -> swaps target channel.
      /// @param other Other channel object.
      /// @return Reference to this channel object.
      inline Channel(Channel &&other) noexcept {
        this->operator = (std::move(other)); 
      }


      /// \b DONE

      /// @brief Move assignment operator -> swaps target channel.
      /// @param other Other channel object.
      /// @return Reference to this channel object. 
      inline Channel &operator = (Channel &&other) noexcept {
        if (_index_ != other._index_) {
          std::swap(_index_, other._index_);
        }
      }


      /// \b DONE

      bool set_state(const channel_state &state) noexcept {
        if (get_state() == state) {
          return true;
        } else if (_valid_index_()) {
          switch(state) {
            case channel_state::disabled: {
              DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 0;
                while(DMAC->Channel[_index_].CHCTRLA.bit.ENABLE);
              DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                = DMAC_CHCTRLB_CMD_NOACT_Val;
              DMAC->Channel[_index_].CHINTFLAG.bit.SUSP = 1;
              return true;
            }
            case channel_state::idle: {
              DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 0;
                while(DMAC->Channel[_index_].CHCTRLA.bit.ENABLE);
              DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                = DMAC_CHCTRLB_CMD_NOACT_Val;
              DMAC->Channel[_index_].CHINTFLAG.bit.SUSP = 1;
              DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 1;
              return true;
            }
            case channel_state::suspended: {
              DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 1;
              DMAC->Channel[_index_].CHINTFLAG.bit.SUSP = 1;
              DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                = DMAC_CHCTRLB_CMD_SUSPEND_Val;
                while(!DMAC->Channel[_index_].CHINTFLAG.bit.SUSP);
              return true;
            }
            case channel_state::active: {
              DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 1;
              DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                = DMAC_CHCTRLB_CMD_NOACT_Val;
              DMAC->Channel[_index_].CHINTFLAG.reg 
                = DMAC_CHINTFLAG_RESETVALUE;
              DMAC->SWTRIGCTRL.reg |= (1 << _index_);
              return true;
            }
          }
        }
        return false;
      }
      

      /// \b COMPLETE

      inline channel_state get_state(void) noexcept {
        if (_valid_index_()) {
          if (!DMAC->Channel[_index_].CHCTRLA.bit.ENABLE) {
            return channel_state::disabled;

          } else if (DMAC->Channel[_index_].CHINTFLAG.bit.SUSP) {
            return channel_state::suspended;

          } else if (DMAC->Channel[_index_].CHSTATUS.bit.BUSY
            || DMAC->Channel[_index_].CHSTATUS.bit.PEND) {
            return channel_state::active;

          } else if (DMAC->Channel[_index_].CHCTRLA.bit.ENABLE) {
            return channel_state::idle;
          }
        }
        return channel_state::unknown;
      }


      /// \b COMPLETE

      inline bool reset_transfer(void) noexcept {
        if (_valid_index_()) {
          set_state(channel_state::idle);
          memdesc(&wbDescArray[_index_], nullptr, _memdesc::clear);
          return true;
        }
        return false;
      }


      /// \b COMPLETE

      bool reset_channel() noexcept {
        if (_valid_index_()) {
          DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 0;
            while(DMAC->Channel[_index_].CHCTRLA.bit.ENABLE);
          DMAC->Channel[_index_].CHCTRLA.bit.SWRST = 1;
            while(DMAC->Channel[_index_].CHCTRLA.bit.SWRST);

          clear_tasks();
          _looped_ = DMA_DEF_CH_LOOPED;
          memdesc(&baseDescArray[_index_], nullptr, _memdesc::clear);
          memdesc(&wbDescArray[_index_], nullptr, _memdesc::clear);
          return true;
        }
        return false;
      }


      /// \b TO-DO

      template<Task&... _tasks_>
      constexpr bool set_tasks(Task&...) noexcept { 
        if constexpr ((_tasks_._assigCH_ == -1 && !_tasks_._linked_) || ...) {
          ((_tasks_._linked_, _tasks_) = ... = _tasks_);
          ((_tasks_._assigCH_ = _index_),...);
        }
      } 


      /// \b TO-DO

      bool clear_tasks(void); 


      /// \b TO-DO

      Task &get_current_task(void) { 

      }


      /// \b COMPLETE

      inline channel_error get_error(void) noexcept {
        if (_valid_index_()) {
          if (DMAC->Channel[_index_].CHINTFLAG.bit.TERR) {
            if (DMAC->Channel[_index_].CHSTATUS.bit.FERR) {
              return channel_error::task_error;
            }
            return channel_error::transfer_error;
          } else if (DMAC->Channel[_index_].CHSTATUS.bit.CRCERR) {
            return channel_error::crc_error;
          }
          return channel_error::none;
        }
        return channel_error::invalid_index;
      }


      /// \b COMPLETE

      ~Channel() {
        if (_valid_index_()) {
          _challoc_mask_ &= ~(1 << _index_);
          reset_channel();
        }
        if (!_challoc_mask_) {
          _reset_dmac_();
          for (int i = 0; i < DMAC_LVL_NUM; i++) {
            NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
            NVIC_DisableIRQ(static_cast<IRQn_Type>(DMAC_0_IRQn + i));
            NVIC_SetPriority(static_cast<IRQn_Type>(DMAC_0_IRQn + i), 
              DEFAULT_IRQ_PRIORITY);
          }
          MCLK->AHBMASK.bit.DMAC_ = 0;
        }
      }

      struct {

        bool set_linked_peripheral(const channel_peripheral&);
        channel_peripheral get_linked_peripheral(void) const;

        bool set_error_callback(error_int_t);
        error_int_t get_error_callback(void);

        bool transfer_callback(transfer_int_t);

        bool transfer_mode(const transfer_mode&);

        bool beat_size(const int&);

        bool priority_lvl(const int&);
        
      }config;

    private:

      bool _looped_ = false;
      int _index_ = -1;
      static inline uint32_t constinit _challoc_mask_ = 0;

      /// \b DONE

      /// @internal 
      /// @brief 
      ///   Returns true if set index of this channel is valid.
      bool _valid_index_() {
        return _index_ >= 0 && _index_ < DMAC_CH_NUM;
      }

      void _reset_dmac_() {
        DMAC->CTRL.bit.DMAENABLE = 0; 
          while(DMAC->CTRL.bit.DMAENABLE);
        DMAC->CRCCTRL.reg &= ~DMAC_CRCCTRL_MASK;
        DMAC->CTRL.bit.SWRST = 1;
          while(DMAC->CTRL.bit.SWRST);
      }

  };


  class Task { 
    friend Channel;

    public:

      /// /@brief Default constructor
      constexpr Task(void) = default;


      /// @brief Copy constructor
      /// @param other Task to copy data from.
      /// @note The assigned channel and linking configuration of the "other"
      ///   given task will NOT be copied by this constructor.
      constexpr Task (const Task &other) noexcept { 
        this->operator=(other);
      } 


      /// @brief 
      ///   Copy constructor
      /// @param other 
      ///   Task to move into this one.
      /// @note 
      ///   The assigned channel and linking configuration of the "other"
      ///   given task WILL be inhereted by constructor.
      constexpr Task (Task &&other) noexcept {
        this->operator=(std::move(other));
      }


      /// @brief 
      ///   Copy assignment operator.
      /// @param other 
      ///   Task to copy into this.
      /// @return  
      ///   A reference to this instance.
      /// @note
      ///   The assigned channel and linking configuration of the "orhter"
      ///   given task are not copied by this method.
      constexpr Task &operator = (const Task &other) noexcept {
        if (this != &other) {
          auto crit = _CritTaskSection_(_assigCH_);
          _cfgmsk_ = other._cfgmsk_;

          auto p_link = _descPtr_->DESCADDR.reg;
          memdesc(_descPtr_, other._descPtr_, _memdesc::copy);
          _descPtr_->DESCADDR.reg = p_link;

          if (update_wb && _is_writeback_()) {
            auto pcount = wbDescArray[_assigCH_].BTCNT.reg;
            memdesc(&wbDescArray[_assigCH_], _descPtr_, _memdesc::copy);
            wbDescArray[_assigCH_].BTCNT.reg = pcount;
          }
        }
        return *this;
      }


      /// @brief  
      ///   Move assignment operator.
      /// @param other 
      ///   Other taks instance to be copied from.
      /// @return 
      ///   A reference to this object.
      /// @note
      ///   The assigned channel and linking configuration of the "other"
      ///   given  task WILL be copied by this method. 
      constexpr Task &operator = (Task &&other) noexcept {
        if (this != &other) {
          // Avoid copy by re-assigning pointer (if base task)
          auto asgn_base = [](Task &base, Task &other) {
            memdesc(&base._desc_, other._descPtr_, _memdesc::move);
            other._descPtr_ = _assign(base._descPtr_, &base._desc_);
            baseTaskArray[base._assigCH_] = &other;
          };
          auto crit = _CritTaskSection_(_assigCH_);
          auto crit = _CritTaskSection_(other._assigCH_);
          _swap(_cfgmsk_, other._cfgmsk_);
          _swap(_linked_, other._linked_);

          // Handle cases -> both = base task, or other = base task
          if (other._validCH_() && &other == _btask_[other._assigCH_]) {
            if (_validCH_() && this == _btask_[_assigCH_]) {
              _swap(_descPtr_, other._descPtr_);
              _swap(_btask_[_assigCH_], _btask_[other._assigCH_]);            
            } else {
              asgn_base(other, *this);
            }
          } else {// Handle cases -> This = base task, or neither
            if (_validCH_() && this == _btask_[_assigCH_]) {
              asgn_base(*this, other);
            } else {
              memdesc(_descPtr_, other._descPtr_, _memdesc::swap);
            }
          }
          _swap(_assigCH_, other._assigCH_);
        }
      }


      /// @brief 
      ///   This method removes this task from it's assigned channel.
      /// @note
      constexpr void unlink(void) noexcept {
        if (_validCH_()) {
          auto crit = _CritTaskSection_(_assigCH_);

          // Handle case -> this = base descriptor
          if (this == _btask_[_assigCH_]) {
            memdesc(&_desc_, &baseDescArray[_assigCH_], _memdesc::move);
            _descPtr_ = &_desc_;
            
            if (_linked_ && _linked_ != this) {
              memdesc(&baseDescArray[_assigCH_], _linked_->_descPtr_,
                _memdesc::move);
              _linked_->_descPtr_ = &baseDescArray[_assigCH_];
              _btask_[_assigCH_] = _linked_;
            } else {
              _btask_[_assigCH_] = nullptr;
            }
          // Handle case -> this = descriptor in linked list
          } else {
            Task *prev = nullptr;
            Task *curr = _btask_[_assigCH_];

            while(curr != this) {  
              assert(curr->_linked_ != _btask_[_assigCH_]
                && curr->_linked_);
              prev = curr;
              curr = curr->_linked_;
            }
            assert(prev);
            prev->_linked_ = _linked_;
            prev->_descPtr_->DESCADDR.reg = _descPtr_->DESCADDR.reg;
          }
          if (update_wb && _is_writeback_()) {
            wbDescArray[_assigCH_].DESCADDR.bit.DESCADDR
              = DMAC_SRCADDR_RESETVALUE;
            if (update_wb) {
              memdesc(&wbDescArray[_assigCH_], nullptr, _memdesc::clear);
            }
          }
          _linked_ = nullptr;
          _descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
          _assigCH_ = -1; 
        }
      }





      /// \b DONE-DONE

      template<task_loc_t src_t, task_loc_t dst_t, int src_incr_v = -1, 
        int dst_incr_v = -1>
      constexpr void set_location(const src_t __restrict &src, const dst_t 
        __restrict &dst, const int& src_incr = -1, const int& dst_incr = -1) noexcept 
        requires task_incr_valid<src_incr_v, dst_incr_v> {
          
        using src_rt = std::remove_all_extents_t<src_t>;
        using dst_rt = std::remove_all_extents_t<dst_t>;
        auto crit = _CritTaskSection_(_assigCH_);

        // Find & set alignment 
        auto align_r = []() consteval -> unsigned int {
          for (int i = std::size(bs_ref) - 1; i >= 0; i--) {
            if (sizeof(src_rt) % bs_ref[i] == 0 
            && sizeof(dst_rt) % bs_ref[i] == 0
            && (src_incr_v == -1 ? 0 : src_incr_v) % bs_ref[i] == 0
            && (dst_incr_v == -1 ? 0 : dst_incr_v) % bs_ref[i] == 0) {
              return i;
            }
          }
          static_assert("Internal error");
        };
        _descPtr_->BTCTRL.bit.BEATSIZE = align_r();

        // Enable/disable incrementing
        constexpr bool sinc = src_incr_v >= 0 || std::is_array_v<src_rt>
          || sizeof(src_rt) > sys_config.min_obj_increment_size;
        constexpr bool dinc = dst_incr_v >= 0 || std::is_array_v<dst_rt>
          || sizeof(dst_rt) > sys_config.min_obj_increment_size;
        _descPtr_->BTCTRL.bit.SRCINC = sinc;
        _descPtr_->BTCTRL.bit.DSTINC = dinc;

        // Find & set custom increment config 
          constexpr auto mod_r = [&align_r]() consteval
            -> std::pair<unsigned int, unsigned int> {
            int stepsel_r = 0;
            int stepsize_r = 0;

            if (src_incr_v < 0 || dst_incr_v < 0) {
              unsigned int stepsize_r = 1;
              bool diff = sizeof(src_rt) > sizeof(dst_rt);

              if (align_r() <= std::size(bs_ref)
                && (std::is_array_v<src_t> || std::is_array_v<dst_t>)) {
                // Get ratio of larger / smaller size
                int ratio = (diff 
                  ? sizeof(src_rt) / sizeof(dst_rt)
                  : sizeof(dst_rt) / sizeof(src_rt)) 
                  / bs_ref[align_r()];
                // Ensure ratio = valid step size
                for (int i = 0; i < std::size(ss_ref); i++) {
                  if (ratio == ss_ref[i]) {
                    stepsize_r = i;
                    break;
                  }
                }
              } 
              // True = Use found config, false = use given config
              if (diff ? src_incr_v < 0 : dst_incr_v < 0 
                && stepsize_r > 1) {
                stepsel_r = sizeof(src_rt) > sizeof(dst_rt) 
                  ? DMAC_BTCTRL_STEPSEL_SRC_Val 
                  : DMAC_BTCTRL_STEPSEL_DST_Val;
              } else {
                stepsel_r = src_incr_v >= 0 
                  ? DMAC_BTCTRL_STEPSEL_SRC_Val 
                  : DMAC_BTCTRL_STEPSEL_DST_Val;
                stepsize_r = ((src_incr_v >= 0  
                  ? src_incr_v : dst_incr_v) / bs_ref[align_r()]);
              }
            // Increment config fully specified
            } else {
              stepsel_r = src_incr_v > dst_incr_v 
                ? DMAC_BTCTRL_STEPSEL_SRC_Val 
                : DMAC_BTCTRL_STEPSEL_DST_Val;
              stepsize_r = src_incr_v > dst_incr_v 
                ? src_incr_v : dst_incr_v;
            }
            return std::make_pair(stepsel_r, stepsize_r);
          }();
          _descPtr_->BTCTRL.bit.STEPSEL = mod_r.first;
          _descPtr_->BTCTRL.bit.STEPSIZE = mod_r.second;

        // Find & set src/dst address     
        _descPtr_->SRCADDR.reg = (uintptr_t)std::addressof(src);
        _descPtr_->DSTADDR.reg = (uintptr_t)std::addressof(dst);
        _cfgmsk_ |= ((1 << CFGMSK_SRC_POS) | (1 << CFGMSK_DST_POS));

        if constexpr (!std::is_volatile_v<src_rt>) {
          _cfgmsk_ &= ~(1 << CFGMSK_SRC_POS);
          _descPtr_->SRCADDR.reg += addr_mod(mod_r.first
            == DMAC_BTCTRL_STEPSEL_SRC_Val);
        }
        if constexpr (!std::is_volatile_v<dst_rt>) {
          _cfgmsk_ &= ~(1 << CFGMSK_DST_POS);
          _descPtr_->DSTADDR.reg += addr_mod(mod_r.first
            == DMAC_BTCTRL_STEPSEL_DST_Val);
        }
        // Update the writeback descriptor if applicable
        if (update_wb && _is_writeback_()) {
          auto pcount = wbDescArray[_assigCH_].BTCNT.reg;
          memdesc(&wbDescArray[_assigCH_], _descPtr_, 
            _memdesc::copy);
          wbDescArray[_assigCH_].BTCNT.reg = pcount;
        }
      }


      void *get_source(void) const {
        auto addr = _descPtr_->SRCADDR.reg; 
        if (_cfgmsk_ & (1 << CFGMSK_SRC_POS)) {
          addr -= addr_mod(true);
        }
        return addr <= 0 ? nullptr : reinterpret_cast<void*>(addr);
      }



      void *get_destination(void) const {
        auto addr = _descPtr_->DSTADDR.reg;
        if (_cfgmsk_ & (1 << CFGMSK_DST_POS)) {
          addr -= addr_mod(false);
        }
        return addr <= 0 ? nullptr : reinterpret_cast<void*>(addr);
      }


      /// \b DONE-DONE

      /// NOTE: Bytes must be a multiple of the size of
      /// both location types.
      constexpr void set_transfer_size(const size_t &bytes) noexcept {
        if (bytes && bytes % _descPtr_->BTCTRL.bit.BEATSIZE == 0) {
          if (_cfgmsk_ & (1 << CFGMSK_SRC_POS)) {
            _descPtr_->SRCADDR.reg -= addr_mod(true);
          }
          if (_cfgmsk_ & (1 << CFGMSK_DST_POS)) {
            _descPtr_->DSTADDR.reg -= addr_mod(false);
          }
          unsigned int cbytes = bytes / _descPtr_
            ->BTCTRL.bit.BEATSIZE;
          auto prev = std::exchange(_descPtr_->BTCNT.reg, cbytes);
          auto crit = _CritTaskSection_(_assigCH_);

          _descPtr_->SRCADDR.reg += addr_mod(_descPtr_
            ->BTCTRL.bit.STEPSEL == DMAC_BTCTRL_STEPSEL_SRC_Val);
          _descPtr_->DSTADDR.reg += addr_mod(_descPtr_
            ->BTCTRL.bit.STEPSEL == DMAC_BTCTRL_STEPSEL_DST_Val);

          if (update_wb && _is_writeback_()) {
            if (wbDescArray[_assigCH_].BTCNT.reg 
              - (cbytes - prev) > 0) {
              wbDescArray[_assigCH_].BTCNT.reg += (cbytes - prev);
            } else {
              wbDescArray[_assigCH_].BTCNT.reg = 0;
            }
          }
        }
      }


      /// \b DONE-DONE

      constexpr unsigned int get_transfer_size(void) const noexcept {
        return _descPtr_->BTCNT.reg;
      }


      /// \b DONE-DONE

      constexpr void set_suspend_channel(const bool &value) noexcept {
            auto res = value 
              ? DMAC_BTCTRL_BLOCKACT_SUSPEND_Val
              : DMAC_BTCTRL_BLOCKACT_NOACT_Val;
        _descPtr_->BTCTRL.bit.BLOCKACT = res;
        if (update_wb && _is_writeback_()) {
          wbDescArray[_assigCH_].BTCTRL.bit.BLOCKACT = res;
        }
      }


      /// \b DONE-DONE

      constexpr bool get_suspend_channel(void) const noexcept {
        return _descPtr_->BTCTRL.bit.BLOCKACT 
          == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
      }  


      /// \b DONE-DONE

      constexpr bool reset(void) noexcept {
        auto crit = _CritTaskSection_(_assigCH_);
        _cfgmsk_ = 0;

        uintptr_t prevLink = _descPtr_->DESCADDR.reg;
        memdesc(_descPtr_, nullptr, _memdesc::clear);
        _descPtr_->DESCADDR.reg = prevLink;

        if (update_wb && _is_writeback_()) {
          memdesc(&wbDescArray[_assigCH_], nullptr, _memdesc::clear);
          wbDescArray[_assigCH_].DESCADDR.bit.DESCADDR = prevLink;
        }
        return true;
      }


      /// \b DONE-DONE

      constexpr ~Task() noexcept {
        if (_linked_) {
          unlink();
          if (_validCH_() && !update_wb) {
            memdesc(&wbDescArray[_assigCH_], nullptr, _memdesc::clear);
          }
        }
      }


    private:

      uint8_t _cfgmsk_ = 0; // bit 1 = src, bit 2 = dst
      uint8_t _assigCH_ = -1;
      Task *_linked_ = nullptr;
      DmacDescriptor *_descPtr_ = &_desc_;
      DmacDescriptor _desc_{};


      /// \b DONE-DONE

      constexpr bool _validCH_(void) const noexcept {
        return _assigCH_ >= 0 && _assigCH_ < DMAC_CH_NUM;
      }


      /// \b DONE

      /// @internal
      /// @brief 
      ///   Determines if this task is currently written-back
      ///   to it's assigned channel.
      /// @return 
      ///   True if written-back to assigned channnel. 
      ///   False otherwise.
      /// @note 
      ///   Handles invalid channel index.      
      constexpr bool _is_writeback_(void) const noexcept {
        return !std::is_constant_evaluated() && _validCH_()
          && wbDescArray[_assigCH_].DESCADDR.reg 
            == _descPtr_->DESCADDR.reg;
      }


      /// \b DONE

      constexpr unsigned int addr_mod(const bool &isSrc) const noexcept {
        bool sel = _descPtr_->BTCTRL.bit.STEPSEL == (isSrc 
          ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val);
        return _descPtr_->BTCNT.reg 
        * (_descPtr_->BTCTRL.bit.BEATSIZE + 1) 
        * (sel ? (1 << _descPtr_->BTCTRL.bit.STEPSIZE) : 1);
      } 


      /// \b TO-DO -> ADD "READY CHANNEL METHOD OR SOMETHING..."

  };  


  namespace {


    /// \b DONE-DONE

    /// @internal
    /// @brief 
    ///   Concept used for validating implicit task
    ///   location parameters.
    /// @link 
    ///   Task.set_location
    template<typename T>
    concept task_loc_t = requires { 
       std::is_trivially_copyable_v<std::decay_t<std::remove_cvref_t<T>>>;
      !std::is_unbounded_array_v<std::remove_cvref_t<T>>;
      !std::is_reference_v<T>;
      !std::is_pointer_v<T>;
    };


    /// \b DONE-DONE

    /// @internal 
    /// @brief
    ///   Concept used for validating increment values
    ///   for task source & destination
    /// @link
    ///   Task.set_location
    template<int src_incr, int dst_incr>
    concept task_incr_valid = requires (int src_fn, int dst_fn) {
      src_incr == src_fn && dst_incr == dst_fn;
      src_incr <= 1 || dst_incr <= 1;
      src_incr >= -1 && dst_incr >= -1;
      []() consteval {
        int valid_n = 0;
        for (int i = 0; i < std::size(ss_ref); i++) {
          valid_n += std::abs(src_incr) == ss_ref[i]  
            + std::abs(dst_incr) == ss_ref[i];
        }
        return valid_n >= 2;
      }();
    };


    /// \b NOT-DONE -> CANNOT ACCESS IO FROM CONSTEXPR

    /// @internal
    /// @brief 
    ///   On construction, this object ensures that the
    ///   specified channel is suspended and on destruction
    ///   it returns the channel back to it's initial state.
    /// @note 
    ///    Multiple instances for the same index is handled.
    /// @note 
    ///   Invalid channel index parameter is handled.
    class _CritTaskSection_ {
      public:
        constexpr _CritTaskSection_(int index) noexcept : _index_(index),
          _pstate_(_valid_index_() && (DMAC->Channel[_index_].CHINTFLAG.reg
            & DMAC_CHINTFLAG_SUSP)) 
        {
          if (!std::is_constant_evaluated() && _valid_index_() && _pstate_) {

            DMAC->Channel[_index_].CHCTRLB.bit.CMD 
              = DMAC_CHCTRLB_CMD_SUSPEND_Val;
            while(DMAC->Channel[_index_].CHCTRLB.bit.CMD 
              == DMAC_CHCTRLB_CMD_SUSPEND_Val);
          }
        }
        constexpr ~_CritTaskSection_() noexcept {
          if (!std::is_constant_evaluated() && _valid_index_() && !_pstate_) {

            DMAC->Channel[_index_].CHCTRLB.bit.CMD 
              = DMAC_CHCTRLB_CMD_RESUME_Val;
          }
        }

      private:
        const bool _pstate_ = false;
        const int _index_ = -1;

        constexpr bool _valid_index_(void) {
          return _index_ >= 0 && _index_ < DMAC_CH_NUM;
        }
    };
  

    /// \b COMPLETE

    enum class _memdesc { copy, move, swap, clear };
    constexpr void memdesc(DmacDescriptor __restrict *into, DmacDescriptor
      __restrict *from = nullptr, const _memdesc &act = _memdesc::clear) noexcept {

      assert(into && (from || act == _memdesc::clear));
      if (std::is_constant_evaluated()) {
        auto const_cpy = [&into, &from]() -> void {
          into->BTCNT.reg = from->BTCNT.reg;
          into->BTCTRL.reg = from->BTCTRL.reg;
          into->DESCADDR.reg = from->DESCADDR.reg;
          into->DSTADDR.reg = from->DSTADDR.reg;
          into->SRCADDR.reg = from->SRCADDR.reg;
        };
        auto clear = [](DmacDescriptor *targ) -> void {
            targ->BTCNT.reg = 0;
            targ->BTCTRL.reg = 0;
            targ->DESCADDR.reg = 0;
            targ->DSTADDR.reg = 0;
            targ->SRCADDR.reg = 0;
        };
        switch(act) {
          case _memdesc::copy: {
            const_cpy();
          }
          case _memdesc::move: {
            const_cpy();
            clear(from);
          }
          case _memdesc::swap: {
            auto _1 = into->BTCNT.reg;
            auto _2 = into->BTCTRL.reg;
            auto _3 = into->DESCADDR.reg;
            auto _4 = into->DSTADDR.reg;
            auto _5 = into->SRCADDR.reg;
            const_cpy();
            from->BTCNT.reg = _1;
            from->BTCTRL.reg = _2;
            from->DESCADDR.reg = _3;
            from->DSTADDR.reg = _4;
            from->SRCADDR.reg = _5;
          }
          case _memdesc::clear: {
            clear(into);
          }
        }
      } else {
        switch(act) {
          case _memdesc::copy: {
            memcpy(into, from, sizeof(DmacDescriptor));
          }
          case _memdesc::move: {
            memmove(into, from, sizeof(DmacDescriptor));
            memset(from, 0, sizeof(DmacDescriptor));
          }
          case _memdesc::swap: {
            uint32_t buff[sizeof(DmacDescriptor) / sizeof(uint32_t)];
            memcpy(&buff, into, sizeof(DmacDescriptor)); 
            memmove(into, from, sizeof(DmacDescriptor));
            memmove(from, buff, sizeof(DmacDescriptor));  
          }
          case _memdesc::clear: {
            memset(into, 0, sizeof(DmacDescriptor));
          }
        }
      }
    }



  }
}



