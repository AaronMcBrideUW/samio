
#pragma once
#include <sio_impl_util.h>
#include <sio_config.h>

#include <sam.h>     // For register definitions
#include <string.h>  // For mem___ functions
#include <utility>   // For std::move, std::swap, std::forward
#include <array>
#include <algorithm>

namespace sio::impl::dma {

  class Channel;
  class Task;

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
    unknown = -1,
    packet = 2,
    task   = 0,
    all    = 3
  };
  enum class channel_peripheral : int {
    unknown = -1,
    none    = DMAC_CHCTRLA_TRIGSRC_DISABLE_Val,
    adc     = 1 //...
  };

  typedef void (*error_int_t)(const int&, const channel_error&);
  typedef void (*transfer_int_t)(const int&);


  namespace {

    /// @internal Non-volatile DMAC descriptor
    ///   used for storing information about tasks.
    typedef struct {
      DMAC_BTCTRL_Type BTCTRL;
      DMAC_BTCNT_Type BTCNT;
      DMAC_SRCADDR_Type SRCADDR;
      DMAC_DSTADDR_Type DSTADDR;
      DMAC_DESCADDR_Type DESCADDR;
    } _DmacDescriptor_ __aligned(8);

    // Storage
    static __attribute__ ((section(".hsram"))) __aligned(8) _DmacDescriptor_ 
      _wbdesc_[DMAC_CH_NUM]{};
    static __attribute__ ((section(".hsram"))) __aligned(8) _DmacDescriptor_ 
      _bdesc_[DMAC_CH_NUM]{};
    static Task *_btask_[DMAC_CH_NUM] = { nullptr };

    /// @internal Reference   
    #define CFGMSK_SRC_POS 0
    #define CFGMSK_DST_POS 1
    #define DEFAULT_IRQ_PRIORITY 0

    /// @internal Hardware constants
    #define update_wb true
    #define DMA_DEF_CH_PRILVL 2
    #define DMA_DEF_CH_RUN_STBY false
    #define DMA_DEF_CH_LOOPED false
    #define DMA_DEF_THRESH DMAC_CHCTRLA_THRESHOLD_1BEAT_Val;
    
    constexpr std::array<int, 3> bs_ref = {1, 2, 4};  
    constexpr std::array<int, 8> ss_ref = {1, 2, 4, 8, 16, 32, 62, 128};
    constexpr std::array<int, 16> bus_ref = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    constexpr std::array<int, 4> thresh_ref = {1, 2, 4, 8};
    constexpr std::array<int, 4> chpri_ref = {0, 1, 2, 3};


    /// @internal
    /// @brief 
    ///   On construction, this object ensures that the
    ///   specified channel is suspended and on destruction
    ///   it returns the channel back to it's initial state.
    /// @note 
    ///    Multiple instances for the same index is handled.
    /// @note 
    ///   Invalid channel index parameter is handled.
    class _CritDMA_ {
      public:

        /// \b COMPLETE-3

        constexpr _CritDMA_(int index) noexcept 
          : _index_(index) {
          if (!std::is_constant_evaluated()) {
            if (index >= 0 && index < DMAC_CH_NUM) {
              _pstate_ = (DMAC->Channel[_index_].CHINTFLAG.reg
                & DMAC_CHINTFLAG_SUSP); 

              if (_pstate_) {
                DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                  = DMAC_CHCTRLB_CMD_SUSPEND_Val;
                while(DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                  == DMAC_CHCTRLB_CMD_SUSPEND_Val);
              }
            }
          }
        }

        /// \b COMPLETE-3

        constexpr ~_CritDMA_() noexcept {
          if (!std::is_constant_evaluated()) {
            if (_index_ >= 0 && _index_ < DMAC_CH_NUM && !_pstate_) {
              DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                = DMAC_CHCTRLB_CMD_RESUME_Val;
            }
          }
        }

      private:
        bool _pstate_ = false;
        const int _index_ = -1;
    };
  

    /// \b COMPLETE-3

    enum class _memdesc { copy, move, swap, clear };
    constexpr void memdesc(_DmacDescriptor_ __restrict *into, _DmacDescriptor_
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
        auto clear = [](_DmacDescriptor_ *targ) -> void {
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
            memcpy(into, from, sizeof(_DmacDescriptor_));
          }
          case _memdesc::move: {
            memmove(into, from, sizeof(_DmacDescriptor_));
            memset(from, 0, sizeof(_DmacDescriptor_));
          }
          case _memdesc::swap: {
            uint32_t buff[sizeof(_DmacDescriptor_) / sizeof(uint32_t)];
            memcpy(&buff, into, sizeof(_DmacDescriptor_)); 
            memmove(into, from, sizeof(_DmacDescriptor_));
            memmove(from, buff, sizeof(_DmacDescriptor_));  
          }
          case _memdesc::clear: {
            memset(into, 0, sizeof(_DmacDescriptor_));
          }
        }
      }
    }


  } // END OF ANON NS



  /// @defgroup DMA misc config
  struct __attribute__((packed, aligned(1))) final {
    static constexpr uint8_t min_obj_increment_size = 8;
    static constexpr uint8_t  prilvl_irq_priority[DMAC_LVL_NUM] = { 1 }; 
    static constexpr bool prilvl_rr_arb[DMAC_LVL_NUM] = { false };
    static constexpr uint8_t prilvl_service_quality[DMAC_LVL_NUM] = { 2 };
  }sys_config{};


  struct active_channel_t final {

    static inline int index(void) noexcept {
      return DMAC->ACTIVE.bit.ID;
    }
    static inline int remaining_bytes(void) noexcept {
      return DMAC->ACTIVE.bit.BTCNT 
        * _wbdesc_[index()].BTCTRL.bit.BEATSIZE;
    }
    static inline int is_busy(void) noexcept {  // TO DO -> Is there a better method we could have here?
      return DMAC->ACTIVE.bit.ABUSY;
    }
  };
  active_channel_t active_channel;




  class Task { 
    friend Channel;

    public:

      /// \b COMPLETE-3

      /// /@brief Default constructor
      template<typename transfer_t>
      requires requires {
        bs_ref.end() != std::find(bs_ref.begin(), bs_ref.end(), 
          std::integral_constant(sizeof(std::remove_cvref_t<
          std::remove_all_extents_t<transfer_t>>)));
      } 
      Task() {
        constexpr ctrl_t beatsize_r = []() consteval {
          auto res = std::find(bs_ref.begin(), bs_ref.end(), 
            sizeof(raw_t<transfer_t>));

          if (res != bs_ref.end()) {
            return ((std::distance(bs_ref.begin(), res) 
              << DMAC_BTCTRL_BEATSIZE_Pos));
          }
          static_assert("Internal Error");
        }();
        _descPtr_->BTCTRL.reg = beatsize_r;
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

      /// @brief Copy constructor
      /// @param other Task to copy data from.
      /// @note The assigned channel and linking configuration of the "other"
      ///   given task will NOT be copied by this constructor.
      constexpr Task (const Task &other) noexcept { 
        this->operator=(other);
      } 


      /// \b COMPLETE-3 -> VALID CONSTEXPR

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


      /// \b COMPLETE-3 -> VALID CONSTEXPR

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
          auto crit = _CritDMA_(_assigCH_);
          _cfgmsk_ = other._cfgmsk_;

          auto p_link = _descPtr_->DESCADDR.reg;
          memdesc(_descPtr_, other._descPtr_, _memdesc::copy);
          _descPtr_->DESCADDR.reg = p_link;

          if (update_wb && _is_writeback_()) {
            auto pcount = _wbdesc_[_assigCH_].BTCNT.reg;
            memdesc(&_wbdesc_[_assigCH_], _descPtr_, _memdesc::copy);
            _wbdesc_[_assigCH_].BTCNT.reg = pcount;
          }
        }
        return *this;
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

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
            other._descPtr_ = std::exchange(base._descPtr_, &base._desc_);
            _btask_[base._assigCH_] = &other;
          };
          auto crit1 = _CritDMA_(_assigCH_);
          auto crit2 = _CritDMA_(other._assigCH_);
          std::swap(_cfgmsk_, other._cfgmsk_);
          std::swap(_linked_, other._linked_);

          // Handle cases -> both = base task, or other = base task
          if (other._validCH_() && &other == _btask_[other._assigCH_]) {
            if (_validCH_() && this == _btask_[_assigCH_]) {
              std::swap(_descPtr_, other._descPtr_);
              std::swap(_btask_[_assigCH_], _btask_[other._assigCH_]);            
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
          std::swap(_assigCH_, other._assigCH_);
        }
        return *this;
      }


      /// @brief 
      ///   This method removes this task from it's assigned channel.
      /// @note
      constexpr bool unlink(void) noexcept {
        if (_validCH_()) {
          auto crit = _CritDMA_(_assigCH_);

          // Handle case -> this = base descriptor
          if (this == _btask_[_assigCH_]) {
            memdesc(&_desc_, &_bdesc_[_assigCH_], _memdesc::move);
            _descPtr_ = &_desc_;
            
            if (_linked_ && _linked_ != this) {
              memdesc(&_bdesc_[_assigCH_], _linked_->_descPtr_,
                _memdesc::move);
              _linked_->_descPtr_ = &_bdesc_[_assigCH_];
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
            _wbdesc_[_assigCH_].DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
            if (update_wb) {
              memdesc(&_wbdesc_[_assigCH_], nullptr, _memdesc::clear);
            }
          }
          _linked_ = nullptr;
          _descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
          _assigCH_ = -1; 
        }
        return true;
      }

      /// \b COMPLETE_3 

      template<typename src_t>
      void set_source(const src_t &src, const int& src_incr = 0) noexcept 
      {
        using src_rt = std::remove_all_extents_t<src_t>;

        // Determine increment config


        // Determine increment enabled/disabled
        constexpr bool sinc = src_incr_v >= 0 
          || std::is_array_v<src_rt>
          || sizeof(src_rt) > sys_config.min_obj_increment_size;
        constexpr bool dinc = dst_incr_v >= 0 
          || std::is_array_v<dst_rt>
          || sizeof(dst_rt) > sys_config.min_obj_increment_size;

        constexpr desc_msk = (align_r << DMAC_BTCTRL_BEATSIZE_Pos) 
          | (sinc << DMAC_BTCTRL_SRCINC_Pos) 
          | (dinc << DMAC_BTCTRL_DSTINC_Pos)
          | (mod_r.first  << DMAC_BTCTRL_STEPSEL_Pos) 
          | (mod_r.second << DMAC_BTCTRL_STEPSIZE_Pos)
          | DMAC_BTCTRL_VALID;

        auto crit = _CritDMA_(_assigCH_);
        _descPtr_->BTCTRL.reg = _descPtr_->BTCTRL.bit.BLOCKACT 
          | desc_msk;
        
        // Find & set src/dst address
        _descPtr_->SRCADDR.reg = (uintptr_t)std::to_address(src); 
        _descPtr_->DSTADDR.reg = (uintptr_t)std::to_address(dst);
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
          auto pcount = _wbdesc_[_assigCH_].BTCNT.reg;
          memdesc(&_wbdesc_[_assigCH_], _descPtr_, 
            _memdesc::copy);
          _wbdesc_[_assigCH_].BTCNT.reg = pcount;
        }
      }


      /// \b COMPLETE-3 -> NOT CONSTEXPR

      void *get_source(void) const {
        auto addr = _descPtr_->SRCADDR.reg; 
        if (_cfgmsk_ & (1 << CFGMSK_SRC_POS)) {
          addr -= addr_mod(true);
        }
        return addr <= 0 ? nullptr : reinterpret_cast<void*>(addr);
      }


      /// \b COMPLETE-3 -> NOT CONSTEXPR

      void *get_destination(void) const {
        auto addr = _descPtr_->DSTADDR.reg;
        if (_cfgmsk_ & (1 << CFGMSK_DST_POS)) {
          addr -= addr_mod(false);
        }
        return addr <= 0 ? nullptr : reinterpret_cast<void*>(addr);
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

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
          auto crit = _CritDMA_(_assigCH_);

          _descPtr_->SRCADDR.reg += addr_mod(_descPtr_
            ->BTCTRL.bit.STEPSEL == DMAC_BTCTRL_STEPSEL_SRC_Val);
          _descPtr_->DSTADDR.reg += addr_mod(_descPtr_
            ->BTCTRL.bit.STEPSEL == DMAC_BTCTRL_STEPSEL_DST_Val);

          if (update_wb && _is_writeback_()) {
            if (_wbdesc_[_assigCH_].BTCNT.reg 
              - (cbytes - prev) > 0) {
              _wbdesc_[_assigCH_].BTCNT.reg += (cbytes - prev);
            } else {
              _wbdesc_[_assigCH_].BTCNT.reg = 0;
            }
          }
        }
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

      constexpr auto get_transfer_size(void) const noexcept { 
        return _descPtr_->BTCNT.reg;
        
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

      constexpr void set_suspend_channel(const bool &value) noexcept {
            auto res = value 
              ? DMAC_BTCTRL_BLOCKACT_SUSPEND_Val
              : DMAC_BTCTRL_BLOCKACT_NOACT_Val;
        _descPtr_->BTCTRL.bit.BLOCKACT = res;
        if (update_wb && _is_writeback_()) {
          _wbdesc_[_assigCH_].BTCTRL.bit.BLOCKACT = res;
        }
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

      constexpr bool get_suspend_channel(void) const noexcept {
         return _descPtr_->BTCTRL.bit.BLOCKACT
          == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
      }  


      /// \b COMPLETE_3 -> VALID CONSTEXPR

      constexpr bool reset(void) noexcept {
        auto crit = _CritDMA_(_assigCH_);
        _cfgmsk_ = 0;

        uintptr_t prevLink = _descPtr_->DESCADDR.reg;
        memdesc(_descPtr_, nullptr, _memdesc::clear);
        _descPtr_->DESCADDR.reg = prevLink;

        if (update_wb && _is_writeback_()) {
          memdesc(&_wbdesc_[_assigCH_], nullptr, _memdesc::clear);
          _wbdesc_[_assigCH_].DESCADDR.bit.DESCADDR = prevLink;
        }
        return true;
      }


      /// \b COMPLETE-3

      constexpr ~Task() noexcept {
        if (_linked_) {
          unlink();
          if (_validCH_() && !update_wb) {
            memdesc(&_wbdesc_[_assigCH_], nullptr, _memdesc::clear);
          }
        }
      }


    private:

      /// \b COMPLETE-3

      constexpr bool _validCH_(void) const noexcept {
        return _assigCH_ >= 0 && _assigCH_ < DMAC_CH_NUM;
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

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
          && _wbdesc_[_assigCH_].DESCADDR.reg 
            == _descPtr_->DESCADDR.reg;
      }


      /// \b COMPLETE-3 -> VALID CONSTEXPR

      constexpr unsigned int addr_mod(const bool &isSrc) const noexcept {
        bool sel = _descPtr_->BTCTRL.bit.STEPSEL == (isSrc 
          ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val);
        return _descPtr_->BTCNT.reg 
        * (_descPtr_->BTCTRL.bit.BEATSIZE + 1) 
        * (sel ? (1 << _descPtr_->BTCTRL.bit.STEPSIZE) : 1);
      } 


      /// @internal Class fields
      uint8_t _cfgmsk_ = 0; // bit 1 = src, bit 2 = dst,
      uint8_t _assigCH_ = -1;
      Task *_linked_ = nullptr;
      _DmacDescriptor_ _desc_{};
      _DmacDescriptor_ *_descPtr_ = &_desc_;

  };  


  class Channel final {

    public:
    
      /// \b COMPLETE-4

      /// @brief Default constructor.
      /// @note Uses next available DMA channel. If no channels
      ///   are available -> object is in undefined state.
      Channel() noexcept {
        static const uintptr_t baseAddr
          = reinterpret_cast<uintptr_t>(_bdesc_);
        static const uintptr_t wbAddr 
          = reinterpret_cast<uintptr_t>(_wbdesc_);

        // If first channel -> Init DMAC peripheral
        if (!_challoc_mask_) {
          if (DMAC->BASEADDR.reg != DMAC_BASEADDR_RESETVALUE
            || DMAC->WRBADDR.reg != DMAC_WRBADDR_RESETVALUE
            || DMAC->CTRL.reg != DMAC_CTRL_RESETVALUE) {
            _reset_dmac_();
          } 
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
          if ((_challoc_mask_ & (1 << i)) == 0) {
            _challoc_mask_ |= (1 << i);
            _index_ = i;
            reset_channel();
          }
        }
      }


      /// \b COMPLETE-4

      /// @brief Move constructor -> swaps target channel.
      /// @param other Other channel object.
      /// @return Reference to this channel object.
      /// @note Leaves other channel in undefined state.
      Channel(Channel &&other) noexcept {
        this->operator = (std::move(other)); 
      }


      /// \b COMPLETE-4

      /// @brief Move assignment operator -> swaps target channel.
      /// @param other Other channel object.
      /// @return Reference to this channel object. 
      Channel &operator = (Channel &&other) noexcept {
        if (_index_ != other._index_) {
          std::swap(_index_, other._index_);
        }
        return *this;
      }


      /// \b TO-DO -> NEED TO ADD CHECKS TO THIS METHOD...

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
              DMAC->Channel[_index_].CHINTFLAG.reg
                = DMAC_CHINTFLAG_RESETVALUE;
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
      

      /// \b COMPLETE-4

      inline channel_state get_state(void) const noexcept {
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


      /// \b COMPLETE-4

      inline bool reset_transfer(void) noexcept {
        if (_valid_index_()) {
          set_state(channel_state::idle);
          memdesc(&_wbdesc_[_index_], nullptr, _memdesc::clear);
          return true;
        }
        return false;
      }


      /// \b COMPLETE-4

      bool reset_channel() noexcept {
        if (_valid_index_()) {
          DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 0;
            while(DMAC->Channel[_index_].CHCTRLA.bit.ENABLE);
          DMAC->Channel[_index_].CHCTRLA.bit.SWRST = 1;
            while(DMAC->Channel[_index_].CHCTRLA.bit.SWRST);

          clear_tasks();
          _looped_ = DMA_DEF_CH_LOOPED;
          memdesc(&_bdesc_[_index_], nullptr, _memdesc::clear);
          memdesc(&_wbdesc_[_index_], nullptr, _memdesc::clear);
          return true;
        }
        return false;
      }


      /// \b COMPLETE-4

      template<Task* ..._tasks_>
      bool set_tasks(Task*...) noexcept 
        requires requires { // No repeats or null tasks
          []() consteval { 
            Task *tempBuff[sizeof...(_tasks_)]{};
            int index = 0;
            return ([&tempBuff, &index](Task *curr) constexpr {
              if (!curr) return false;
              for (int i = 0; i < index; i++) {
                if (tempBuff[i] == curr) return false;
              }
              tempBuff[index++] = curr;
              return true;          
            }(_tasks_) && ...); // curr = fold
          };
        }
      {
        static auto unlink_task = [](Task *targ) {
          targ->_assigCH_ = -1;
          targ->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
          targ->_linked_ = nullptr;
        };
        static constinit Task *base = []() consteval -> Task* {
          return ((base = _tasks_, true) || ...);
        }();

        if (_valid_index_()) {
          auto crit = _CritDMA_(_index_);
          bool baseFlag = false;

          // Handle currently assigned tasks
          if (_btask_[_index_]) {
            Task *curr = _btask_[_index_];

            if (curr != base) {
              baseFlag = true;
              memdesc(&curr->_desc_, &_bdesc_[_index_], 
                _memdesc::copy);
              curr->_descPtr_ = &curr->_desc_;
            }
            // Unlink applicable tasks
            ([&curr, this](Task *targ) {
              Task *next = (curr && curr->_linked_ != _btask_[_index_] 
                ? curr->_linked_ : nullptr);
              if (targ != curr) {
                if (curr) unlink_task(curr);
                targ->unlink();
              }
              curr = next;
            }(_tasks_),...); // Targ = fold

            while(curr && curr != _btask_[_index_]) {
              unlink_task(curr);
              curr = curr->_linked_;
            }
          }
          // Set new base task
          if (baseFlag) {
            memdesc(&_bdesc_[_index_], base->_descPtr_, 
              _memdesc::copy);
            base->_descPtr_ = &_bdesc_[_index_];
            _btask_[_index_] = base;
          }
          Task dummy;
          Task *prev = _looped_ ? (_tasks_,...) : &dummy;

          ((prev->_linked_ = _tasks_, 
            prev->_descPtr_->DESCADDR.reg 
              = reinterpret_cast<uintptr_t>(_tasks_->_descPtr_),
            prev->_assigCH_ = _index_,
            prev = _tasks_
          ),...);
          return true;
        }
        warn("Attempt to access channel in undefined state");
        return false;
      } 


      /// \b FINAL

      bool clear_tasks(void) noexcept {
        if (_valid_index_()) [[likely]] {
          if (_btask_[_index_]) {

            Task *current = _btask_[_index_];
            set_state(channel_state::disabled);

            // Clear descriptor storage arrays
            memdesc(&current->_desc_, &_bdesc_[_index_], _memdesc::move);
            memdesc(&_wbdesc_[_index_], nullptr, _memdesc::clear);
            _btask_[_index_] = nullptr;
            current->_descPtr_ = &current->_desc_;

            Task *next = nullptr;
            bool bflag = true;
            while(current && (current != _btask_[0] || bflag)) {
              bflag = false;
              next = current->_linked_;
              current->_assigCH_ = -1;
              current->_linked_ = nullptr;
              current->_descPtr_->DESCADDR.reg = DMAC_SRCADDR_RESETVALUE;
              current = next;
            }
            return true;
          }
          return false;
        }
        warn("Attempt to access channel in undefined state");
        return false;
      }



      /// \b FINAL

      Task *get_current_task(void) const { 
        if (_valid_index_()) {
          Task *current = _btask_[_index_];
          bool bflag = false;
          while(current && (bflag || current != _btask_[_index_])) {
            bflag = true;
            if (current->_is_writeback_()) {
              return current;
            }
            current = current->_linked_;
          }
        }
        warn("Attempt to access channel in undefined state");
        return nullptr;
      }


      /// \b FINAL

      inline channel_error get_error(void) const noexcept {
        if (_valid_index_()) [[likely]] {
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
        warn("Attempt to access channel in undefined state");
        return channel_error::invalid_index;
      }


      /// \b FINAL

      ~Channel() noexcept {
        if (_valid_index_()) {
          _challoc_mask_ &= ~(1 << _index_);
          reset_channel();

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
      }


      /// \b FINAL

      struct channel_config {

        /// \b FINAL

        inline bool set_linked_peripheral(const channel_peripheral 
          &value) noexcept {
          if (super->_valid_index_()) [[likely]] {
            DMAC->Channel[super->_index_].CHCTRLA.bit.TRIGSRC
              = static_cast<int>(value);
            return true;
          }
          warn("Attempt to access channel in undefined state");
          return false;
        }

        /// \b FINAL

        inline channel_peripheral get_linked_peripheral(void) const noexcept {
          if (super->_valid_index_()) [[likely]] {
            return static_cast<channel_peripheral>
              (DMAC->Channel[super->_index_].CHCTRLA.bit.TRIGSRC);
          }
          warn("Attempt to access channel in undefined state");
          return channel_peripheral::unknown;
        };


        /// \b FINAL

        inline bool set_transfer_mode(const transfer_mode &mode) noexcept {
          if (super->_valid_index_()) [[likely]] {
            auto crit = _CritDMA_(super->_index_);
            DMAC->Channel[super->_index_].CHCTRLA.bit.TRIGACT = 
              static_cast<std::underlying_type_t<transfer_mode>>(mode);
            return true;
          }
          warn("Attempt to access channel in undefined state");
          return false;
        }


        /// \b FINAL

        inline transfer_mode get_transfer_mode() const noexcept {
          if (super->_valid_index_()) [[likely]] {
            return static_cast<transfer_mode>
              (DMAC->Channel[super->_index_].CHCTRLA.bit.TRIGACT);
          }
          warn("Attempt to access channel in undefined state!");
          return transfer_mode::unknown;
        }

        /// \b FINAL

        template<int _elements_>
        bool set_packet_size(const int &elements = _elements_) noexcept
          requires requires {
            []() {
              for (auto&& curr : bus_ref) {
                if (_elements_ == curr) return true;
              }
              return false;
            }();
          }
        {
          constexpr auto burstlen_r = []() consteval -> unsigned int {
            for (int i = 0; i < REF_SIZE(bus_ref); i++) {
              if (_elements_ == bus_ref[i]) return i;  
            }
            static_assert("Internal error.");
          }();
          constexpr auto thresh_r = []() consteval -> unsigned int {
            for (int i = REF_SIZE(thresh_ref) - 1; i >= 0; i--) {
              if (_elements_ % thresh_ref[i] == 0) {
                return i;
              }
            }
            return DMA_DEF_THRESH;
          }();
          if (elements != _elements_) {
            warn("Non-type template parameter does not match value" 
              "passed to function!");
          }
          if (super->_valid_index_()) [[likely]] {
            auto crit = _CritDMA_(super->_index_);
            DMAC->Channel[super->_index_].CHCTRLA.bit.BURSTLEN = burstlen_r;
            DMAC->Channel[super->_index_].CHCTRLA.bit.THRESHOLD = thresh_r
            return true;
          }
          warn("Attempt to access channel in undefined state");
          return false;
        }


        /// \b FINAL

        inline int get_packet_size() const noexcept {
          if (super->_valid_index_()) [[likely]] {
            return bus_ref[DMAC->Channel[super->_index_]
              .CHCTRLA.bit.BURSTLEN];
          }
          warn("Attempt to access channel in undefined state");
          return -1;
        }

        /// \b FINAL

        template<int _lvl_>
        bool set_priority_lvl(const int &lvl) noexcept 
          requires requires {
            []() {
              for (auto&& curr : chpri_ref) {
                if (_lvl_ == curr) return true;
              } 
              return false;
            }();
          }
        {
          constexpr auto chpri_r = []() consteval -> unsigned int {
            for (int i = 0; i < REF_SIZE(chpri_ref); i++) {
              if (lvl == chpri_ref[i]) {
                return i;
              }
            }
            static_assert("Internal error.");
          }();
          if (lvl != _lvl_) [[unlikely]] {
            warn("Non-type template parameter does not match value" 
              "passed to function!");
          } 
          if (super->_valid_index_()) [[likely]] {
            DMAC->Channel[super->_index_].CHPRILVL.bit.PRILVL = chpri_r;
            return true;
          }
          warn("Attempt to access channel in undefined state");
          return false;          
        }


        /// \b FINAL

        int get_priority_lvl() const noexcept {
          if (super->_valid_index_()) [[likely]] {
            return chpri_ref[DMAC->Channel[super->_index_]
              .CHPRILVL.bit.PRILVL];
          }
          warn("Attempt to access channel in undefined state");
          return -1;
        }


        private:
          friend Channel;
          explicit channel_config(Channel *super)
            : super(super) {}; 
          Channel *super;
      }config{this};

    private:

      bool _looped_ = false;
      int _index_ = -1;
      static inline uint32_t _challoc_mask_ = 0; 


      /// \b FINAL

      /// @internal 
      /// @brief 
      ///   Returns true if set index of this channel is valid.
      constexpr bool _valid_index_() const {
        return _index_ >= 0 && _index_ < DMAC_CH_NUM;
      }

      /// \b COMPLETE-4

      void _reset_dmac_() {
        DMAC->CTRL.bit.DMAENABLE = 0; 
          while(DMAC->CTRL.bit.DMAENABLE);
        DMAC->CRCCTRL.reg &= ~DMAC_CRCCTRL_MASK;
        DMAC->CTRL.bit.SWRST = 1;
          while(DMAC->CTRL.bit.SWRST);
      }
  };




} // END OF SIO::IMPL::DMA



