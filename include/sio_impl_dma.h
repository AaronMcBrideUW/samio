
#pragma once
#include <sam.h>
#include <sio_impl_util.h>
#include <sio_defs.h>
#include <string.h>
#include <utility>
#include <cmath>
#include <memory>

namespace sio::impl::dma {
  
  /// @internal Hardware constants
  #define DMA_DEF_UPDATE_ACTIVE true
  #define DMA_DEF_CH_PRILVL 2
  #define DMA_DEF_CH_RUN_STBY false
  static constexpr uint8_t bsRef[] = {1, 2, 4};  
  static constexpr uint8_t ssRef[] = {1, 2, 4, 8, 16, 32, 62, 128};

  typedef void (*error_cb_t)(const int&, const channel_error&);
  typedef void (*transfer_cb_t)(const int&);

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
    crc_error
  };
  enum class transfer_mode {};
  enum class channel_peripheral : int {};

  /// @defgroup DMA misc config
  struct __attribute__((packed, aligned(1))) {
    static inline uint8_t min_obj_increment_size = 8;
    static inline uint8_t prilvl_irq_priority[DMAC_LVL_NUM] = { 1 }; 
    static inline bool prilvl_rr_arb[DMAC_LVL_NUM] = { false };
    static inline uint8_t prilvl_service_quality[DMAC_LVL_NUM] = { 2 };
  }sys_config{};


  typedef struct active_channel_t {
    static _aInline int index(void) {
      return DMAC->ACTIVE.bit.ID;
    }
    static _aInline int remaining_bytes(void) {
      return DMAC->ACTIVE.bit.BTCNT 
        * wbDescArray[index()].BTCTRL.bit.BEATSIZE;
    }
    static _aInline int is_busy(void) { // TO DO -> Is there a better method we could have here?
      return DMAC->ACTIVE.bit.ABUSY;
    }
  };
  inline active_channel_t active_channel;


  ///////// RECALL -> ENSURE _index_ = -1 handled for all methods
  class Channel {

    public:
    
      /// @brief Default constructor.
      /// @note Uses next available DMA channel. If no channels
      ///   are available -> object is in undefined state.
      Channel() {
        static const uintptr_t baseAddr = mem_addr(baseDescArray);
        static const uintptr_t wbAddr = mem_addr(wbDescArray);

        if (!_challoc_mask_) { 
          if (DMAC->BASEADDR.bit.BASEADDR == baseAddr
            && DMAC->WRBADDR.bit.WRBADDR == wbAddr
            && !DMAC->CTRL.bit.DMAENABLE) {
            DMAC->CTRL.bit.DMAENABLE;
            return;
          }
          DMAC->CTRL.bit.DMAENABLE = 0; 
            while(DMAC->CTRL.bit.DMAENABLE);
          DMAC->CRCCTRL.reg &= ~DMAC_CRCCTRL_MASK;
          DMAC->CTRL.bit.SWRST = 1;
            while(DMAC->CTRL.bit.SWRST);

          for (int i = 0; i < DMAC_LVL_NUM; i++) {
            NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), 
              sio::impl::dma::sys_config.prilvl_irq_priority[i]);
            NVIC_EnableIRQ((IRQn_Type)(DMAC_0_IRQn + i));
            
            DMAC->PRICTRL0.reg |=
              (sio::impl::dma::sys_config.prilvl_rr_arb[i] 
              << (DMAC_PRICTRL0_RRLVLEN0_Pos + i * 8)) |
              (sio::impl::dma::sys_config.prilvl_service_quality[i] 
              << (DMAC_PRICTRL0_QOS0_Pos + i * 8));
          }
          DMAC->BASEADDR.reg = baseAddr;
          DMAC->WRBADDR.reg = wbAddr;
          DMAC->CTRL.reg |= DMAC_CTRL_LVLEN_Msk;
          MCLK->AHBMASK.bit.DMAC_ = 1;
          DMAC->CTRL.bit.DMAENABLE = 1;
        }
        for (int i = 0; i < DMAC_CH_NUM; i++) {
          if (_challoc_mask_ & (1 << i) == 0) {
            _challoc_mask_ |= (1 << i);
            _index_ = i;
            reset_channel();
          }
        }
      }
      /// @brief Move constructor -> swaps target channel.
      /// @param other Other channel object.
      /// @return Reference to this channel object.
      Channel(Channel &&other) {
        this->operator = (std::move(other)); 
      }
      /// @brief Move assignment operator -> swaps target channel.
      /// @param other Other channel object.
      /// @return Reference to this channel object. 
      Channel &operator = (Channel &&other) noexcept {
        if (other._index_ != _index_) {
          std::swap(_index_, other._index_);
        }
      }

      bool set_state(const channel_state &state) {
        if (_index_ == -1) {
          return false;
        } else if (state == get_state()) {
          return true;
        }
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
          default: {
            return false;
          }
        }
      }

      channel_state get_state(void) {
        if (_index_ != -1) {
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

      bool reset_transfer(void) {
        if (_index_ == -1) {
          return false;
        }
        auto crit = _CritTaskSection_(_index_);
        memset(&wbDescArray[_index_], 0, sizeof(DmacDescriptor));
        return true;
      }

      bool reset_channel() {
        if (_index_ == -1) {
          return false;
        }
        DMAC->Channel[_index_].CHCTRLA.bit.ENABLE = 0;
          while(DMAC->Channel[_index_].CHCTRLA.bit.ENABLE);
        DMAC->Channel[_index_].CHCTRLA.bit.SWRST = 1;
          while(DMAC->Channel[_index_].CHCTRLA.bit.SWRST);
          
        clear_tasks();
        Task test1;
        Task test2;
        set_tasks(test1, test2);
        memset(&baseDescArray[_index_], 0, sizeof(DmacDescriptor));
        memset(&wbDescArray[_index_], 0, sizeof(DmacDescriptor));
        return true;
      }

      template<Task&... _tasks_>
      bool set_tasks(Task&...) { 

        if (sizeof...(_tasks_) > 0) {

          static constepxr Task *taskArr[]{std::addressof(_tasks_)...};
          static constexpr auto numt = sizeof...(_tasks_);

          memcpy(&baseDescArray[_index_], taskArr[0], 
            sizeof(DmacDescriptor));
          baseDescBuffer[_index_] = std::exchange(taskArr[0]->_descPtr_, 
            &baseDescArray[_index_]);
          taskArr[0]->_assigCH_ = _index_;

          Task *prev = taskArr[0];
          for (int i = 1; i < sizeof...(_tasks_); i++) {
            if (taskArr[i] == taskArr[0]) {
              break;
            } else if (taskArr[i] && taskArr[i]->_assigCH_ == -1) {
              prev->_descPtr_->DESCADDR.bit.DESCADDR 
                = mem_addr(taskArr[i]);
              prev->_linked_ = taskArr[i];
              taskArr[i]->_assigCH_ = _index_;
              prev = taskArr[i];
            }
          }
          if (_looped_) {
            prev->_descPtr_->DESCADDR.bit.DESCADDR 
              = mem_addr(taskArr[0]); 
            prev->_linked_ = taskArr[0];
          } else {
            prev->_descPtr_->DESCADDR.bit.DESCADDR
              = DMAC_SRCADDR_RESETVALUE;
            prev->_linked_ = nullptr;
          }
        }
        return true;   
      }

      bool clear_tasks(void); ////// TO DO 

      _aInline Task &get_current_task(void) { // TO DO 

      }

      channel_error get_error(void) {
        if (_index_ != -1) {
          if (DMAC->Channel[_index_].CHINTFLAG.bit.TERR) {
            if (DMAC->Channel[_index_].CHSTATUS.bit.FERR) {
              return channel_error::task_error;
            }
            return channel_error::transfer_error;
          } else if (DMAC->Channel[_index_].CHSTATUS.bit.CRCERR) {
            return channel_error::crc_error;
          }
        }
        return channel_error::none; 
      }

      ~Channel() {
        _challoc_mask_ &= ~(1 << _index_);
        reset_channel();
        if (_challoc_mask_ == 0) {
          DMAC->CTRL.bit.DMAENABLE = 0;
        }
      }

      struct {

        // DMAC->Channel[_index_].CHCTRLA.bit.RUNSTDBY
        //   = configGroup::chRunStandby[_index_];
        // DMAC->Channel[_index_].CHPRILVL.bit.PRILVL
        //   = configGroup::chPrilvl[_index_];
        // DMAC->Channel[_index_].CHINTENSET.reg |=
        //   ((int)errorCB << DMAC_CHINTENSET_TERR_Pos) |
        //   ((int)transferCB << DMAC_CHINTENSET_TCMPL_Pos);

        bool linked_peripheral(const channel_peripheral&);

        bool error_callback(error_cb_t);

        bool transfer_callback(transfer_cb_t);

        bool transfer_mode(const transfer_mode&);

        bool beat_size(const int&);

        bool priority_lvl(const int&);
        
      }config;

    private:

      bool _looped_ = false;
      int _index_ = -1;

  };


  /////////////////////////////////////// TO DO -> ADD CHECKS FOR NULL TASK IN EVERY METHOD
  //                                      AND ENSURE THAT THERE IS ALWAYS A NEW TASK IN THE BUFFER ARRAY
  class Task { 
    friend Channel;

    public:

      /// @brief Default allocating constructor   
      explicit Task(void) {
        _descPtr_ = new DmacDescriptor;
      }

      /// @brief Copy constructor .
      Task (const Task &other) : Task() {
        if (this != &other) {
          this->operator = (other);
        }
      } 

      /// @brief Move constructor.
      /// NOTE: This constructor leaves moved task in 
      ///   undefined state.
      Task(Task &&other) noexcept {
        if (this != &other) {
          this->operator = (std::move(other)); 
        }
      }

      /// @brief Copy assignment operator.
      Task &operator = (const Task &other) {
        if (this != &other) {
          auto crit = _CritTaskSection_(_assigCH_);
          auto link = _descPtr_->DESCADDR.bit.DESCADDR;
          memcpy(_descPtr_, other._descPtr_, sizeof(DmacDescriptor));
          _descPtr_->DESCADDR.bit.DESCADDR = link;
        }
        return *this;
      }

      /// @brief Move assignment operator.
      Task &operator = (Task &&other) noexcept {
        if (this != &other) {
          auto crit = _CritTaskSection_(_assigCH_);
          auto crit = _CritTaskSection_(other._assigCH_);
          std::swap(_descPtr_, other._descPtr_);
          std::swap(_assigCH_, other._assigCH_);
          std::swap(_linked_, other._linked_);
        }
        return *this;
      }

      /// NOTE: Bytes must be a multiple of the size of
      /// both location types.
      bool set_transfer_size(const unsigned int &bytes, 
        const bool &udpateActive = DMA_DEF_UPDATE_ACTIVE) {
        auto beatsize = _descPtr_->BTCTRL.bit.BEATSIZE;
        if (!is_multiple(bytes, beatsize)) {
          return false;
        }
        auto crit = _CritTaskSection_(_assigCH_);
        decltype(beatsize) value_r = safe_div(bytes, beatsize);
        int diff_r = _descPtr_->BTCNT.bit.BTCNT - value_r;
        
        _descPtr_->BTCNT.bit.BTCNT = value_r;
        if (udpateActive && _is_writeback_()) {
          wbDescArray[_assigCH_].BTCNT.bit.BTCNT = bound_min(diff_r, 0);
        }
        return true;
      }

      int get_transfer_size(void) {
        return _descPtr_->BTCNT.bit.BTCNT;
      }

      inline void set_suspend_channel(const bool &value,
        const bool &updateActive = DMA_DEF_UPDATE_ACTIVE) {
        auto crit = _CritTaskSection_(_assigCH_);
        auto baVal = value ? DMAC_BTCTRL_BLOCKACT_SUSPEND_Val 
          : DMAC_BTCTRL_BLOCKACT_NOACT_Val;
        
        _descPtr_->BTCTRL.bit.BLOCKACT = baVal;
        if (updateActive && _is_writeback_()) {
          wbDescArray[_assigCH_].BTCTRL.bit.BLOCKACT = baVal;
        }
      }

      inline bool get_suspend_channel(void) {
        return _descPtr_->BTCTRL.bit.BLOCKACT 
          == DMAC_BTCTRL_BLOCKACT_SUSPEND_Val;
      }  

      bool unlink(const bool &updateActive = DMA_DEF_UPDATE_ACTIVE) {
        if (_assigCH_ != -1) {
        auto crit = _CritTaskSection_(_assigCH_);

          Task *prev = this;
          while(prev->_linked_ && prev->_linked_ != this) {
            prev = prev->_linked_;
          }
          // Handle case -> this = base task     
          if (this == baseTaskArray[_assigCH_]) {
            assert(baseDescBuffer[_assigCH_]);
            _descPtr_ = std::exchange(baseDescBuffer[_assigCH_], nullptr);            
   
            if (_linked_ && _linked_ != this) { 
              assert(_linked_->_assigCH_ == _assigCH_);
              memcpy(&baseDescArray[_assigCH_], _linked_->_descPtr_, 
                sizeof(DmacDescriptor));
              baseDescBuffer[_assigCH_] = std::exchange(_linked_->_descPtr_, 
                &baseDescArray[_assigCH_]);
            } else {
              memset(&baseDescArray[_assigCH_], 0, sizeof(DmacDescriptor));
              if (updateActive && _is_writeback_()) {
                wbDescArray[_assigCH_].DESCADDR.bit.DESCADDR 
                  = DMAC_SRCADDR_RESETVALUE;
              }
            }
          // Handle case -> this = within task list
          } else {
            assert(prev);
            prev->_descPtr_->DESCADDR.bit.DESCADDR
              = _descPtr_->DESCADDR.bit.DESCADDR;
          }
          _assigCH_ = -1;
          _linked_ = nullptr;
          _descPtr_->DESCADDR.bit.DESCADDR = DMAC_SRCADDR_RESETVALUE;
        }
        return true;
      }

      bool reset() {
        auto crit = _CritTaskSection_(_assigCH_);
        uintptr_t prevLink = _descPtr_->DESCADDR.bit.DESCADDR;

        memset(_descPtr_, 0, sizeof(DmacDescriptor));
        _descPtr_->DESCADDR.bit.DESCADDR = prevLink;

        if (_is_writeback_()) {
          memset(&wbDescArray[_assigCH_], 0, sizeof(DmacDescriptor));
          wbDescArray[_assigCH_].DESCADDR.bit.DESCADDR = prevLink;
        }
        return true;
      }

      ~Task() {
        if (_linked_) {
          unlink(true);
        }
        if (_descPtr_) {
          delete _descPtr_;
        }
      }

      template<typename src_t, typename dest_t, int incrSrc_v, 
        int incrDest_v, int align_v> 
      void set_location(const src_t __restrict *src, const dest_t __restrict *dest,
        const int &incrSrc, const int &incrDest, const int &align)
        requires _c_valid_explicit_loc_<src_t, dest_t, incrSrc_v, 
          incrDest_v, align_v> {

        auto crit = _CritTaskSection_(_assigCH_); 
        _descPtr_->BTCTRL.bit.BEATSIZE = indexOf(align_v, bsRef);

        // Set increment config
        _descPtr_->BTCTRL.bit.STEPSEL = _srcIncr_ 
          ? DMAC_BTCTRL_STEPSEL_SRC_Val : DMAC_BTCTRL_STEPSEL_DST_Val;
        _descPtr_->BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;

        if constexpr (_srcIncr_ > 1) {
          _descPtr_->BTCTRL.bit.STEPSIZE = indexOf(incrSrc_v, ssRef);
        } else if constexpr (_destIncr_ > 1) {
          _descPtr_->BTCTRL.bit.STEPSIZE = indexOf(incerDest_v, ssRef);
        }
        // Set src/dest address
        _descPtr_->SRCADDR.bit.SRCADDR = mem_addr(src);
        _descPtr_->DSTADDR.bit.DSTADDR = mem_addr(dest);
        if constexpr (!std::is_volatile_v<src_t>) {
          _descPtr_->SRCADDR.bit.SRCADDR += addr_mod(srcIncr > 1);
        }
        if constexpr (!std::is_volatile_V<dest_t>) {
          _descPtr_->DSTADDR.bit.DSTADDR += addr_mod(destIncr > 1);
        }
      }

      template<typename src_t, typename dest_t>
      void set_location(const src_t __restrict *src, const dest_t __restrict *dest)
        requires _c_valid_implicit_loc_<src_t, dest_t> {

        auto crit = _CritTaskSection_(_assigCH_);
        typedef std::remove_volatile_t<src_t> strip_src_t;
        typedef std::remove_volatile_t<dest_t> strip_dest_t;

        auto addr_mod = [this](const bool &e_stepsize) -> uintptr_t {
          return _descPtr_->BTCNT.bit.BTCNT 
            * (_descPtr_->BTCTRL.bit.BEATSIZE + 1) 
            * (e_stepsize ? (1 << _descPtr_->BTCTRL.bit.STEPSIZE) : 1); 
        };
        
        // Get & set alignment
        static constexpr int commAlign =
          !(sizeof(src_t) % bsRef[2]) && !(sizeof(dest_t) % bsRef[2]) ? 2 :
          !(sizeof(src_t) % bsRef[1]) && !(sizeof(dest_t) % bsRef[1]) ? 1 : 0;
        _descPtr_->BTCTRL.bit.BEATSIZE = commAlign;

        // Set increment config
        _descPtr_->BTCTRL.bit.SRCINC = std::is_array_v<strip_src_t> 
          || sizeof(strip_src_t) < CFG_MIN_INCR_SIZE;

        _descPtr_->BTCTRL.bit.DSTINC = std::is_array_v<strip_dest_t> 
          || sizeof(strip_dest_t) < CFG_MIN_INCR_SIZE;

        _descPtr_->BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;
        auto constexpr ssRes = [&]() consteval -> std::pair<int, int> {
          if constexpr (sizeof(src_t) < sizeof(dest_t)) {

            if constexpr (std::is_array_v<strip_src_t> 
              && std::is_array_v<strip_dest_t> 
              && sizeof(src_t) == bsRef[commAlign]) {

              return std::make_pair(DMAC_BTCTRL_STEPSEL_DST_Val, 
                indexOf(sizeof(dest_t) - sizeof(src_t), ssRef));

            } else if constexpr (!std::is_array_v<src_t> 
              && std::is_array_v<strip_dest_t>) {

              return std::make_pair(DMAC_BTCTRL_STEPSEL_DST_Val, 
                indexOf(sizeof(dest_t) - sizeof(src_t), ssRef));
            }
          }
          return std::make_pair(0, 0);
        }();
        static_assert(ssRes.second != -1, 
          "Internal error: Task type alignment.");
        _descPtr_->BTCTRL.bit.STEPSEL = ssRes.first;
        _descPtr_->BTCTRL.bit.STEPSIZE = ssRes.second;

        // Set address -> if volatile do not apply addr modifier
        _descPtr_->SRCADDR.bit.SRCADDR = mem_addr(src);
        _descPtr_->DSTADDR.bit.DSTADDR = mem_addr(dest);
        if constexpr (!std::is_volatile_v<src_t>) {
          _descPtr_->SRCADDR.bit.SRCADDR += addr_mod(ssRes.first);
        }
        if constexpr (!std::is_volatile_v<dest_t>) {
          _descPtr_->DSTADDR.bit.DSTADDR += addr_mod(ssRes.first);
        }
      }

      void *get_source(void) {
        if (_descPtr_->SRCADDR.bit.SRCADDR == DMAC_SRCADDR_RESETVALUE) {
          return nullptr;
        }
        return (void*)_descPtr_->SRCADDR.bit.SRCADDR;
      }

      void *get_dest(void) {
        if (_descPtr_->DSTADDR.bit.DSTADDR == DMAC_SRCADDR_RESETVALUE) {
          return nullptr;
        }
        return (void*)_descPtr_->DSTADDR.bit.DSTADDR;
      }

    private:

      uint8_t _assigCH_ = -1;
      Task *_linked_ = nullptr;
      DmacDescriptor *_descPtr_ = nullptr;

      bool _is_writeback_() {
        return _assigCH_ != -1 && wbDescArray[_assigCH_]
          .DESCADDR.bit.DESCADDR == _descPtr_->DESCADDR.bit.DESCADDR;
      }

  };  




  namespace {

    /// @internal Used for Task.set_location (auto)
    template<typename src_t, typename dest_t>
    concept _c_valid_implicit_loc_ = requires {
      std::is_trivially_copyable_v<std::decay_t
        <std::remove_cvref_t<src_t>>> &&
      std::is_trivially_copyable_v<std::decay_t
        <std::remove_cvref_t<dest_t>>> &&
      !std::is_reference_v<src_t> && 
      !std::is_reference_v<dest_t> &&
      !std::is_pointer_v<src_t> &&
      !std::is_pointer_v<dest_t> &&
      !std::is_unbounded_array_v<std::remove_cvref_t<src_t>> &&
      !std::is_unbounded_array_v<std::remove_cvref_t<dest_t>>; 
    };

    /// @internal Used for Task.set_location (specified)
    template<typename src_t, typename dest_t, int incrSrc_v, 
      int incrDest_v, int align_v>
    concept _c_valid_explicit_loc_ = requires {
      incrSrc_v >= 0 && 
      incrDest_v >= 0 &&
      align_v >= 0 &&
      indexOf(incrSrc_v, ssRef) != -1 &&
      indexOf(incrDest_v, ssRef) != -1 &&
      indexOf(align_v, bsRef) != -1 &&
      ((incrSrc_v > 1) & (incrDest_v > 1) == 0) &&
      !std::is_reference_v<src_t> &&
      !std::is_reference_v<dest_t> &&
      !std::is_pointer_v<src_t> &&
      !std::is_pointer_v<dest_t>;
    };

    class _CritTaskSection_ {
      public:
        _CritTaskSection_(int _index_) : _index_(std::move(_index_)) {
          if ((_index_ < 0 || _index_ >= DMAC_CH_NUM) 
            && !DMAC->Channel[_index_].CHINTFLAG.bit.SUSP) {

            DMAC->Channel[_index_].CHCTRLB.bit.CMD 
              = DMAC_CHCTRLB_CMD_SUSPEND_Val;
              
            _critStateMask_ |= (1 << _index_);
              while(DMAC->Channel[_index_].CHCTRLB.bit.CMD 
                == DMAC_CHCTRLB_CMD_SUSPEND_Val);
          }
        }
        ~_CritTaskSection_() {
          if ((_index_ < 0 || _index_ >= DMAC_CH_NUM) && _critStateMask_ 
              & (1 << _index_)) {
            DMAC->Channel[_index_].CHCTRLB.bit.CMD 
              = DMAC_CHCTRLB_CMD_RESUME_Val;
            _critStateMask_ &= ~(1 << _index_);
          }
        }
      private:
        const int _index_ = -1;
        static inline uint32_t _critStateMask_ = 0;
    };









  }

}



