
#pragma once
#include <sam.h>
#include <sio_impl_util.h>
#include <sio_defs.h>
#include <string.h>
#include <utility>
#include <math.h>

namespace sio::impl::dma {
  

  enum class channel_error {};

  enum class channel_state {};

  enum class transfer_mode {};

  enum class channel_peripheral : int {};

  typedef void (*error_cb_t)(const int&, const channel_error&);
  typedef void (*transfer_cb_t)(const int&);


  struct active_channel_t {
  
    static _aInline int index(void);

    static _aInline int remaining_bytes(void);
    
    static _aInline channel_state state(void);
  
  };
  inline active_channel_t active_channel;


  struct sys_ctrl_t {

    static bool set_init(const bool&);
    static bool get_init(void);

    static bool set_enabled(const bool&);
    static bool get_enabled(void);

  };
  inline sys_ctrl_t sys_ctrl;


  class Channel {
    public:
      Channel(); // DONE

      bool set_state(const channel_state&);
      channel_state get_state(void);

      bool reset_transfer(void);

      bool set_tasks(Task&...);

      _aInline Task &get_current_task(void);

      channel_error get_error(void);

      struct {

        bool linked_peripheral(const channel_peripheral&);

        bool error_callback(error_cb_t);

        bool transfer_callback(transfer_cb_t);

        bool transfer_mode(const transfer_mode&);

        bool beat_size(const int&);

        bool priority_lvl(const int&);

      }config;

    private:
      int index;
      Task *baseTask;
  };


  class Task { 
    friend Channel;

    public:

      /// @brief Default allocating constructor   
      Task(void) {
        _descPtr_ = new DmacDescriptor;
        _alloc_ = true;
      }

      /// @brief Copy constructor 
      Task (const Task &other) {
        if (this != &other) {
          this->operator = (other);
        }
      } 

      /// @brief Move constructor
      Task(Task &&other) noexcept {
        if (this != &other) {
          this->operator = (std::move(other));
        }
      }

      /// @brief Copy assignment operator
      Task &operator = (const Task &other) {
        if (this != &other) {
          CritTaskSection_(this->_assigCH_);
          auto sAddr = _descPtr_->DESCADDR.bit.DESCADDR;
          memcpy(_descPtr_, other._descPtr_, sizeof(DmacDescriptor));
          _descPtr_->DESCADDR.bit.DESCADDR = sAddr;
        }
      }

      /// @brief Move assignment operator 
      Task &operator = (Task &&other) {
        if (this != &other) {
          CritTaskSection_(this->_assigCH_);
          CritTaskSection_(other._assigCH_);
          _descPtr_ = std::exchange(other._descPtr_, _descPtr_);
          _assigCH_ = std::exchange(other._assigCH_, _assigCH_);
          _linked_ = std::exchange(other._linked_, _linked_);
          _alloc_ = std::exchange(other._alloc_, _alloc_);
        }
      }

      bool set_transfer_size(const bool&);
      int get_transfer_size(void);

      bool set_suspend_channel(const bool&);
      bool get_suspend_channel(void);  

      bool reset();

      ~Task();

      typedef int src_t;
      typedef int dest_t;
      bool set_location(const src_t __restrict*, const dest_t __restrict*) {
        static constexpr uint8_t bsRef[] = {1, 2, 4};  

        // [x] Set alignment 
        // [ ] Set increment config
        // [ ] Set addresses

        auto constexpr get_align = []() constexpr {
          if constexpr (!(sizeof(src_t) % bsRef[2]) 
             && !(sizeof(dest_t) % bsRef[2])) {
            return 2;
          } else if constexpr (!(sizeof(src_t) % bsRef[1]) 
            && !(sizeof(dest_t) % bsRef[1])) {
            return 1;
          }
          return 0;
        };
        static constexpr int commAlign = get_align();
        _descPtr_->BTCTRL.bit.BEATSIZE = commAlign;

        static_assert((std::is_array_v<src_t> && std::is_array_v<dest_t>
          && sizeof(src_t) > commAlign && sizeof(dest_t) > commAlign) == false,
          "Task is ill-formed: Only one incrementing location allowed.");

          _descPtr_->BTCTRL.bit.SRCINC = std::is_array_v<src_t>;      
          _descPtr_->BTCTRL.bit.DSTINC = std::is_array_v<dest_t>;

          if constexpr (sizeof(src_t) > )

      }
      void *get_source(void);
      void *get_dest(void);

    private:

      uint8_t _assigCH_ = -1;
      bool _alloc_ = false;
      Task *_linked_ = nullptr;
      DmacDescriptor *_descPtr_ = nullptr;

      class CritTaskSection_ {
        public:
          CritTaskSection_(int index) : index(std::move(index)) {
            if ((index < 0 || index >= DMAC_CH_NUM) && !DMAC->Channel[index] 
                .CHINTFLAG.bit.SUSP) {
              DMAC->Channel[index].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_SUSPEND_Val;
              critStateMask |= (1 << index);
              while(DMAC->Channel[index].CHCTRLB.bit.CMD == DMAC_CHCTRLB_CMD_SUSPEND_Val);
            }
          }
          ~CritTaskSection_() {
            if ((index < 0 || index >= DMAC_CH_NUM) && critStateMask 
                & (1 << index)) {
              DMAC->Channel[index].CHCTRLB.bit.CMD = DMAC_CHCTRLB_CMD_RESUME_Val;
              critStateMask &= ~(1 << index);
            }
          }
        private:
          const int index = -1;
          static inline uint32_t critStateMask = 0;
      };

  };  





  namespace {









  }

}



