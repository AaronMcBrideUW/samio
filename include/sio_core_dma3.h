///////////////////////////////////////////////////////////////////////////////////////////////////
//// FILE: DIRECT MEMORY ACCESS (DMA) 
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <sio_core_config.h>
#include <sio_temp.h>
#include <sam.h>
#include <utility>
#include <algorithm>
#include <iterator>
#include <string.h>
#include <bit>
#include <tuple>

namespace sio::core {

  struct SysModule;
  struct PrilvlModule;
  struct ChannelModule;
  struct DescriptorModule;
  struct TransferDescriptor;
  struct DmaPeripheral;

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: ENUMS & TYPEDEFS
///////////////////////////////////////////////////////////////////////////////////////////////////

  enum class INTERRUPT_FLAG {
    TRANSFER_COMPLETE,
    TRANSFER_ERROR,
    DESCRIPTOR_ERROR
  };

  enum class CHANNEL_STATE {
    DISABLED,
    IDLE,
    SUSPENDED,
    ACTIVE,
  };

  enum class TRANSFER_MODE : int {
    PACKET  = DMAC_CHCTRLA_TRIGACT_BURST_Val,
    TASK    = DMAC_CHCTRLA_TRIGACT_BLOCK_Val,
    ALL     = DMAC_CHCTRLA_TRIGACT_TRANSACTION_Val
  };

  enum class LINKED_PERIPHERAL : int {
    ADC,
    /// TBC
  };

  using ch_interrupt_t = void (*)(const int, const INTERRUPT_FLAG); 
  using length_t = decltype(std::declval<DmacDescriptor>().BTCNT.reg);

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: BASE MODULE
///////////////////////////////////////////////////////////////////////////////////////////////////

  struct BaseModule {
    Dmac *const _reg_;

    ch_interrupt_t _cbarray_[DMAC_CH_NUM]{};
    TransferDescriptor *_btd_[DMAC_CH_NUM]{};
    DmacDescriptor _wbdesc_[DMAC_CH_NUM]{};
    DmacDescriptor _bdesc_[DMAC_CH_NUM]{};

    static constexpr int bs_ref[3]      = {1, 2, 4};  
    static constexpr int ss_ref[8]      = {1, 2, 4, 8, 16, 32, 62, 128};
    static constexpr int bus_ref[16]    = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    static constexpr int thresh_ref[4]  = {1, 2, 4, 8};
    static constexpr int chpri_ref[4]   = {1, 2, 3, 4};
    static constexpr int squal_ref[4]   = {1, 2, 3, 4};
    static constexpr int irq_pri_ref    = 1;
    static constexpr std::pair<uintptr_t, uintptr_t> ram_addr_ref[] = {
      {FLASH_ADDR, FLASH_ADDR + FLASH_SIZE}, 
      {HSRAM_ADDR, HSRAM_ADDR + HSRAM_SIZE}, 
      {BKUPRAM_ADDR, BKUPRAM_ADDR + BKUPRAM_SIZE}
    };
  }; 

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: SYSTEM MODULE DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////

  struct SysModule {
    BaseModule *const _base_;

    void initialized(const bool &init) noexcept;    
    bool initialized() const;

    bool enabled(const bool &enabled) noexcept;
    bool enabled() const noexcept;

    inline int active_index() const noexcept;

    inline int pend_count() const noexcept;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: PRIORITY LEVEL MODULE DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////
  
  struct PrilvlModule {
    BaseModule *const _base_;
    const int _lvl_;
   
    void round_robin_mode(const bool &enabled) noexcept;
    inline bool round_robin_mode() const noexcept;

    bool service_quality(const int &quality_lvl) noexcept;
    int service_quality() const noexcept;
    
    void enabled(const bool &enabled) noexcept;
    inline bool enabled() const noexcept;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER DESCRIPTOR DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////

  struct TransferDescriptor {

    TransferDescriptor() noexcept = default;
    TransferDescriptor(const TransferDescriptor &other) noexcept;    
    TransferDescriptor(TransferDescriptor &&other) noexcept;

    TransferDescriptor &operator = (const TransferDescriptor &other) noexcept;
    TransferDescriptor &operator = (TransferDescriptor &&other) noexcept;

    explicit inline operator bool() noexcept;

    template<typename src_t, int _incr_ = -1>
    void source(src_t *src_ptr, int increment = -1) noexcept {
      set_location<true, src_t, _incr_>(*this, src_ptr); 
    }
    inline void *source() const noexcept;

    template<typename dst_t, bool _incr_ = -1>
    void destination(dst_t *dst_ptr, int increment = -1) noexcept {
      set_location<false, dst_t, _incr_>(*this, dst_ptr); 
    }    
    inline void *destination() const noexcept;
    
    bool length(const length_t &bytes) noexcept;
    inline length_t length() const noexcept;
    
    inline void suspend_mode(const bool &enabled) noexcept;
    inline bool suspend_mode() const noexcept;
    
    inline DescriptorModule *assigned_list() const noexcept;
    
    inline void unlink() noexcept;

    ~TransferDescriptor() noexcept;

    struct {
      uint16_t _srcsize_;
      uint16_t _dstsize_;
      bool _srcmod_ = false;
      bool _dstmod_ = false;
    }_cfg_;
    DescriptorModule *_assig_;
    TransferDescriptor *_next_ = nullptr;
    DmacDescriptor *_descPtr_ = &_desc_;
    DmacDescriptor _desc_{};
    
    unsigned int addr_mod(const bool &is_src) const noexcept;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DESCRIPTOR MODULE DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////


  struct DescriptorModule {
    BaseModule *const _base_;
    const int _index_;
    bool _looped_ = false;

    bool set(TransferDescriptor **desc_list, const size_t &length) noexcept;

    template<int N>
    inline bool set(TransferDescriptor *(desc_list)[N]) {
      return set(desc_list, N);
    }

    template<TransferDescriptor& ...desc_list>
    inline bool set(TransferDescriptor&...) noexcept {
      TransferDescriptor *desc_array[sizeof...(desc_list)] = { (&desc_list)... };
      return set(desc_array, sizeof...(desc_list));
    }

    inline bool set(TransferDescriptor& desc) noexcept;

    bool add(const int &index, TransferDescriptor &targ) noexcept;

    TransferDescriptor *get(const int &index) noexcept;

    TransferDescriptor &operator [] (const int &index) noexcept;

    TransferDescriptor *get_last() noexcept;

    TransferDescriptor *remove(const unsigned int &index) noexcept;
    inline void remove(TransferDescriptor &targ) noexcept;

    void clear(const bool &clear_active) noexcept;

    size_t size() const noexcept;

    int indexOf(TransferDescriptor &targ) const noexcept;
    
    void looped(const bool &enabled) noexcept;
    inline bool looped() const noexcept;
  };

  /// TO DO -> ADD CHECKING FOR BASE INIT
  
  struct ChannelModule {
    using enum CHANNEL_STATE;
    BaseModule *const _base_;
    DescriptorModule *const _desc_;
    const int _index_;

    bool state(const CHANNEL_STATE &value) noexcept;
    CHANNEL_STATE state() const noexcept;

    inline void reset_transfer(const bool &forced = false) noexcept;

    void reset_channel() noexcept;

    bool queue_transfer(const int &index, const bool &forced = false) noexcept;

    inline bool is_busy() const noexcept;

    inline int remaining_bytes(const bool &in_elements) const noexcept;

    int current_index() const noexcept;
    
    void linked_peripheral(const LINKED_PERIPHERAL &periph) noexcept;
    inline LINKED_PERIPHERAL linked_peripheral() const noexcept;

    bool packet_size(const int &elements) noexcept;
    inline bool packet_size(const int &) const noexcept;

    void transfer_mode(const TRANSFER_MODE &mode) noexcept;
    inline TRANSFER_MODE transfer_mode() const noexcept;

    bool priority_lvl(const int &lvl) noexcept;
    inline int priority_lvl() const noexcept;

    inline void interrupt_callback(ch_interrupt_t interrupt_fn) noexcept;
  };

  #define init_seq ([]{static int i; return i++;}())

  struct DmaPeripheral {
    DmaPeripheral(const int &index) : _index_(index) {} 
    const int _index_;
    BaseModule base{DMAC + _index_};
    SysModule sys{&base};
    PrilvlModule prilvl[DMAC_LVL_NUM]{{&base, init_seq}};
    DescriptorModule descList[DMAC_CH_NUM]{{&base, init_seq}};
    ChannelModule channel[DMAC_CH_NUM]{{&base, &descList[init_seq], init_seq}};
  };

}
