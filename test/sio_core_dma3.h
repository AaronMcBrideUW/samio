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

  enum class TRANSFER_MODE : unsigned int {
    PACKET  = DMAC_CHCTRLA_TRIGACT_BURST_Val,
    TASK    = DMAC_CHCTRLA_TRIGACT_BLOCK_Val,
    ALL     = DMAC_CHCTRLA_TRIGACT_TRANSACTION_Val
  };

  enum class LINKED_PERIPHERAL : unsigned int {
    NONE,
    ADC,
    /// TBC
  };

  using ch_interrupt_t = void (*)(const unsigned int, const INTERRUPT_FLAG); 
  using length_t = decltype(std::declval<DmacDescriptor>().BTCNT.reg);

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: BASE MODULE
///////////////////////////////////////////////////////////////////////////////////////////////////

  struct BaseModule {
    Dmac *const _reg_;

    ch_interrupt_t _cbarray_[DMAC_CH_NUM]{};
    TransferDescriptor *_btd_[DMAC_CH_NUM]{};

    DmacDescriptor _wbdesc_[DMAC_CH_NUM];
    DmacDescriptor _bdesc_[DMAC_CH_NUM];

    static constexpr uint8_t bs_ref[3]      = {1, 2, 4};  
    static constexpr uint8_t ss_ref[8]      = {1, 2, 4, 8, 16, 32, 62, 128};
    static constexpr uint8_t bus_ref[16]    = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    static constexpr uint8_t thresh_ref[4]  = {1, 2, 4, 8};
    static constexpr uint8_t chpri_ref[4]   = {1, 2, 3, 4};
    static constexpr uint8_t squal_ref[4]   = {1, 2, 3, 4};
    static constexpr uint8_t irq_pri_ref    = 1;
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

    /// @b TESTED
    void initialized(const bool &init) noexcept;    
    bool initialized() const;

    /// @b TESTED
    bool enabled(const bool &enabled) noexcept;
    bool enabled() const noexcept;

    int active_index() const noexcept;

    int pend_count() const noexcept;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: PRIORITY LEVEL MODULE DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////
  
  struct PrilvlModule {
    BaseModule *const _base_;
    const int _lvl_;
   
    void round_robin_mode(const bool &enabled) noexcept;
    bool round_robin_mode() const noexcept;

    bool service_quality(const unsigned int &quality_lvl) noexcept;
    int service_quality() const noexcept;
    
    void enabled(const bool &enabled) noexcept;
    bool enabled() const noexcept;
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: TRANSFER DESCRIPTOR DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////

  struct TransferDescriptor {

    TransferDescriptor() noexcept = default;

    template<typename src_t, typename dst_t>
    TransferDescriptor(const src_t *src_ptr, const dst_t *dst_ptr, 
      const int &bytes) {
      set_location<true, src_t, -1>(*this, src_ptr);
      set_location<false, dst_t, -1>(*this, dst_ptr);
      length(bytes);
    }

    TransferDescriptor(const TransferDescriptor &other) noexcept 
      { this->operator=(other); }

    TransferDescriptor(TransferDescriptor &&other) noexcept 
      { this->operator=(std::move(other)); }

    TransferDescriptor &operator = (const TransferDescriptor &other) noexcept;
    TransferDescriptor &operator = (TransferDescriptor &&other) noexcept;

    bool valid() const noexcept;
    explicit operator bool() const noexcept { return valid(); }

    /// @internal @b DONT-USE -> Prefer following methods
    void set_location(const bool &, void *, const size_t &, const int &, 
      const bool &) noexcept;

    template<typename src_t>
    void source(src_t &src, int increment = -1) noexcept {
      set_location(true, (void*)std::addressof(src), 
        sizeof(std::remove_pointer_t<src_t>), increment, false); 
    }
    template<typename src_t, size_t N>
    void source(src_t (&src)[N], int increment = -1) {
      set_location(true, (void*)std::addressof(src), 
      sizeof(std::remove_pointer_t<src_t>) * N, increment, true);
    }
    void *source() const noexcept;

    template<typename dst_t>
    void dest(dst_t &dst, int increment = -1) noexcept {
      set_location(false, (void*)std::addressof(dst), 
        sizeof(std::remove_pointer_t<dst_t>), increment, false); 
    }   
    template<typename dst_t, size_t N>
    void dest(dst_t (&dst)[N], int increment = -1) {
      set_location(false, (void*)std::addressof(dst), 
        sizeof(std::remove_pointer_t<dst_t>) * N, increment, true);
    } 
    void *dest() const noexcept;
    
    bool length(const length_t &bytes) noexcept;
    length_t length() const noexcept;
    
    void suspend_mode(const bool &enabled) noexcept;
    bool suspend_mode() const noexcept;

    TransferDescriptor *linked_descriptor() const noexcept
      { return _next_; }
    
    ChannelModule *assigned_channel() const noexcept 
      { return _assig_; }

    void unlink();

    ~TransferDescriptor() noexcept { unlink(); }

    __PACKED_STRUCT {
      uint16_t _srcsize_ = 0;
      uint16_t _dstsize_ = 0;
      bool _srcmod_ = false;
      bool _dstmod_ = false;
    }_cfg_;
    ChannelModule *_assig_ = nullptr;
    TransferDescriptor *_next_ = nullptr;
    DmacDescriptor *_descPtr_ = &_desc_;
    DmacDescriptor _desc_{};
  };

///////////////////////////////////////////////////////////////////////////////////////////////////
//// SECTION: DESCRIPTOR MODULE DECLARATION
///////////////////////////////////////////////////////////////////////////////////////////////////
  
  struct ChannelModule {
    using enum CHANNEL_STATE;
    BaseModule *const _base_;
    const int _index_;

    void set_transfer(TransferDescriptor**, const size_t&,
      const bool &looped = false);

    template<int N>
    void set_transfer(TransferDescriptor *(&desc_array)[N], 
      const bool &looped = false) {
      set_transfer(desc_array, N, looped); 
    }

    void set_transfer(TransferDescriptor *desc, const bool &looped = false) {
      TransferDescriptor *desc_array[] = {desc};
      set_transfer(desc_array, (bool)desc, looped);
    }

    bool state(const CHANNEL_STATE &value) noexcept;
    CHANNEL_STATE state() const noexcept;

    bool is_active() const noexcept;

    void reset_channel() noexcept;

    void reset_transfer(const bool &current_only) noexcept;

    bool queue_transfer(const unsigned int &index) noexcept;

    TransferDescriptor *current_transfer() const noexcept;

    bool remaining_bytes(const length_t &bytes) noexcept;
    length_t remaining_bytes() const noexcept;
    
    void linked_peripheral(const LINKED_PERIPHERAL &periph) noexcept;
    LINKED_PERIPHERAL linked_peripheral() const noexcept;

    bool packet_size(const unsigned int &elements) noexcept;
    bool packet_size() const noexcept;

    void transfer_mode(const TRANSFER_MODE &mode) noexcept;
    TRANSFER_MODE transfer_mode() const noexcept;

    bool priority_lvl(const unsigned int &lvl) noexcept;
    int priority_lvl() const noexcept;

    void enabled_durring_standby(const bool &enabled) noexcept;
    bool enabled_durring_standby() const noexcept;

    void interrupt_callback(ch_interrupt_t interrupt_fn) noexcept;
  };

  extern void dma_irq_handler(volatile BaseModule*);

  #define init_seq ([]{static int i; return i++;}())
  
  struct {
    const int _index_;
    BaseModule base{DMAC + _index_};
    SysModule sys{&base};
    PrilvlModule prilvl[DMAC_LVL_NUM]{{&base, init_seq}};
    ChannelModule channel[DMAC_CH_NUM]{{&base, init_seq}};
  }dma[DMAC_INST_NUM]{init_seq};

  static volatile BaseModule *irq_base = &dma[0].base; 
  static void DMAC_0_Handler() { dma_irq_handler(irq_base); }
  static void DMAC_1_Handler() { dma_irq_handler(irq_base); }
  static void DMAC_2_Handler() { dma_irq_handler(irq_base); }
  static void DMAC_3_Handler() { dma_irq_handler(irq_base); }
  static void DMAC_4_Handler() { dma_irq_handler(irq_base); }

}

