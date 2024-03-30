
/// INFO: This file holds temporary implementations of functions
///       that are used by other code.

#pragma once
#include <sam.h>
#include <utility>

#define SIO_ASSERT_RUNTIME_ENABLED false
#define SIO_ASSERT_COMPTIME_ENABLED true
#define SIO_ASSERT_BKPT_NUM 17


  /// @todo save (to flash?) file/line/msg params.
  constexpr bool sio_assert(const bool &cond) {
    if (!cond) {
      if (std::is_constant_evaluated()) {
        if (SIO_ASSERT_COMPTIME_ENABLED) {
          static_assert("SIO ASSERT TRIGGERED");
        }
      } else {
        if (SIO_ASSERT_RUNTIME_ENABLED) {
          __disable_irq();
          __DMB();
          __BKPT(SIO_ASSERT_BKPT_NUM);
          for(;;);
        }
      }
    }
    return cond;
  }

  constexpr bool sio_assert() { return sio_assert(true); }
