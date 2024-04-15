
/// INFO: This file holds temporary implementations of functions
///       that are used by other code.

#pragma once
#include <sam.h>
#include <Arduino.h>
#include <utility>

#define SIO_ASSERT_RUNTIME_ENABLED false
#define SIO_ASSERT_COMPTIME_ENABLED true
#define SIO_THROW_COMPTIME_ENABLED true
#define SIO_THROW_RUNTIME_ENABLED false
#define SIO_WARN_COMPTIME_ENABLED true
#define SIO_WARN_RUNTIME_ENABLED true
#define SIO_ASSERT_BKPT_NUM 17
#define SIO_THROW_BKPT_NUM 18

  enum class prog_error_t {
    null,
    invalid_state
  };

  enum class warn_type {
    critical,
    null
  };



  constexpr void sio_warn(const warn_type &w_type, const char *msg) {
    if (std::is_constant_evaluated()) {
      #if SIO_WARN_COMPTIME_ENABLED
        #warning SIO WARNING TRIGGERED
      #endif
    } else {
      if constexpr (SIO_WARN_RUNTIME_ENABLED) {
        if (Serial) {
          Serial.print("SIO WARNING TRIGGERED:");
          Serial.print("Message: ");
          Serial.println(msg);
        }
      }
    }
  }



  /// @todo save to flash and possible stacktrace?
  constexpr void sio_throw(const prog_error_t &err) {
    if (std::is_constant_evaluated()) {
      if constexpr (SIO_THROW_COMPTIME_ENABLED) {
        static_assert("SIO ERROR THROWN");
      }
    } else {
      if constexpr (SIO_THROW_RUNTIME_ENABLED) {
        __disable_irq();
        __DMB();
        __BKPT(SIO_THROW_BKPT_NUM);
        for(;;);
      }
    }
  }



  /// @todo save (to flash?) file/line/msg params.
  constexpr bool sio_assert(const bool &cond) {
    if (!cond) {
      if (std::is_constant_evaluated()) {
        if constexpr (SIO_ASSERT_COMPTIME_ENABLED) {
          static_assert("SIO ASSERT TRIGGERED");
        }
      } else {
        if constexpr (SIO_ASSERT_RUNTIME_ENABLED) {
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
