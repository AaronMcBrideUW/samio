
#pragma once
#include <sam.h>
#include <utility>
#include <any>
#include <type_traits>

#define ASSERT_BKPT_NUM 21

namespace sio::impl::util {


  /// \b TO-DO

  struct global_config_t {
    static constexpr bool assert_enabled = true; 
  };
  constexpr global_config_t global_config;


  /// \b TO-DO

  /// @brief Terminates program if condition evaluates to false.
  /// @param cond Condition to evaluate.
  /// @return bool - Result of condition.
  constexpr inline bool assert(const bool &cond) noexcept;


  /// \b TO-DO

  constexpr inline void warn(const char *msg) {
    return;
  }





  


}