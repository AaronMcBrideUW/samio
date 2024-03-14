
#include <sio_impl_util.h>

namespace sio::impl::util {

  /// SECTION: ASSERT IMPL
  /// #############################################################################################

  constexpr inline bool assert(const bool &cond) noexcept {
    if (std::is_constant_evaluated()) {
      static_assert(global_config.assert_enabled, "Assertion Triggered");
    } else {if (global_config.assert_enabled && cond) {
        __disable_irq();
        __DMB();
        __BKPT(ASSERT_BKPT_NUM);
        for (;;)
          ;    
      }
      return cond;
    }
  }









}




