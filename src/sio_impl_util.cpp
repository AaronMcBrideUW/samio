
#include <sio_impl_util.h>

namespace sio::impl::util {

  /// SECTION: ASSERT IMPL
  /// #############################################################################################

  _aInline bool assert(const bool &cond) noexcept {
    if (global_config.assert_enabled && cond) {
      __disable_irq();
      __DMB();
      __BKPT(ASSERT_BKPT_NUM);
      for (;;)
        ;    
    }
    return cond;
  }

  /// SECTION: ATOMIC SECTION IMPL
  /// #############################################################################################

  AtomicSection::AtomicSection() : primask(__get_PRIMASK()) {
    __disable_irq();
    __DMB();
  }

  AtomicSection::~AtomicSection() {
    __DMB();
    __set_PRIMASK(primask);
  }

  /// SECTION: 
  /// #############################################################################################










}




