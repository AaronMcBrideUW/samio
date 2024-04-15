
#pragma once
#include <sam.h>

namespace sioc::temp {

  /// @a FINAL_V1
  struct atomic_section {

    atomic_section() {
      init_primsk = __get_PRIMASK();
      __disable_irq();
      __DMB();
    }

    ~atomic_section() {
      __DMB();
      __set_PRIMASK(init_primsk);
    }

    private:
      using atomic_t = decltype(__get_PRIMASK());
      atomic_t init_primsk{};
  };



}