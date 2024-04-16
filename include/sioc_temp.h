
#pragma once
#include <sam.h>
#include <concepts>
#include <math.h>

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


  /// @a FINAL_V1
  template<std::integral N, std::integral D>
    requires (std::is_nothrow_convertible_v<N, D>)
  std::common_type_t<N, D> div_ceil(const N &numerator, const D &denominator) {
    div_t div_result = div(numerator, denominator);
    return div_result.quot + div_result.rem;
  } 

} 