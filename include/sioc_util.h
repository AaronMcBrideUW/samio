
#pragma once
#include <sam.h>
#include <utility>


#define INIT_SEQ []{ static int cnt{0}; return cnt++; }()


namespace sioc::util {

  


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

  /// @brief Divides two numbers & rounds the result UP to the greates common factor.
  /// @tparam T Type of numerator, denominator and returned value.
  /// @param numerator 
  /// @param denominator 
  template<std::integral T>  
  T div_ceil(const T &numerator, const T &denominator) {
    return numerator / denominator + (((numerator < 0) ^ (denominator > 0)) && 
        (numerator%denominator));
  }



}