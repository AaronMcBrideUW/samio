
#pragma once
#include <sam.h>
#include <utility>
#include <experimental/type_traits>

#define _aInline __attribute__((always_inline))
#define ASSERT_BKPT_NUM 21

namespace sio::impl::util {

  struct global_config_t {
    static inline bool assert_enabled = true; 

  };
  inline global_config_t global_config;

  

  /// @brief Terminates program if condition evaluates to false.
  /// @param cond Condition to evaluate.
  /// @return bool - Result of condition.
  _aInline bool assert(const bool &cond) noexcept;



  class AtomicSection {
    public:

      /// @brief When object is constructed an atomic section is
      //    automatically entered.
      AtomicSection();
  
      /// @brief When object is destroyed previously begun atomic
      ///   section is automatically left.
      ~AtomicSection();      

    private:
      const decltype(__get_PRIMASK()) primask;
  };



  /// @brief Gets memory address of object.
  /// @return 32 bit unsigned integer (uintptr_t) 
  template<typename T>
  uintptr_t mem_addr(const T &value) {
    return reinterpret_cast<uintptr_t>(std::addressof(value));
  }
  template<typename T>
  uintptr_t mem_addr(const T *&value) {
    return reinterpret_cast<uintptr_t>(value);
  }

  template<typename T>
  requires requires { std::is_integral_v<T>; }
  T bound(const T &value, const T &min, const T &max) {
    return value < min ? min : (value > max ? max : value);
  }

  template<typename T>
  requires requires { std::is_integral_v<T>; }
  T bound_min(const T &value, const T &min) {
    return value < min ? min : value;
  }

  template<typename T>
  requires requires { std::is_integral_v<T>; }
  T bound_max(const T &value, const T&max) {
    return value > max ? max : value;
  }

  template<typename T, unsigned int N>
  requires requires { N > 0; }
  int indexOf(const T &value, const T (&arr)[N]) {
    for (int i = 0; i < N; i++) {
      if (arr[i] == value) {
        return i;
      }
    }
    return -1;
  }

  template<typename T, unsigned int N>
  requires requires { N > 0; }
  bool is_contained(const T &value, const T (&arr)[N]) {
    return indexOf<T, N>(value, arr) != -1;
  }

  template<typename den_t, typename num_t>
  requires requires { std::is_integral_v<den_t> && std::is_integral_v<num_t> 
    && std::is_convertible_v<den_t, num_t>; }
  auto safe_div(const long &den, const long &num) noexcept { 
    return den ? den / num : den;
  } 

}