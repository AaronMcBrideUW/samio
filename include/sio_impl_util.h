
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



  /// @brief Returns memory address of object/variable.
  /// @param value Target value of arbitrary type.
  /// @return 32 bit unsigned integer -> uintptr_t.
  template<typename T>
  uintptr_t mem_addr(const T &value) {
    return reinterpret_cast<uintptr_t>(std::addressof(value));
  }
  template<typename T>
  uintptr_t mem_addr(const T *&value) {
    return reinterpret_cast<uintptr_t>(value);
  }

  template<typename T>
  requires requires { std::is_arithmetic_v<T>; }
  T bound(const T &value, const T &min, const T &max) {
    return value < min ? min : (value > max - 1 ? max - 1 : value);
  }

  template<typename T>
  requires requires { std::is_arithmetic_v<T>; }
  T bound_min(const T &value, const T &min) {
    return value < min ? min : value;
  }

  template<typename T>
  requires requires { std::is_arithmetic_v<T>; }
  T bound_max(const T &value, const T&max) {
    return value > max ? max : value;
  }

  template<typename T1, typename T2, unsigned int N>
  requires requires { N > 0 && std::is_convertible_v<T1, T2>; }
  constexpr int indexOf(const T1 &value, const T2 (&arr)[N]) 
  {
    for (int i = 0; i < N; i++) {
      if (arr[i] == value) {
        return i;
      }
    }
    return -1;
  }

  template<typename T, unsigned int N>
  requires requires { N > 0; }
  constexpr bool is_contained(const T &value, const T (&arr)[N]) 
  {
    return indexOf<T, N>(value, arr) != -1;
  }

  // TO DO -> Ensure that universal ref type is correct for prupose &
  // see about adding move semantics to these functions
  template<typename num1_t, typename num2_t>
  constexpr bool is_multiple(const num1_t &num1,const num2_t &num2) noexcept
     requires c_valid_div<num1_t, num2_t> 
  {
    if (num1 == 0 || num2 == 0) {
      return false;
    }
    return num1 > num2 ? num2 % num1 : num1 % num2;
  }

  template<typename den_t, typename num_t>
  constexpr auto safe_div(const den_t &den,const num_t &num) noexcept 
    requires c_valid_div<den_t, num_t> 
  { 
    return den ? den / num : den;
  } 

  template<typename den_t, typename num_t>
  concept c_valid_div = requires {
    std::is_integral_v<std::remove_cvref_t<num_t>> && 
    std::is_integral_v<std::remove_cvref_t<den_t>>; 
  };
}