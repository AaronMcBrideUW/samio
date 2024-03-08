
#pragma once
#include <sam.h>
#include <utility>
#include <type_traits>

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


  template<typename T>
  constexpr T _swap(T &obj1, T &obj2) 
    noexcept(std::is_nothrow_swappable_v<T>)
                                              /// \b TO-DO->ADD_CONSTRAINTS
  {
    T temp = std::move(obj1);
    obj1 = std::move(obj2);
    obj2 = std::move(temp);
  }

  template<typename T1, typename T2 = T1>
  constexpr T1 _assign(T1 &assignTo, T2 &&valueFrom) 
    noexcept(std::is_nothrow_assignable_v<T1, T2>)
                                                /// \b TO-DO->ADD_CONSTRAINTS
  {
    T1 temp = std::move(assignTo);
    assignTo = std::forward<T2>(valueFrom);
    return temp;
  }
  template<typename T1, typename T2>
  concept assign_c = requires (T1 &val1, T2 &&val2, T1 &val1_test) {
    std::is_assignable_v<T1, T2>;
    val1_test = val1;
    val1 = val2;
  };

  /// \b NOT-DONE

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


  /// @brief Gets the memory address of an object
  /// @param _value_ Target object of arbitrary type.
  /// @return A 32 bit unsigned integer.
  template<typename T>
  uintptr_t mem_addr(const T &_value_) noexcept {
    return reinterpret_cast<uintptr_t>(std::addressof(_value_));
  }

  template<typename T>
  uintptr_t mem_addr(const T *&_value_) noexcept {
    return reinterpret_cast<uintptr_t>(_value_);
  }

  template<typename T>
  requires requires { std::is_arithmetic_v<T>; }
  T bound(const T &_value_, const T &min, const T &max) {
    return _value_ < min ? min : (_value_ > max - 1 ? max - 1 : _value_);
  }

  template<typename T>
  requires requires { std::is_arithmetic_v<T>; }
  T bound_min(const T &_value_, const T &min) {
    return _value_ < min ? min : _value_;
  }

  template<typename T>
  requires requires { std::is_arithmetic_v<T>; }
  T bound_max(const T &_value_, const T&max) {
    return _value_ > max ? max : _value_;
  }

  template<typename T1, typename T2, unsigned int N>
  requires requires { N > 0 && std::is_convertible_v<T1, T2>; }
  constexpr int indexOf(const T1 &_value_, const T2 (&arr)[N]) 
  {
    for (int i = 0; i < N; i++) {
      if (arr[i] == _value_) {
        return i;
      }
    }
    return -1;
  }

  template<typename T, unsigned int N>
  requires requires { N > 0; }
  constexpr bool is_contained(const T &_value_, const T (&arr)[N]) 
  {
    return indexOf<T, N>(_value_, arr) != -1;
  }

  // NEED TO LOOK OVER THESE...
  template<typename num1_t, typename num2_t>
  constexpr bool is_multiple(const num1_t &num1,const num2_t &num2) noexcept
  {
    if (num1 == 0 || num2 == 0) {
      return false;
    }
    return num1 > num2 ? num2 % num1 : num1 % num2;
  }

  template<typename den_t, typename num_t>
  constexpr auto safe_div(const den_t &den,const num_t &num) noexcept 
  { 
    return den ? den / num : den;
  } 
}