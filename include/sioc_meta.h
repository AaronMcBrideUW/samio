#pragma once
#include <sio_error.h>
#include <utility>
#include <source_location>
#include <vector>
#include <cstddef>





namespace sioc::meta 
{

  /// @a FINAL_V1
  template<typename inst_t>
  struct StaticCounter {

    template<unsigned cnt_v>
    struct FlagGetter {
      friend auto count_flag(FlagGetter<cnt_v>);
    };

    template<unsigned cnt_v>
    struct FlagSetter {
      friend auto count_flag(FlagGetter<cnt_v>) {}
      static constexpr unsigned n = cnt_v;
    };

    template<auto unique_v, unsigned next_v = 0>
    [[nodiscard]] static consteval auto ExecuteCount() {
      constexpr bool cnt_overflow = requires(FlagGetter<next_v> get) {
          count_flag(get);
      };
      if constexpr (cnt_overflow) {
        return ExecuteCount<unique_v, next_v + 1>();
      } else {   
        FlagSetter<next_v> s;
        static_assert(s.n < max_cnt, "SIO ERROR (INTERNAL): static count overflow.");
        return &s.n;
      }
    }
    template<auto unique_v, auto count_v = ExecuteCount<unique_v>()>
    static constexpr unsigned int &_static_count_ = count_v;
  };

  // #define static_count(_inst_) StaticCounter<_inst_>::template _static_count_<[]{}>

  template<typename T>
  constexpr unsigned int &static_counter = StaticCounter<T>::_static_count_<[]{}>; 

}


