
#pragma once
#include <sam.h>
#include <core_dma_defs.h>
#include <impl_util.h>

namespace sioc::dma {

    struct {

        static inline transferCB_t transferCB = nullptr;
        static inline errorCB_t errorCB = nullptr;
        static inline int irq_priority = 1;

        static inline bool prilvl_enabled[4] = { true };
        static inline bool round_robin[DMAC_LVL_NUM] = { true, true, true, true };
        static inline int service_quality[DMAC_LVL_NUM] = { 1, 1, 1, 1 };

    }config;



    struct {

      static bool set_init(const bool&);
      static bool get_init(void);

      static bool set_enabled(const bool&);
      static bool get_enabled(void);

    }sys;

    struct {

      static int get_index(void);

      static bool get_busy(void);

      static int get_remaining(void);

    }active_ch; 

    
    struct crc_module {

      static bool set_crc_size(const int&);
      static int get_crc_size(void);

      static bool set_channel(const int&);
      static int get_channel(void);

    }crc;

    template<int index>
    class channel {

      static bool set_init(const bool&);
      static bool get_init(void);

      static bool set_enabled(const bool&);
      static bool get_enabled(void);

      static bool set_active(const bool&);
      static bool get_active(void);

      static bool set_suspend(const bool&);
      static bool get_suspend(void);      

      static bool set_peripheral(const channel_peripheral&);
      static channel_peripheral get_peripheral(void);

      static task_type &get_base_task(void);
      static task_type &get_wb_task(void);

      static bool get_errors();
  
      struct config {

      };

      protected:
        task_type &baseTask;
    };

    class task_type {

      template<typename T1, typename T2>
      bool set_locations(const T1&, const T2&);
      void *get_source(void);
      void *get_destination(void);

      bool set_length(const int&);
      int get_length(void);

      bool set_link(task_type&);
      task_type *get_link(void);

      private:
        task_type *link;
        DmacDescriptor desc;
    };

}

