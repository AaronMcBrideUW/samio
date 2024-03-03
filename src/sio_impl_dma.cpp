
#include <sio_impl_dma.h>

namespace sio::impl::dma {

  using namespace sio::impl::util;

  static __attribute__ ((section(".hsram"))) __aligned(16) DmacDescriptor 
    wbDescArray[DMAC_CH_NUM] = {};
  static __attribute__ ((section(".hsram"))) __aligned(16) DmacDescriptor 
    baseDescArray[DMAC_CH_NUM] = {};
  static Task *baseTaskArray[DMAC_CH_NUM] = { nullptr };

  /// SECTION: INTERNAL
  /// #############################################################################################



    static struct TaskImpl_ {

    //   std::pair<int, Task*> find_task(const DmacDescriptor &targDesc) {
    //     Task *currentTask = nullptr;
    //     for (int i = 0; i < sizeof(baseTaskArray) / sizeof(baseTaskArray[0]); i++) {
    //       currentTask = baseTaskArray[i];

    //       if (currentTask) {
    //         if (currentTask->descPtr == &targDesc) {
    //           return std::make_pair(i, currentTask);
    //         }
    //         Task *chTask = currentTask;
    //         while(chTask->linkedTask && chTask->linkedTask != currentTask) {
    //           if (chTask->descPtr == &targDesc) {
    //             return std::make_pair(i, currentTask);
    //           }
    //         }
    //       }
    //       currentTask = currentTask->linkedTask;
    //     }
    //     return std::make_pair(-1, nullptr);
    //   }
    // }TaskImpl;

    }TaskImpl;


  /// SECTION: CHANNEL
  /// #############################################################################################


  Channel::Channel() {

  }



  /// SECTION: TASK
  /// #############################################################################################

  /// @brief Constructors





}




