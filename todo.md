
# TO-DO:

  ### CORE-DMA:
  - Complete testing of ALL functionality fixing any errors along the way.         
  - Check implementation of writeback cases in descriptor list functions to 
    ensure that they are correct/ideal.
  - Check implementation of CriticalSection to ensure that writeback case is 
    correct.
  - Go through functions and ensure that all update_valid() calls are necessary.
  - Go through functions and ensure that use of critical section is reserved
    for places when it is needed only -> prefer invalidating/revalidating 
    descriptor.
  - Add logic in writeback to handle suspend interrupt.
  - Add logic in writeback to handle hit on invalid descriptor.
  - Convert as much code as possible to compile time logic by changing all 
    eligable functions to templates and adding constexpr variables/consteval 
    lambdas in them.
  - Ensure that all default config parameters are checked for validity and
    properly mapped to corresponding values.
  - Edit names of config parameters to be consistent with either spec or
    unique naming convension (choose 1 only).
  - Go through and change types of variables to preserve space requirements.

  - Change names of enums to be shorter (recall they are in namespace)

