// This file loads custom code from the Custom/ subdirectory if
// CUSTOM_CODE_FILENAME is defined.

#include "Grbl.h"

#define CUSTOM_CODE_FILENAME    "../Custom/clock_engine.cpp"

#ifdef CUSTOM_CODE_FILENAME
    #include CUSTOM_CODE_FILENAME
#endif

#ifdef DISPLAY_CODE_FILENAME
#    include DISPLAY_CODE_FILENAME
#endif