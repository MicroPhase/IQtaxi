#pragma once

/*
 * Minimal ExtIO API surface used by HDSDR/Winrad.
 * Spec: http://www.sdradio.eu/weaksignals/code/Winrad_Extio.pdf
 * Extended enums follow common ExtIO headers (e.g. ExtIO_LimeSDR).
 */

#ifdef _WIN32
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  define EXTIO_CALL __stdcall
#else
#  define EXTIO_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HDSDR/Winrad provide the IQ callback as cdecl (NOT stdcall).
 * Official LC_ExtIO_Types.h:
 *   typedef int (* pfnExtIOCallback)(int, int, float, void*);
 * Calling it as __stdcall corrupts the stack every IQ block
 * (-> 0xc00000fd / 0xc0000409). DLL exports remain __stdcall.
 */
typedef int (*pfnExtIOCallback)(int cnt, int status, float IQoffs, void *IQdata);

enum extHWtypeT {
    exthwNone = 0,
    exthwSDR14 = 1,
    exthwSDRX = 2,
    exthwUSBdata16 = 3,   /* int16 interleaved I/Q via callback */
    exthwSCdata = 4,
    exthwUSBdata24 = 5,
    exthwUSBdata32 = 6,
    exthwUSBfloat32 = 7,
    exthwHPSDR = 8,
    exthwUSBdataU8 = 9,
    exthwUSBdataS8 = 10,
    exthwFullPCM32 = 11
};

enum extHWstatusT {
    extHw_Disconnected = 0,
    extHw_READY = 1,
    extHw_RUNNING = 2,
    extHw_ERROR = 3,
    extHw_OVERLOAD = 4,
    extHw_Changed_SampleRate = 100,
    extHw_Changed_LO = 101,
    extHw_Lock_LO = 102,
    extHw_Unlock_LO = 103,
    extHw_Changed_LO_NotRecalcIF = 104,
    extHw_Changed_TUNE = 105,
    extHw_Changed_MODE = 106,
    extHw_Start = 107,
    extHw_Stop = 108,
    extHw_Changed_FILTER = 109,
    extHw_Changed_ATTENUATOR = 125,
    extHw_Changed_RF_IF = 136
};

#ifdef __cplusplus
}
#endif
