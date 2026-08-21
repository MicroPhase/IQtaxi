# Vendored AD9361 no-OS driver

This directory contains the AD9361 and common no-OS sources required by the
M300 host-side RF controller. The snapshot was copied verbatim from:

```text
board/e200/init_ad9361_e200/lib/ad9361
board/e200/init_ad9361_e200/lib/no_os
```

Keeping the snapshot here makes `host_app/e200/MICROPHASE_IQ_TAXI`
self-contained. In particular, its CMake build must not depend on the board
firmware source tree or on the repository's surrounding directory layout.

The original Analog Devices copyright and redistribution terms are retained
in the individual source and header files.
