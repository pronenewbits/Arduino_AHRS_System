# Arduino_AHRS_System_With_EKF_UKF
This is a compact realtime embedded Attitude and Heading Reference System (AHRS) using Recursive Least Squares (RLS) for magnetometer calibration, and EKF/UKF for sensor fusion for Arduino platform.

- It's not using Eigen (small source code - more simple to understand).
- It's not using C++ Standard Library/std (for embedded consideration).
- If you set `SYSTEM_IMPLEMENTATION` to `SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT` in `konfig.h`, the code is platform agnostic (not using any library beside these C header files: `stdlib.h`, `stdint.h`, and `math.h`).
- There's no malloc/new/free dynamic memory allocation (for real time application). But it use heavy stack local variables, so you need to run it through memory analyzer if you are really concerned about implement this in mission critical hard real time application.

This code is the application of Extended Kalman Filter and Unscented Kalman Filter library I've made.

- The EKF library and documentation can be found in [this repository](https://github.com/pronenewbits/Embedded_EKF_Library).
- The UKF library and documentation can be found in [this repository](https://github.com/pronenewbits/Embedded_UKF_Library).


