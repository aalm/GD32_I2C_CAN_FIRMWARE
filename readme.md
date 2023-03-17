Forked from https://github.com/Longan-Labs/GD32_I2C_CAN_FIRMWARE/commit/c2d772884fa3bd57015d34f57689de8dedbea7ec


# GD32 I2C CAN FIRMWARE

This code uses GD32E103/GD32C103 to realize the function of I2C to CAN Bus.

See Docs/readme.txt for the working description.

Since the GD32E103 and GD32C103 are pin to pin compatible, these codes will work for both chips.

## Development environment and tools

This project is compiled with GCC 8.3.1 arm-none-eabi.

Simulation tools can use JLink or something SWD-compatible.

## Main differences from Longan implementation

* I2C speed increased.
* CAN communication is disabled until config is received from I2C.
* Implemented all 28 CAN filters.
* Implemented sleep modes for CAN.
* Latest compiler and GD32 base firmware.
* CAN-FD support dropped. Unfortunately I can't test this and removed it.

## Respository Contents

* minimal project configured using cmake


## License

```
MIT License

Copyright (c) 2018 @ Longan Labs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
