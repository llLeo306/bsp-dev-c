# bsp-dev-c

BSP Package for development board C.

## Usage

```bash
git clone https://github.com/bsp-dev-c/bsp-dev-c.git
cd bsp-dev-c
git submodule update --init --recursive
cmake . -DDEV_C_TEST_BUILD=True -DCMAKE_TOOLCHAIN_FILE:STRING=cmake/gcc-arm-none-eabi.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Bbuild -G Ninja
cmake --build build
ls build/
```
