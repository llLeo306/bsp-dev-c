# bsp-dev-c

BSP Package for development board C.

## Usage

### Windows Ninja

Windows上使用 Ninja会出现无法增量编译的问题，暂时没有解决方法

```bash
git clone https://github.com/QDU-Robomaster/bsp-dev-c
cd bsp-dev-c
git submodule update --init --recursive
pip install libxr xrobot
xr_cubemx_cfg -d ./ -c --xrobot
xrobot_init_mod --config https://raw.githubusercontent.com/QDU-Robomaster/dev-c-robots/refs/heads/main/test.yaml --dir .\Modules\
xrobot_setup
cmake . -DCMAKE_TOOLCHAIN_FILE:STRING=cmake/gcc-arm-none-eabi.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Bbuild -G Ninja
cmake --build build
ls build/
```

### Windows Makefile

Windows上使用 GNU Makefile 只能单线程编译，可以换用MinGW Makefiles，但是不兼容linux

```bash
git clone https://github.com/QDU-Robomaster/bsp-dev-c
cd bsp-dev-c
git submodule update --init --recursive
pip install libxr xrobot
xr_cubemx_cfg -d ./ -c --xrobot
xrobot_init_mod --config https://raw.githubusercontent.com/QDU-Robomaster/dev-c-robots/refs/heads/main/test.yaml --dir .\Modules\
xrobot_setup
cmake . -DCMAKE_TOOLCHAIN_FILE:STRING=cmake/gcc-arm-none-eabi.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Bbuild -G "Unix Makefiles"
cmake --build build
ls build/
```

### Linux

linux无任何问题

```bash
git clone https://github.com/QDU-Robomaster/bsp-dev-c
cd bsp-dev-c
git submodule update --init --recursive
pip install libxr xrobot
xr_cubemx_cfg -d ./ -c --xrobot
xrobot_init_mod --config https://raw.githubusercontent.com/QDU-Robomaster/dev-c-robots/refs/heads/main/test.yaml --dir ./Modules
xrobot_setup
cmake . -DCMAKE_TOOLCHAIN_FILE:STRING=cmake/gcc-arm-none-eabi.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Bbuild -G Ninja
cmake --build build
ls build/
```
