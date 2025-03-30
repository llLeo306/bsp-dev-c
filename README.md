# bsp-dev-c

BSP Package for development board C.

## Usage

```bash
git clone https://github.com/bsp-dev-c/bsp-dev-c.git
cd bsp-dev-c
git submodule update --init --recursive
pip install libxr xrobot
xr_cubemx_cfg -d ./ -c --xrobot
xrobot_init_mod --config https://raw.githubusercontent.com/QDU-Robomaster/dev-c-robots/refs/heads/main/test.yaml --dir .\Modules\ && xrobot_setup
cmake . -DCMAKE_TOOLCHAIN_FILE:STRING=cmake/gcc-arm-none-eabi.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -Bbuild -G Ninja
cmake --build build
ls build/
```
