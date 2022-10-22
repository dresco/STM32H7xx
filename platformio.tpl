#
# Template for Web Builder, STM32H7xx
#

[platformio]
include_dir = Inc
src_dir = Src

[common]
build_flags =
  -I .
  -D L1_CACHE_ENABLE=1
  -D OVERRIDE_MY_MACHINE
  -I FATFS/Target
  -I FATFS/App
  -I Middlewares/Third_Party/FatFs/src
  -I Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
  -I Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  -I USB_DEVICE/Target
  -I USB_DEVICE/App
lib_deps =
  bluetooth
  grbl
  eeprom
  fans
  keypad
  laser
  motors
  trinamic
  odometer
  openpnp
  sdcard
  spindle
  webui
  FATFS/App
  FATFS/Target
  Middlewares/Third_Party/FatFs
  USB_DEVICE/App
  USB_DEVICE/Target
  Middlewares/ST/STM32_USB_Device_Library/Class
  Middlewares/ST/STM32_USB_Device_Library/Core
lib_extra_dirs =
  .

[eth_networking]
build_flags =
  -I LWIP/App
  -I LWIP/Target
  -I Middlewares/Third_Party/LwIP/src/include
  -I Middlewares/Third_Party/LwIP/system
  -I Middlewares/Third_Party/LwIP/src/include/netif
  -I Middlewares/Third_Party/LwIP/src/include/lwip
  -I Drivers/BSP/Components/lan8742
  # Floating point support for printf, required for WebUI v3
  -Wl,-u,_printf_float
lib_deps =
   networking
   webui
   Middlewares/Third_Party/LwIP
   Drivers/BSP/Components/lan8742
lib_extra_dirs =

# Note: The platformio package registry does not yet include framework-stm32cubeh7@v1.10, which introduced
#       the rewritten ST ethernet driver. We are therefore using a custom location for the framework, cloned
#       from the official ST repo at https://github.com/STMicroelectronics/STM32CubeH7, with only minimal
#       changes to remove unneeded content & add a package.json file.
[env]
platform = ststm32
platform_packages = framework-stm32cubeh7 @ https://github.com/dresco/STM32CubeH7.git
framework = stm32cube
# Do not produce .a files for lib deps (which would prevent them from overriding weak symbols)
lib_archive = no
lib_ldf_mode = off

[env:%env_name%]
board = %board%
board_build.ldscript = %ldscript%
build_flags = ${common.build_flags}
%build_flags%
lib_deps = ${common.lib_deps}
%lib_deps%
lib_extra_dirs = ${common.lib_extra_dirs}
