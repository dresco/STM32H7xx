/*
  my_machine.h - configuration for STM32H7xx processors

  Part of grblHAL

  Copyright (c) 2021-2022 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
//#define BOARD_PROTONEER_3XX   // For use with a Nucleo-F756ZG board.
//#define BOARD_GENERIC_UNO     // For use with a Nucleo-F756ZG board.
//#define BOARD_BTT_SKR_30      // BTT SKR V3 board.
//#define BOARD_WEACT_MINI_H743 // WeAct MiniSTM32H743 board.
//#define BOARD_REFERENCE       // grblHAL reference board map.
//#define BOARD_MY_MACHINE      // Add my_machine_map.h before enabling this!

#if defined(NUCLEO_H743) || defined(NUCLEO_H723)
#define IS_NUCLEO_DEVKIT 1
#else
#define IS_NUCLEO_DEVKIT 0
#endif

// Configuration
// Uncomment to enable.

#if !IS_NUCLEO_DEVKIT && !defined(USB_SERIAL_CDC)   // The Nucleo-F756ZG board has an off-chip UART to USB interface.
//#define USB_SERIAL_CDC       1 // Serial communication via native USB.
#endif
//#define SAFETY_DOOR_ENABLE   1 // Enable safety door input.
//#define VFD_ENABLE           1 // Set to 1 or 2 for Huanyang VFD spindle. More here https://github.com/grblHAL/Plugins_spindle
//#define MODBUS_ENABLE        1 // Set to 1 for auto direction, 2 for direction signal on auxillary output pin.
//#define WEBUI_ENABLE         3 // Enable ESP3D-WEBUI plugin along with networking and SD card plugins.
//#define WEBUI_AUTH_ENABLE    1 // Enable ESP3D-WEBUI authentication.
//#define WEBUI_INFLASH        1 // Store WebUI files in flash instead of on SD card.
//#define ETHERNET_ENABLE      1 // Ethernet streaming. Uses networking plugin.
//#define BLUETOOTH_ENABLE     1 // Set to 1 for HC-05 module. Uses Bluetooth plugin.
//#define SDCARD_ENABLE        1 // Run gcode programs from SD card, uses sdcard plugin.
//#define KEYPAD_ENABLE        1 // I2C keypad for jogging etc., uses keypad plugin.
//#define ODOMETER_ENABLE      1 // Odometer plugin.
//#define PPI_ENABLE           1 // Laser PPI plugin. To be completed.
//#define LASER_COOLANT_ENABLE 1 // Laser coolant plugin. To be completed.
//#define LB_CLUSTERS_ENABLE   1 // LaserBurn cluster support.
//#define OPENPNP_ENABLE       1 // OpenPNP plugin. To be completed.
//#define FANS_ENABLE          1 // Enable fan control via M106/M107. Enables fans plugin.
//#define PLASMA_ENABLE        1 // Plasma (THC) plugin. To be completed.
//#define TRINAMIC_ENABLE   2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE   2209 // Trinamic TMC2209 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE   5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C         1 // Trinamic I2C - SPI bridge interface.
//#define TRINAMIC_DEV         1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
//#define EEPROM_ENABLE        2 // I2C EEPROM support. Set to 1 for 24LC16 (2K), 3 for 24C32 (4K - 32 byte page) and 2 for other sizes. Uses eeprom plugin.
//#define EEPROM_IS_FRAM       1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
/**/

// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED            1
//#define X_AUTO_SQUARE       1
//#define Y_GANGED            1
//#define Y_AUTO_SQUARE       1
//#define Z_GANGED            1
//#define Z_AUTO_SQUARE       1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX    1
//#define Y_GANGED_LIM_MAX    1
//#define Z_GANGED_LIM_MAX    1

#if ETHERNET_ENABLE > 0
//#define TELNET_ENABLE       1 // Telnet daemon - requires Ethernet streaming enabled.
//#define WEBSOCKET_ENABLE    1 // Websocket daemon - requires Ethernet streaming enabled.
//#define MDNS_ENABLE         1 // mDNS daemon.
//#define SSDP_ENABLE         1 // SSDP daemon - requires HTTP enabled.
//#define MQTT_ENABLE         1 // MQTT client API, only enable if needed by plugin code.
#ifdef SDCARD_ENABLE
//#define FTP_ENABLE          1 // Ftp daemon - requires SD card enabled.
//#define HTTP_ENABLE         1 // http daemon - requires SD card enabled.
//#define WEBDAV_ENABLE       1 // webdav protocol - requires http daemon and SD card enabled.
#endif

// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_HOSTNAME        "grblHAL"
//#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_IP              "192.168.5.1"
//#define NETWORK_GATEWAY         "192.168.5.1"
//#define NETWORK_MASK            "255.255.255.0"
//#define NETWORK_FTP_PORT        21
//#define NETWORK_TELNET_PORT     23
//#define NETWORK_HTTP_PORT       80
#if HTTP_ENABLE
//#define NETWORK_WEBSOCKET_PORT  81
#else
//#define NETWORK_WEBSOCKET_PORT  80
#endif
#endif
