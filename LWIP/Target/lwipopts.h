/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Target/lwipopts.h
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "main.h"

/*-----------------------------------------------------------------------------*/
/* Current version of LwIP supported by CubeMx: 2.1.2 -*/
/*-----------------------------------------------------------------------------*/

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32CubeMX Specific Parameters (not defined in opt.h) ---------------------*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- WITH_RTOS disabled (Since FREERTOS is not set) -----*/
#define WITH_RTOS 0
/*----- CHECKSUM_BY_HARDWARE enabled -----*/
#define CHECKSUM_BY_HARDWARE 1
/*-----------------------------------------------------------------------------*/

/* LwIP Stack Parameters (modified compared to initialization value in opt.h) -*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- Value in opt.h for LWIP_DHCP: 0 -----*/
#define LWIP_DHCP 1
/*----- Default value in ETH configuration GUI in CubeMx: 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536
/*----- Default Value for LWIP_IGMP: 0 ---*/
#define LWIP_IGMP 1
/*----- Value in opt.h for NO_SYS: 0 -----*/
#define NO_SYS 1
/*----- Value in opt.h for SYS_LIGHTWEIGHT_PROT: 1 -----*/
#define SYS_LIGHTWEIGHT_PROT 0

/*----- Value in opt.h for MEM_ALIGNMENT: 1 -----*/
/*
 * todo: benchmark.
 * Intent is to avoid unnecessary cache maintenance at buffer boundaries (in fatfs code
 * using buffers passed from lwip), but is this even noticeable?
 */
#if L1_CACHE_ENABLE
#define MEM_ALIGNMENT 32
#else
#define MEM_ALIGNMENT 4
#endif

/*----- Default Value for MEM_SIZE: 1600 ---*/
#define MEM_SIZE (28*1024)

/*----- Default Value for H7 devices: 0x30044000 -----*/
#define LWIP_RAM_HEAP_POINTER 0x24004000

/*----- Value supported for H7 devices: 1 -----*/
#define LWIP_SUPPORT_CUSTOM_PBUF 1

/*----- Value in opt.h for LWIP_ETHERNET: LWIP_ARP || PPPOE_SUPPORT -*/
#define LWIP_ETHERNET 1
/*----- Value in opt.h for LWIP_DNS_SECURE: (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING | LWIP_DNS_SECURE_RAND_SRC_PORT) -*/
#define LWIP_DNS_SECURE 7
/*----- Value in opt.h for TCP_SND_QUEUELEN: (4*TCP_SND_BUF + (TCP_MSS - 1))/TCP_MSS -----*/
#define TCP_SND_QUEUELEN 9
/*----- Value in opt.h for TCP_SNDLOWAT: LWIP_MIN(LWIP_MAX(((TCP_SND_BUF)/2), (2 * TCP_MSS) + 1), (TCP_SND_BUF) - 1) -*/
#define TCP_SNDLOWAT 1071
/*----- Value in opt.h for TCP_SNDQUEUELOWAT: LWIP_MAX(TCP_SND_QUEUELEN)/2, 5) -*/
#define TCP_SNDQUEUELOWAT 5
/*----- Value in opt.h for TCP_WND_UPDATE_THRESHOLD: LWIP_MIN(TCP_WND/4, TCP_MSS*4) -----*/
#define TCP_WND_UPDATE_THRESHOLD 536
/*----- Default Value for LWIP_NETIF_STATUS_CALLBACK: 0 ---*/
#define LWIP_NETIF_STATUS_CALLBACK 1
/*----- Value in opt.h for LWIP_NETIF_LINK_CALLBACK: 0 -----*/
#define LWIP_NETIF_LINK_CALLBACK 1
/*----- Value in opt.h for LWIP_NETCONN: 1 -----*/
/*----- Default Value for LWIP_NUM_NETIF_CLIENT_DATA: 0 ---*/
#define LWIP_NUM_NETIF_CLIENT_DATA 2
#define LWIP_NETCONN 0
/*----- Value in opt.h for LWIP_SOCKET: 1 -----*/
#define LWIP_SOCKET 0
/*----- Value in opt.h for RECV_BUFSIZE_DEFAULT: INT_MAX -----*/
#define RECV_BUFSIZE_DEFAULT 2000000000
/*----- Default Value for LWIP_MDNS: 0 ---*/
#define LWIP_MDNS 1
/*----- Default Value for LWIP_MDNS_RESPONDER: 0 ---*/
#define LWIP_MDNS_RESPONDER 1
/*----- Default Value for MDNS_MAX_SERVICES: 0 ---*/
#define MDNS_MAX_SERVICES 8
/*----- Value in opt.h for LWIP_STATS: 1 -----*/
#define LWIP_STATS 0
/*----- Value in opt.h for CHECKSUM_GEN_IP: 1 -----*/
#define CHECKSUM_GEN_IP 0
/*----- Value in opt.h for CHECKSUM_GEN_UDP: 1 -----*/
#define CHECKSUM_GEN_UDP 0
/*----- Value in opt.h for CHECKSUM_GEN_TCP: 1 -----*/
#define CHECKSUM_GEN_TCP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP: 1 -----*/
#define CHECKSUM_GEN_ICMP 0
/*----- Value in opt.h for CHECKSUM_GEN_ICMP6: 1 -----*/
#define CHECKSUM_GEN_ICMP6 0
/*----- Value in opt.h for CHECKSUM_CHECK_IP: 1 -----*/
#define CHECKSUM_CHECK_IP 0
/*----- Value in opt.h for CHECKSUM_CHECK_UDP: 1 -----*/
#define CHECKSUM_CHECK_UDP 0
/*----- Value in opt.h for CHECKSUM_CHECK_TCP: 1 -----*/
#define CHECKSUM_CHECK_TCP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP: 1 -----*/
#define CHECKSUM_CHECK_ICMP 0
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP6: 1 -----*/
#define CHECKSUM_CHECK_ICMP6 0
/*-----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/*
   ----------------------------------------
   ---------- TCP tuning options ----------
   ----------------------------------------
*/
#define TCP_SND_BUF             (4*TCP_MSS)


/*
   ---------------------------------------
   ------------ HTTPD options ------------
   ---------------------------------------
*/
 #define LWIP_HTTPD_CUSTOM_FILES         0
 #define LWIP_HTTPD_DYNAMIC_HEADERS      1
 #define LWIP_HTTPD_DYNAMIC_FILE_READ    1
 #define LWIP_HTTPD_SUPPORT_V09          0
 #define LWIP_HTTPD_SUPPORT_11_KEEPALIVE 1
 #define LWIP_HTTPD_SUPPORT_POST         1
 #define HTTPD_LIMIT_SENDING_TO_2MSS     0

/*
   ---------------------------------------
   ------------ Debug options ------------
   ---------------------------------------
*/

 //#define LWIP_DEBUG                      1

 #define ETHARP_DEBUG                    LWIP_DBG_ON
 #define IP_DEBUG                        LWIP_DBG_ON
 #define TCP_DEBUG                       LWIP_DBG_ON
 #define TCP_OUTPUT_DEBUG                LWIP_DBG_ON
 #define HTTPD_DEBUG                     LWIP_DBG_ON
 #define TCP_QLEN_DEBUG                  LWIP_DBG_ON
 #define LWIP_DBG_TYPES_ON               (LWIP_DBG_ON | LWIP_DBG_TRACE)

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__LWIPOPTS__H__ */
