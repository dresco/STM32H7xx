#ifndef __mx_lwip_H
#define __mx_lwip_H

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"

/* Global Variables ----------------------------------------------------------*/
extern ETH_HandleTypeDef EthHandle;

#endif /*__ mx_lwip_H */
