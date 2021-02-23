/*
 * net.c
 *
 *  Created on: Feb 23, 2021
 *      Author: keyva
 */

#include "boot.h"

/**
 *
 */
void NetInit(void)
{
}

/**
 *
 */
void NetApp(void)
{
}

extern struct netif gnetif;

/**
 *
 */
void NetTransmitPacket(blt_int8u *data, blt_int8u len)
{
//	struct pbuf buf;
//	buf.payload = data;
//	buf.len = len;
//	gnetif.output(&gnetif, &buf, gnetif. const struct eth_addr * src, const struct eth_addr * dst,
//            u16_t eth_type);
}

/**
 */
blt_bool NetReceivePacket(blt_int8u *data, blt_int8u *len)
{
//	struct pbuf *p;
//	struct netif *n = &gnetif;
//	n->input(p, n);

//	data = buf.payload;
//	*len = buf.len;
}

