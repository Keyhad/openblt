/************************************************************************************//**
* \file         Source/net.c
* \brief        Bootloader TCP/IP network communication interface source file.
* \ingroup      Core
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2014  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#if (BOOT_COM_NET_ENABLE > 0)
  #include "lwip/err.h"
  #include "lwip/def.h"
  #include "lwip/altcp.h"
  #include "lwip/altcp_tcp.h"
#endif

#include "net.h"
#include <string.h>


#if (BOOT_COM_NET_ENABLE > 0)
/****************************************************************************************
* Configuration macros
****************************************************************************************/
/* Extend the default time that the backdoor is open if firmware updates via TCP/IP
 * are supported. in this case an activation of the bootloader results in an
 * initialization of the ethernet MAC. when connected to the network via a router this
 * can take several seconds. Feel free to shorten/lengthen this time for finetuning,
 * Note that adding this configuration macro to blt_conf.h overrides the value here.
 */
#ifndef BOOT_COM_NET_BACKDOOR_EXTENSION_MS
#define BOOT_COM_NET_BACKDOOR_EXTENSION_MS   (10000)
#endif


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Delta time for the uIP periodic timer. */
#define NET_UIP_PERIODIC_TIMER_MS   (500)
/** \brief Delta time for the uIP ARP timer. */
#define NET_UIP_ARP_TIMER_MS        (10000)
/** \brief Macro for accessing the Ethernet header information in the buffer */
#define NET_UIP_HEADER_BUF          ((struct uip_eth_hdr *)&uip_buf[0])


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void NetServerTask(void);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds the time out value of the uIP periodic timer. */
static blt_int32u periodicTimerTimeOut;
/** \brief Holds the time out value of the uIP ARP timer. */
static blt_int32u ARPTimerTimeOut;
#if (BOOT_COM_NET_DHCP_ENABLE > 0)
/** \brief Holds the MAC address which is used by the DHCP client. */
static struct uip_eth_addr macAddress;
#endif
/** \brief Boolean flag to determine if the module was initialized or not. */
static blt_bool netInitializedFlag = BLT_FALSE;
#if (BOOT_COM_NET_DEFERRED_INIT_ENABLE == 1)
/** \brief Boolean flag initialized such that the normal initialization via NetInit()
 *         should be deferred. A called to NetDeferredInit() is need to do the actual
 *         initialization of this module.
 */
static blt_bool netInitializationDeferred = BLT_TRUE;
#else
/** \brief Boolean flag initialized such that the normal initialization via NetInit()
 *         proceeds as usual.
 */
static blt_bool netInitializationDeferred = BLT_FALSE;
#endif

#define NET_ALLOC_HTTP_STATE() (struct net_state *)mem_malloc(sizeof(struct net_state))
#define NET_FREE_NET_STATE(x) mem_free(x)
#define NET_POLL_INTERVAL	4 // x500ms

/* Return values for http_send_*() */
#define NET_DATA_TO_SEND_FREED    3
#define NET_DATA_TO_SEND_BREAK    2
#define NET_DATA_TO_SEND_CONTINUE 1
#define NET_NO_DATA_TO_SEND       0


static err_t net_accept(void *arg, struct altcp_pcb *tpcb, err_t err);
static err_t net_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err);
static void net_err(void *arg, err_t err);
static err_t net_poll(void *arg, struct altcp_pcb *pcb);
static err_t net_sent(void *arg, struct altcp_pcb *pcb, u16_t len);
static err_t net_close_or_abort_conn(struct altcp_pcb *pcb, struct net_state *ns, u8_t abort_conn);
static err_t net_write(struct altcp_pcb *pcb, const void *ptr, u16_t *length, u8_t apiflags);


/** Free a struct net_state.
 * Also frees the file data if dynamic.
 */
static void
net_state_free(struct net_state *ns)
{
  if (ns != NULL) {
    //net_state_eof(ns);
    //ns_remove_connection(ns);
    NET_FREE_NET_STATE(ns);
  }
}

/**
 * The connection shall be actively closed (using RST to close from fault states).
 * Reset the sent- and recv-callbacks.
 *
 * @param pcb the tcp pcb to reset callbacks
 * @param hs connection state to free
 */
static err_t
net_close_or_abort_conn(struct altcp_pcb *pcb, struct net_state *ns, u8_t abort_conn)
{
  err_t err;
  LWIP_DEBUGF(NET_DEBUG, ("Closing connection %p\n", (void *)pcb));

  altcp_arg(pcb, NULL);
  altcp_recv(pcb, NULL);
  altcp_err(pcb, NULL);
  altcp_poll(pcb, NULL, 0);
  altcp_sent(pcb, NULL);
  if (ns != NULL) {
    net_state_free(ns);
  }

  if (abort_conn) {
    altcp_abort(pcb);
    return ERR_OK;
  }
  err = altcp_close(pcb);
  if (err != ERR_OK) {
    LWIP_DEBUGF(NET_DEBUG, ("Error %d closing %p\n", err, (void *)pcb));
    /* error closing, try again later in poll */
    altcp_poll(pcb, net_poll, NET_POLL_INTERVAL);
  }
  return err;
}

/**
 * The connection shall be actively closed.
 * Reset the sent- and recv-callbacks.
 *
 * @param pcb the tcp pcb to reset callbacks
 * @param hs connection state to free
 */
static err_t
net_close_conn(struct altcp_pcb *pcb, struct net_state *ns)
{
  return net_close_or_abort_conn(pcb, ns, 0);
}

/** Initialize a struct http_state.
 */
static void
net_state_init(struct net_state *ns)
{
  /* Initialize the structure. */
  memset(ns, 0, sizeof(struct net_state));
  ns->dto_counter = 1;
}

/** Allocate a struct net_state. */
static struct net_state *
net_state_alloc(void)
{
  struct net_state *ret = NET_ALLOC_HTTP_STATE();
  if (ret != NULL) {
    net_state_init(ret);
    /*net_add_connection(ret);*/
  }
  return ret;
}

static err_t net_accept(void *arg, struct altcp_pcb *pcb, err_t err)
{
  struct net_state *ns;
  LWIP_UNUSED_ARG(err);
  LWIP_UNUSED_ARG(arg);
  LWIP_DEBUGF(NET_DEBUG, ("net_accept %p / %p\n", (void *)pcb, arg));

  if ((err != ERR_OK) || (pcb == NULL)) {
	return ERR_VAL;
  }

  /* Set priority */
  altcp_setprio(pcb, TCP_PRIO_MIN);

  /* Allocate memory for the structure that holds the state of the
	 connection - initialized by that function. */

  ns = net_state_alloc();
  if (ns == NULL) {
	LWIP_DEBUGF(NET_DEBUG, ("net_accept: Out of memory, RST\n"));
	return ERR_MEM;
  }
  ns->netpcb = pcb;

  /* Tell TCP that this is the structure we wish to be passed for our
	 callbacks. */
  altcp_arg(pcb, ns);

  /* Set up the various callback functions */
  altcp_recv(pcb, net_recv);
  altcp_err(pcb, net_err);
  altcp_poll(pcb, net_poll, NET_POLL_INTERVAL);
  altcp_sent(pcb, net_sent);

  return ERR_OK;
}

/** Call tcp_write() in a loop trying smaller and smaller length
 *
 * @param pcb altcp_pcb to send
 * @param ptr Data to send
 * @param length Length of data to send (in/out: on return, contains the
 *        amount of data sent)
 * @param apiflags directly passed to tcp_write
 * @return the return value of tcp_write
 */
static err_t
net_write(struct altcp_pcb *pcb, const void *ptr, u16_t *length, u8_t apiflags)
{
  u16_t len, max_len;
  err_t err;
  LWIP_ASSERT("length != NULL", length != NULL);
  len = *length;
  if (len == 0) {
    return ERR_OK;
  }
  /* We cannot send more data than space available in the send buffer. */
  max_len = altcp_sndbuf(pcb);
  if (max_len < len) {
    len = max_len;
  }

  do {
    LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE, ("Trying to send %d bytes\n", len));
    err = altcp_write(pcb, ptr, len, apiflags);
    if (err == ERR_MEM) {
      if ((altcp_sndbuf(pcb) == 0) ||
          (altcp_sndqueuelen(pcb) >= TCP_SND_QUEUELEN)) {
        /* no need to try smaller sizes */
        len = 1;
      } else {
        len /= 2;
      }
      LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE,
                  ("Send failed, trying less (%d bytes)\n", len));
    }
  } while ((err == ERR_MEM) && (len > 1));

  if (err == ERR_OK) {
    LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE, ("Sent %d bytes\n", len));
    *length = len;
  } else {
    LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE, ("Send failed with err %d (\"%s\")\n", err, lwip_strerr(err)));
    *length = 0;
  }

  return err;
}

/** Sub-function of net_send(): This is the normal send-routine for non-ssi files
 *
 * @returns: - 1: data has been written (so call tcp_ouput)
 *           - 0: no data has been written (no need to call tcp_output)
 */
static u8_t
net_send_data_nonssi(struct altcp_pcb *pcb, struct net_state *ns)
{
  err_t err;
  u16_t len;
  u8_t data_to_send = 0;

  /* We are not processing an SHTML file so no tag checking is necessary.
   * Just send the data as we received it from the file. */
  len = (u16_t)LWIP_MIN(ns->left, 0xffff);

  err = net_write(pcb, ns->dto_data, &len, 0);
  if (err == ERR_OK) {
    data_to_send = 1;
    ns->dto_len += len;
    ns->left -= len;
  }

  return data_to_send;
}

/**
 * Try to send more data on this pcb.
 *
 * @param pcb the pcb to send data
 * @param hs connection state
 */
static u8_t
net_send(struct altcp_pcb *pcb, struct net_state *ns)
{
  u8_t data_to_send = NET_NO_DATA_TO_SEND;

  LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE, ("net_send: pcb=%p ns=%p left=%d\n", (void *)pcb,
              (void *)ns, ns != NULL ? (int)ns->left : 0));

  /* If we were passed a NULL state structure pointer, ignore the call. */
  if (ns == NULL) {
    return 0;
  }

  data_to_send = net_send_data_nonssi(pcb, ns);

  if (ns->left == 0) {
    /* We reached the end of the file so this request is done.
     * This adds the FIN flag right into the last data segment. */
    LWIP_DEBUGF(NET_DEBUG, ("End of data.\n"));
    return 0;
  }
  LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE, ("send_data end.\n"));
  return data_to_send;
}


/**
 * Data has been received on this pcb.
 * this should normally only happen once (if the request fits in one packet).
 */
static err_t
net_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err)
{
  struct net_state *ns = (struct net_state *)arg;
  LWIP_DEBUGF(NET_DEBUG | LWIP_DBG_TRACE, ("net_recv: pcb=%p pbuf=%p err=%s\n", (void *)pcb,
              (void *)p, lwip_strerr(err)));

  if ((err != ERR_OK) || (p == NULL) || (ns == NULL)) {
    /* error or closed by other side? */
    if (p != NULL) {
      /* Inform TCP that we have taken the data. */
      altcp_recved(pcb, p->tot_len);
      pbuf_free(p);
    }
    if (ns == NULL) {
      /* this should not happen, only to be robust */
      LWIP_DEBUGF(NET_DEBUG, ("Error, net_recv: ns is NULL, close\n"));
    }
    net_close_conn(pcb, ns);
    return ERR_OK;
  }

  /* Inform TCP that we have taken the data. */
  altcp_recved(pcb, p->tot_len);

  /* only process the data if its length is not longer than expected. otherwise just
   * ignore it. XCP is request/response. this means that a new requests should
   * only be processed when the response the the previous request was sent. new
   * requests before the last response was sent can therefore also be ignored.
   */
  if (((p->tot_len - 4) <= BOOT_COM_NET_RX_MAX_DATA) &&
       (ns->dto_tx_pending == BLT_FALSE))
  {
    /* the first 4 bytes contain a counter value in which we are not really interested */
    XcpPacketReceived((blt_int8u*)p->payload + 4, (blt_int8u)(p->tot_len - 4));
  }

  pbuf_free(p);
  net_send(pcb, ns);

  return ERR_OK;
}

/**
 * The pcb had an error and is already deallocated.
 * The argument might still be valid (if != NULL).
 */
static void
net_err(void *arg, err_t err)
{
  struct net_state *hs = (struct net_state *)arg;
  LWIP_UNUSED_ARG(err);

  LWIP_DEBUGF(NET_DEBUG, ("net_err: %s", lwip_strerr(err)));

  if (hs != NULL) {
    net_state_free(hs);
  }
}

/**
 * Data has been sent and acknowledged by the remote host.
 * This means that more data can be sent.
 */
static err_t
net_sent(void *arg, struct altcp_pcb *pcb, u16_t len)
{
  struct net_state *ns = (struct net_state *)arg;

  if (ns == NULL) {
    return ERR_OK;
  }

  ns->retries = 0;
  ns->dto_tx_pending = 0;

  net_send(pcb, ns);

  return ERR_OK;
}


/************************************************************************************//**
** \brief     Initializes the TCP/IP network communication interface.
** \return    none.
**
****************************************************************************************/
void NetInit(void)
{
  struct altcp_pcb *netpcb = altcp_tcp_new_ip_type(IPADDR_TYPE_ANY);
  LWIP_ASSERT("NetInit: tcp_new failed", netpcb != NULL);
  if (netpcb) {
    altcp_setprio(netpcb, TCP_PRIO_MIN);
    /* set SOF_REUSEADDR here to explicitly bind httpd to multiple interfaces */
    err_t err = altcp_bind(netpcb, IP_ANY_TYPE, BOOT_COM_NET_PORT);
    LWIP_UNUSED_ARG(err); /* in case of LWIP_NOASSERT */
    LWIP_ASSERT("NetInit: tcp_bind failed", err == ERR_OK);
    netpcb = altcp_listen(netpcb);
    LWIP_ASSERT("NetInit: tcp_listen failed", netpcb != NULL);
    altcp_accept(netpcb, net_accept);
  }
} /*** end of NetInit ***/


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data Pointer to byte array with data that it to be transmitted.
** \param     len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
void NetTransmitPacket(blt_int8u *data, blt_int8u len)
{
} /*** end of NetTransmitPacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \param     len Pointer where the length of the packet is to be stored.
** \return    BLT_TRUE if a packet was received, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool NetReceivePacket(blt_int8u *data, blt_int8u *len)
{
  return BLT_FALSE;
} /*** end of NetReceivePacket ***/


/************************************************************************************//**
** \brief     The uIP network application that implements XCP on TCP/IP. Note that this
**            application make use of the fact that XCP is request/response based. So
**            no new request will come in when a response is pending for transmission,
**            if so, the transmission of the pending response is aborted.
** \return    none.
**
****************************************************************************************/
void NetApp()
{
} /*** end of NetApp ***/


/**
 * The poll function is called every 2nd second.
 * If there has been no data sent (which resets the retries) in 8 seconds, close.
 * If the last portion of a file has not been sent in 2 seconds, close.
 *
 * This could be increased, but we don't want to waste resources for bad connections.
 */
static err_t
net_poll(void *arg, struct altcp_pcb *pcb)
{
  /* get pointer to application state */
  struct net_state *ns = (struct net_state *)arg;

  /* check if there is a packet waiting to be transmitted. this is done via polling
   * because then it is possible to asynchronously send data. otherwise data is
   * only really send after a newly received packet was received.
   */
  if (ns->dto_tx_req == BLT_TRUE)
  {
    /* reset the transmit request flag. */
	ns->dto_tx_req = BLT_FALSE;
    if (ns->dto_len > 0)
    {
      /* set the transmit pending flag. */
      ns->dto_tx_pending = BLT_TRUE;
      /* submit the data for transmission. */
      net_send(pcb, ns);
      //tcp_write(pcb, ns->dto_data, ns->dto_len, 0);
    }
  }

  return ERR_OK;
}

/************************************************************************************//**
** \brief     Runs the TCP/IP server task.
** \return    none.
**
****************************************************************************************/
static void NetServerTask(void)
{
} /*** end of NetServerTask ***/

#endif /* BOOT_COM_NET_ENABLE > 0 */


/*********************************** end of net.c **************************************/
