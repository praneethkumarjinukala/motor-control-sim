/* modbus_tcp.c - Minimal Modbus TCP server over lwIP raw API
 *
 * Exposes one holding register:
 *   40001  (address 0x0000)  = motor omega * 10  [integer, 0..3000]
 *
 * Modbus function codes handled:
 *   0x03  Read Holding Registers
 *   0x06  Write Single Register  (write to 40001 sets a speed override)
 *
 * Wire:  STM32F407 Ethernet MAC -> Renode EthernetSwitch -> host TAP
 *        IP: 192.168.0.10/24 (static, configured in lwipopts.h)
 *        Port: 502
 *
 * Test from host:
 *   python3 -c "
 *   import socket, struct
 *   s = socket.create_connection(('192.168.0.10', 502))
 *   # MBAP + FC03: read 1 register from address 0
 *   req = struct.pack('>HHHBBHH', 1,0,6,1,3,0,1)
 *   s.sendall(req)
 *   print(struct.unpack('>HHHBBBH', s.recv(11)))
 *   "
 */

#include "modbus_tcp.h"
#include "lwip/tcp.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include <string.h>
#include <stdint.h>

#define MODBUS_PORT   502
#define REG_OMEGA     0x0000   /* holding register 40001 */
#define REG_SETPOINT  0x0001   /* holding register 40002 (read-only) */
#define NUM_REGS      2

/* ---- register table ---- */
static volatile uint16_t g_regs[NUM_REGS] = {0};

void modbus_set_omega(uint16_t val)    { g_regs[REG_OMEGA]    = val; }
void modbus_set_setpoint(uint16_t val) { g_regs[REG_SETPOINT] = val; }

/* ---- Modbus frame parser ---- */
static uint16_t process_request(const uint8_t *in, uint16_t in_len,
                                 uint8_t *out) {
    if (in_len < 8) return 0;

    uint16_t tid   = (uint16_t)(in[0]<<8 | in[1]);
    /* protocol id [2:3] = 0 for Modbus */
    /* length      [4:5] */
    uint8_t  unit  = in[6];
    uint8_t  fc    = in[7];
    uint16_t addr  = (uint16_t)(in[8]<<8 | in[9]);
    uint16_t count = (uint16_t)(in[10]<<8| in[11]);

    (void)unit;

    if (fc == 0x03) {                        /* Read Holding Registers */
        if (addr + count > NUM_REGS) count = NUM_REGS - addr;
        uint16_t byte_cnt = count * 2;
        /* MBAP header */
        out[0] = tid >> 8; out[1] = tid & 0xFF;
        out[2] = 0; out[3] = 0;
        uint16_t plen = 3 + byte_cnt;
        out[4] = plen >> 8; out[5] = plen & 0xFF;
        out[6] = 1;   /* unit */
        out[7] = 0x03;
        out[8] = (uint8_t)byte_cnt;
        for (uint16_t i = 0; i < count; ++i) {
            uint16_t v = g_regs[addr + i];
            out[9 + i*2]   = v >> 8;
            out[10 + i*2]  = v & 0xFF;
        }
        return (uint16_t)(9 + byte_cnt);

    } else if (fc == 0x06) {                 /* Write Single Register */
        uint16_t val = (uint16_t)(in[10]<<8 | in[11]);
        if (addr < NUM_REGS) g_regs[addr] = val;
        /* Echo back */
        memcpy(out, in, 12);
        return 12;
    }
    return 0;   /* unsupported FC - silent drop */
}

/* ---- lwIP raw TCP callbacks ---- */
static err_t modbus_recv(void *arg, struct tcp_pcb *pcb,
                          struct pbuf *p, err_t err) {
    (void)arg;
    if (!p || err != ERR_OK) { tcp_close(pcb); return ERR_OK; }

    static uint8_t resp[256];
    uint8_t *data = (uint8_t *)p->payload;
    uint16_t rlen = process_request(data, (uint16_t)p->len, resp);
    if (rlen) tcp_write(pcb, resp, rlen, TCP_WRITE_FLAG_COPY);

    pbuf_free(p);
    tcp_output(pcb);
    return ERR_OK;
}

static err_t modbus_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg; (void)err;
    tcp_recv(newpcb, modbus_recv);
    return ERR_OK;
}

/* ---- lwIP netif (STM32F407 Ethernet MAC via Renode) ---- */
static struct netif g_netif;
extern err_t ethernetif_init(struct netif *netif);  /* from lwIP port */

void modbus_tcp_init(void) {
    lwip_init();

    ip4_addr_t ip, nm, gw;
    IP4_ADDR(&ip, 192,168,0,10);
    IP4_ADDR(&nm, 255,255,255,0);
    IP4_ADDR(&gw, 192,168,0,1);

    netif_add(&g_netif, &ip, &nm, &gw, NULL, ethernetif_init, ethernet_input);
    netif_set_default(&g_netif);
    netif_set_up(&g_netif);

    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, MODBUS_PORT);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, modbus_accept);
}

void ethernetif_poll(void) {
    /* Called from main() idle loop - drives lwIP timers + Ethernet DMA */
    extern void ethernetif_input(struct netif *netif);
    ethernetif_input(&g_netif);
    sys_check_timeouts();
}
