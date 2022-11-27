/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Erbium (Er) CoAP client example.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "contiki-net.h"
#include "contiki.h"
#include "er-coap-engine.h"
#include "pressure_sensor.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

/* FIXME: This server address is hard-coded for Cooja and link-local for unconnected border router. */
// 2001:0660:4403:04c2::b070
#define DEV_0(ipaddr) uip_ip6addr(ipaddr, 0x2001, 0x0660, 0x4403, 0x04c2, 0x0, 0x0, 0x0, 0xb070)
#define DEV_1(ipaddr) uip_ip6addr(ipaddr, 0x2001, 0x0660, 0x4403, 0x04c2, 0x0, 0x0, 0x0, 0x2450)
#define DEV_CR(ipaddr) uip_ip6addr(ipaddr, 0x2001, 0x0660, 0x4403, 0x04c2, 0x0, 0x0, 0x0, 0x2352)
/* #define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 0x1) */

#define LOCAL_PORT UIP_HTONS(COAP_DEFAULT_PORT + 1)
#define REMOTE_PORT UIP_HTONS(COAP_DEFAULT_PORT)

#define TOGGLE_INTERVAL 5
#define LOW_THRESHOLD 11
#define HIGH_THRESHOLD 250
#define PROCESSING_TIME 20

#define IP_DEV_0 0
#define IP_DEV_1 1
#define IP_DEV_CR 2

#define REQ_GET 0
#define REQ_PUMP_ON 1
#define REQ_PUMP_OFF 2
#define REQ_ALARM 3

uip_ipaddr_t server_ipaddr[3];
static struct etimer et;
linear_tank_t linear_tank;

/* Example URIs that can be queried. */
#define NUMBER_OF_URLS 3
/* leading and ending slashes only for demo purposes, get cropped automatically when setting the Uri-Path */
char *service_urls[NUMBER_OF_URLS] =
    {"/sensors/pressure", "/actuators/pump", "/alarm"};

enum processing_state_t {
    TRY_FILLING,
    FILLING,
    WAITING,
    EMPTYING
};

static enum processing_state_t processing_state;

static float response_pressure = -1;

const char *process_state_as_str(enum processing_state_t s);

/* This function is will be passed to COAP_BLOCKING_REQUEST() to handle responses. */
void get_pressure_handler(void *response) {
    const uint8_t *chunk;
    char buffer[256];

    int len = coap_get_payload(response, &chunk);
    unsigned int code = ((coap_packet_t *)response)->code;
    if (code != REST.status.OK || len == 0) {
        response_pressure = -1;
        return;
    }

    strncpy(buffer, (const char *)chunk, len);
    buffer[len] = '\0';
    sscanf(buffer, "%f", &response_pressure);
    printf(" %f\n", response_pressure);
}

static int ok = 0;
void post_pump_handler(void *response) {
    ok = 1;
    unsigned int code = ((coap_packet_t *)response)->code;
    if (code != REST.status.OK) {
        printf("Request bad\n");
        ok = 0;
        return;
    }
}

PROCESS(er_example_client, "Processing device client");
AUTOSTART_PROCESSES(&er_example_client);

PROCESS_THREAD(er_example_client, ev, data) {
    PROCESS_BEGIN();

    static coap_packet_t request[4]; /* This way the packet can be treated as pointer as usual. */

    DEV_0(&server_ipaddr[IP_DEV_0]);
    DEV_1(&server_ipaddr[IP_DEV_1]);
    DEV_CR(&server_ipaddr[IP_DEV_CR]);

    /* receives all CoAP messages */
    coap_init_engine();
    tank_init(&linear_tank, 300, 0, CONST, 40);
    processing_state = EMPTYING;
    etimer_set(&et, TOGGLE_INTERVAL * CLOCK_SECOND);
    static float pressure = 0;
    static int filling_from = 0;

    static const char *msg[] = {"ON", "OFF"};

    coap_init_message(&request[REQ_GET], COAP_TYPE_CON, COAP_GET, 0);
    coap_set_header_uri_path(&request[REQ_GET], service_urls[0]);

    coap_init_message(&request[REQ_PUMP_ON], COAP_TYPE_CON, COAP_POST, 0);
    coap_set_header_uri_path(&request[REQ_PUMP_ON], service_urls[1]);
    coap_set_payload(&request[REQ_PUMP_ON], (uint8_t *)msg[0], /*sizeof(msg[0]) - 1*/ strlen(msg[0]));

    coap_init_message(&request[REQ_PUMP_OFF], COAP_TYPE_CON, COAP_POST, 0);
    coap_set_header_uri_path(&request[REQ_PUMP_OFF], service_urls[1]);
    coap_set_payload(&request[REQ_PUMP_OFF], (uint8_t *)msg[1], strlen(msg[1]));

    while (1) {
        PROCESS_YIELD();

        if (etimer_expired(&et)) {
            pressure = sensor_get(&linear_tank);
            static float dev_pressures[2] = {-1, -1};

            printf("Pressure registered: %f, container %s, state %s\n", pressure, slope_state_as_str(linear_tank.state), process_state_as_str(processing_state));
            // The tank is completely empty and can try to start a new cycle
            if (processing_state == EMPTYING && pressure <= LOW_THRESHOLD) {
                processing_state = TRY_FILLING;
                filling_from = 0;
            }

            // During these states I always ask for tank pressures
            if (processing_state == FILLING || processing_state == TRY_FILLING) {
                printf("Asking pressures\n");
                printf("Device 0:");
                COAP_BLOCKING_REQUEST(&server_ipaddr[0], REMOTE_PORT, &request[0],
                                      get_pressure_handler);
                dev_pressures[0] = response_pressure;
                printf("Device 1:");
                COAP_BLOCKING_REQUEST(&server_ipaddr[1], REMOTE_PORT, &request[0],
                                      get_pressure_handler);
                dev_pressures[1] = response_pressure;
            }

            if (dev_pressures[0] + dev_pressures[1] < HIGH_THRESHOLD - pressure) {
                // Error, can't continue
                printf("pressures: %f,%f\n", dev_pressures[0], dev_pressures[1]);
                printf("Sending alarm, can't continue\n");

                COAP_BLOCKING_REQUEST(&server_ipaddr[filling_from], REMOTE_PORT, &request[2],
                                      post_pump_handler);
                if (ok) {
                    change_state(&linear_tank, CONST);
                    etimer_stop(&et);
                }

                coap_init_message(&request[REQ_ALARM], COAP_TYPE_CON, COAP_POST, 0);
                coap_set_header_uri_path(&request[REQ_ALARM], service_urls[2]);

                ok = 0;
                while (!ok) {
                    COAP_BLOCKING_REQUEST(&server_ipaddr[IP_DEV_CR], REMOTE_PORT, &request[3],
                                          post_pump_handler);
                }
                PROCESS_WAIT_EVENT();
            }

            // Can start a filling cycle
            if (processing_state == TRY_FILLING && HIGH_THRESHOLD - response_pressure < pressure && pressure < HIGH_THRESHOLD) {
                if (dev_pressures[0] < LOW_THRESHOLD) {
                    filling_from = 1;
                }
                printf("Asking pump on to %d\n", filling_from);

                COAP_BLOCKING_REQUEST(&server_ipaddr[filling_from], REMOTE_PORT, &request[REQ_PUMP_ON],
                                      post_pump_handler);
                if (ok) {
                    change_state(&linear_tank, INC);
                    processing_state = FILLING;
                }
            }

            // filling tank, is full; stop to wait
            if (processing_state == FILLING && pressure >= HIGH_THRESHOLD) {
                printf("Asking pump off\n");

                COAP_BLOCKING_REQUEST(&server_ipaddr[0], REMOTE_PORT, &request[2],
                                      post_pump_handler);

                if (ok) {
                    processing_state = WAITING;
                    change_state(&linear_tank, CONST);

                    etimer_stop(&et);
                    etimer_set(&et, PROCESSING_TIME * CLOCK_SECOND);
                    etimer_restart(&et);

                    printf("Sleeping for process\n");
                }
            }
            
            // Tank from which i'm filling is empty
            if (processing_state == FILLING && dev_pressures[filling_from] < LOW_THRESHOLD) {
                printf("Swtiching to tank\n");
                COAP_BLOCKING_REQUEST(&server_ipaddr[filling_from], REMOTE_PORT, &request[REQ_PUMP_OFF],
                                      post_pump_handler);
                if (ok) {
                    change_state(&linear_tank, CONST);
                    processing_state = TRY_FILLING;
                }
            }

            // wait time finished, open outgoing pump and free tank
            if (processing_state == WAITING && etimer_expired(&et)) {
                processing_state = EMPTYING;
                printf("Finishing wait\n");

                change_state(&linear_tank, DEC);
                filling_from = 0;

                etimer_stop(&et);
                etimer_set(&et, TOGGLE_INTERVAL * CLOCK_SECOND);
                etimer_restart(&et);
            }

            if (etimer_expired(&et)) {
                etimer_reset(&et);
            }
        }
    }

    PROCESS_END();
}

static const char *str_value[] = {"TRY_FILLING", "FILLING", "WAITING", "EMPTYING"};
const char *process_state_as_str(enum processing_state_t s) {
    switch (s) {
        case TRY_FILLING:
            return str_value[0];
        case FILLING:
            return str_value[1];
        case WAITING:
            return str_value[2];
        case EMPTYING:
            return str_value[3];
        default:
            return str_value[0];
    }
}
