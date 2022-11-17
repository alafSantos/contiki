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
#include "contiki.h"
#include "contiki-net.h"
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
/* #define SERVER_NODE(ipaddr)   uip_ip6addr(ipaddr, 0xbbbb, 0, 0, 0, 0, 0, 0, 0x1) */

#define LOCAL_PORT UIP_HTONS(COAP_DEFAULT_PORT + 1)
#define REMOTE_PORT UIP_HTONS(COAP_DEFAULT_PORT)

#define TOGGLE_INTERVAL 1
#define LOW_THRESHOLD 11
#define HIGH_THRESHOLD 200

PROCESS(er_example_client, "Erbium Example Client");
AUTOSTART_PROCESSES(&er_example_client);

uip_ipaddr_t server_ipaddr;
static struct etimer et;
linear_tank_t linear_tank;

/* Example URIs that can be queried. */
#define NUMBER_OF_URLS 2
/* leading and ending slashes only for demo purposes, get cropped automatically when setting the Uri-Path */
char *service_urls[NUMBER_OF_URLS] =
    {"/sensors/pressure", "/actuators/pump"};

//static enum client_state {
//  ASKING_PRESSURE
//};

static enum {
  ON,
  OFF
}asked_pump_state;

static float response_pressure = -1;
/* This function is will be passed to COAP_BLOCKING_REQUEST() to handle responses. */
void get_pressure_handler(void *response)
{
  const uint8_t *chunk;
  char buffer[256];

  int len = coap_get_payload(response, &chunk);

  strncpy(buffer, (const char*)chunk, len);
  buffer[len] = '\0';
  sscanf(buffer, "%f", &response_pressure);

  printf("|%.*s", len, (char *)chunk);
}

void post_pump_handler(void *response)
{
  unsigned int code = ((coap_packet_t *)response)->code;
  if (code != REST.status.OK)
  {
    printf("Request bad\n");
    return;
  }
  printf("Request OK\n");
  if (asked_pump_state == ON)
    change_state(&linear_tank, INC);
  else
    change_state(&linear_tank, CONST);
}

PROCESS_THREAD(er_example_client, ev, data)
{
  PROCESS_BEGIN();

  static coap_packet_t request[2]; /* This way the packet can be treated as pointer as usual. */

  DEV_0(&server_ipaddr);

  /* receives all CoAP messages */
  coap_init_engine();
  tank_init(&linear_tank,1000,0,CONST,2);
  etimer_set(&et, TOGGLE_INTERVAL * CLOCK_SECOND);

  while (1)
  {
    PROCESS_YIELD();

    if (etimer_expired(&et))
    {
      float pressure = sensor_get(&linear_tank);
      printf("Pressure registered: %f, container %s",pressure, slope_state_as_str(linear_tank.state));
      if (pressure <= LOW_THRESHOLD)
      {
        printf("Asking pressures\n");

        coap_init_message(&request[0], COAP_TYPE_CON, COAP_GET, 0);
        coap_set_header_uri_path(&request[0], service_urls[0]);

        COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, &request[0],
                              get_pressure_handler);
      }
      else if (HIGH_THRESHOLD - response_pressure < pressure && pressure < HIGH_THRESHOLD )
      {
        printf("Asking pump on\n");

        coap_init_message(&request[1], COAP_TYPE_CON, COAP_POST, 0);
        coap_set_header_uri_path(&request[1], service_urls[1]);

        const char msg[] = "ON";
        asked_pump_state = ON;

        coap_set_payload(&request[1], (uint8_t *)msg, sizeof(msg) - 1);

        COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, &request[1],
                              post_pump_handler);
      }
      else if (pressure >= HIGH_THRESHOLD)
      {
        printf("Asking pump off\n");

        coap_init_message(&request[1], COAP_TYPE_CON, COAP_POST, 0);
        coap_set_header_uri_path(&request[1], service_urls[1]);

        const char msg[] = "OFF";
        asked_pump_state = OFF;

        coap_set_payload(&request[1], (uint8_t *)msg, sizeof(msg) - 1);

        COAP_BLOCKING_REQUEST(&server_ipaddr, REMOTE_PORT, &request[1],
                              post_pump_handler);
      }

      PRINT6ADDR(&server_ipaddr);
      PRINTF(" : %u\n", UIP_HTONS(REMOTE_PORT));

      printf("\n--Done--\n");

      etimer_reset(&et);
    }
  }

  PROCESS_END();
}
