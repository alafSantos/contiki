#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../ips.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "contiki.h"
#include "er-coap-engine.h"
#include "httpd-simple.h"
#include "net/ip/uip-debug.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/netstack.h"
#include "net/rpl/rpl.h"

static char buf[512];
static int blen;
#define ADD(...)                                                       \
    do {                                                               \
        blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__); \
    } while (0)

#define LOCAL_PORT UIP_HTONS(COAP_DEFAULT_PORT + 1)
#define REMOTE_PORT UIP_HTONS(COAP_DEFAULT_PORT)

#define IP_DEV_0 0
#define IP_DEV_1 1
#define IP_DEV_CR 2

#define REQ_GET 0
#define REQ_PUMP_ON 1
#define REQ_PUMP_OFF 2
#define REQ_ALARM 3

#define NUMBER_OF_URLS 2

static uip_ipaddr_t server_ipaddr[1];
static coap_packet_t request[1];

char *service_urls[NUMBER_OF_URLS] = {"/sensors/pressure", "/alarm"};

static float response_pressure = -1;

static int ok = 0;
void get_pressure_handler(void *response);
void delete_alarm_handler(void *response);

process_event_t ev_g;
struct process proc_g;
PROCESS(webserver_process, "IoT-LAB Web server");
PROCESS_THREAD(webserver_process, ev, data) {
    PROCESS_BEGIN();

    httpd_init();
    coap_init_engine();
    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
        httpd_appcall(data);
    }

    PROCESS_END();
}
AUTOSTART_PROCESSES(&webserver_process);

int client_done = 0;
PROCESS(coapclient_process, "coap client send");
PROCESS_THREAD(coapclient_process, ev, data) {
    PROCESS_BEGIN();
    client_done = 0;
    printf("Clent started\n");
    COAP_BLOCKING_REQUEST(&server_ipaddr[0], REMOTE_PORT, &request[0],
                          get_pressure_handler);

    client_done = 1;
    printf("Client done\n");
    //process_post(&webserver_process,PROCESS_EVENT_CONTINUE,NULL);
    PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static PT_THREAD(http_callback(struct httpd_state *s)) {
    // PT_BEGIN(&s->outputpt);
    PSOCK_BEGIN(&s->sout);

    printf("|%s|\n", s->filename);
    int no_answer = 1;
    if (/*s->method == GET &&*/ strncmp(s->filename, "/pressure/", sizeof("/pressure/") - 1) == 0) {
        if (strncmp(s->filename, "/pressure/dev_0", sizeof("/pressure/dev_0")) == 0) {
            DEV_0(&server_ipaddr[0]);
        }

        if (strncmp(s->filename, "/pressure/dev_1", sizeof("/pressure/dev_1") - 1) == 0) {
            DEV_1(&server_ipaddr[0]);
        }

        if (strncmp(s->filename, "/pressure/dev_p", sizeof("/pressure/dev_p") - 1) == 0) {
            DEV_P(&server_ipaddr[0]);
        }
        no_answer = 0;
        coap_init_message(&request[0], COAP_TYPE_CON, COAP_GET, 0);
        coap_set_header_uri_path(&request[0], service_urls[0]);

        printf("Asked pressure\n");
        //process_start(&coapclient_process,NULL);
        //COAP_BLOCKING_REQUEST(&server_ipaddr[0], REMOTE_PORT, &request[0],
        //                      get_pressure_handler);
        PT_YIELD_UNTIL(&s->outputpt, client_done != 0);
        //while(!client_done)
        //    PT_YIELD(&s->outputpt);
            //PT_YIELD(PROCESS_CURRENT());

        printf("BAck from yeld!\n");
        if (response_pressure != -1) {
            sprintf(buf, "%f", response_pressure);
            SEND_STRING(&s->sout, buf);
        }
    }

    if (s->method == POST && strncmp(s->filename, "/alarm", sizeof("/alarm") - 1) == 0) {
        coap_init_message(&request[0], COAP_TYPE_CON, COAP_POST, 0);
        coap_set_header_uri_path(&request[0], service_urls[1]);

        no_answer = 0;
        DEV_CR(&server_ipaddr[0]);
        ok = 0;
        while (!ok) {
            //COAP_BLOCKING_REQUEST(&server_ipaddr[0], REMOTE_PORT, &request[0],
            //                      delete_alarm_handler);
        }
    }

    if (s->method == DELETE && strncmp(s->filename, "/alarm", sizeof("/alarm") - 1) == 0) {
        coap_init_message(&request[0], COAP_TYPE_CON, COAP_DELETE, 0);
        coap_set_header_uri_path(&request[0], service_urls[1]);
        no_answer = 0;
        DEV_CR(&server_ipaddr[0]);
        ok = 0;
        while (!ok) {
            //COAP_BLOCKING_REQUEST(&server_ipaddr[0], REMOTE_PORT, &request[0],
            //                      delete_alarm_handler);
        }
    }
    if (no_answer) {
        SEND_STRING(&s->sout, "Error: Page not found");
    }
    PSOCK_END(&s->sout);
    // PT_END(&s->outputpt);
}

/*---------------------------------------------------------------------------*/
httpd_simple_script_t
httpd_simple_get_script(const char *name) {
    return http_callback;
}

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

void delete_alarm_handler(void *response) {
    ok = 1;
    unsigned int code = ((coap_packet_t *)response)->code;
    if (code != REST.status.OK) {
        printf("Request bad\n");
        ok = 0;
        return;
    }
}
