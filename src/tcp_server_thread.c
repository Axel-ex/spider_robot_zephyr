#include "zephyr/logging/log.h"
#include <zephyr/kernel.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

#define SSID "MEO-BD8310"
#define PSK "9f24731014"

LOG_MODULE_REGISTER(tcp_server, LOG_LEVEL_DBG);

void wifi_connect(void)
{
    struct net_if* iface = net_if_get_default();

    struct wifi_connect_req_params wifi_params = {0};

    wifi_params.ssid = SSID;
    wifi_params.psk = PSK;
    wifi_params.ssid_length = strlen(SSID);
    wifi_params.psk_length = strlen(PSK);
    wifi_params.channel = WIFI_CHANNEL_ANY;
    wifi_params.security = WIFI_SECURITY_TYPE_PSK;
    wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ;
    wifi_params.mfp = WIFI_MFP_OPTIONAL;

    printk("Connecting to SSID: %s\n", wifi_params.ssid);

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params,
                 sizeof(struct wifi_connect_req_params)))
    {
        printk("WiFi Connection Request Failed\n");
    }
}

// TCP_SERVER
#define TCP_SERVER_THREAD_PRIORITY 20
#define TCP_SERVER_STACK_SIZE 1024 // Probably will have to be bigger

void tcp_server_thread(void)
{
    wifi_connect();

    LOG_INF("Wifi connected!");
    while (true)
    {
        k_sleep(K_SECONDS(2));
    }
}

K_THREAD_DEFINE(tcp_server_thread_id, TCP_SERVER_STACK_SIZE, tcp_server_thread,
                NULL, NULL, NULL, TCP_SERVER_THREAD_PRIORITY, K_USER, 0);
