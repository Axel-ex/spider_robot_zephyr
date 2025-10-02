#include "zephyr/logging/log.h"
#include "zephyr/net/net_ip.h"
#include "zephyr/net/net_mgmt.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/wifi_mgmt.h>

#define SERVER_PORT 5000
#define MAX_CLIENT_QUEUE 1

LOG_MODULE_REGISTER(tcp_server, LOG_LEVEL_DBG);
K_SEM_DEFINE(wifi_connected, 0, 1);
K_SEM_DEFINE(ipv4_obtained, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void handle_ipv4_result(struct net_if* iface)
{
    int i = 0;

    for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++)
    {
        char buf[NET_IPV4_ADDR_LEN];

        if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP)
        {
            continue;
        }

        LOG_INF("IPv4 address: %s",
                net_addr_ntop(
                    AF_INET,
                    &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
                    buf, sizeof(buf)));
    }

    k_sem_give(&ipv4_obtained);
}

static void net_event_handler(struct net_mgmt_event_callback* event_cb,
                              uint64_t mgmt_event, struct net_if* iface)
{
    switch (mgmt_event)
    {
        case NET_EVENT_WIFI_CONNECT_RESULT:
        {
            const struct wifi_status* status =
                (const struct wifi_status*)event_cb->info;
            if (status->status == 0)
            {
                LOG_INF("Wifi connected!");
                k_sem_give(&wifi_connected);
            }
            else
            {
                LOG_ERR("Wifi connection failed (%d)", status->status);
            }
            break;
        }
        case NET_EVENT_WIFI_DISCONNECT_RESULT:
        {
            const struct wifi_status* status =
                (const struct wifi_status*)event_cb->info;
            if (status->status)
                LOG_DBG("Wifi disconnection request (%d)", status->status);
            else
            {
                LOG_ERR("Wifi diconnected");
                k_sem_take(&wifi_connected, K_NO_WAIT);
            }
        }
        case NET_EVENT_IPV4_ADDR_ADD:
            handle_ipv4_result(iface);
            break;
        default:
            break;
    }
}

void wifi_connect(void)
{
    struct net_if* iface = net_if_get_default();

    struct wifi_connect_req_params wifi_params = {0};

    wifi_params.ssid = CONFIG_MY_WIFI_SSID;
    wifi_params.psk = CONFIG_MY_WIFI_PSK;
    wifi_params.ssid_length = strlen(CONFIG_MY_WIFI_SSID);
    wifi_params.psk_length = strlen(CONFIG_MY_WIFI_PSK);
    wifi_params.channel = WIFI_CHANNEL_ANY;
    wifi_params.security = WIFI_SECURITY_TYPE_PSK;
    wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ;
    wifi_params.mfp = WIFI_MFP_OPTIONAL;

    LOG_INF("Connecting to SSID: %s", wifi_params.ssid);

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params,
                 sizeof(struct wifi_connect_req_params)))
        LOG_ERR("WiFi Connection Request Failed");
}

// TCP_SERVER
#define TCP_SERVER_THREAD_PRIORITY 5
#define TCP_SERVER_STACK_SIZE 1024 // Probably will have to be bigger

void handle_client(int client_socket) { return; }

void tcp_server_thread(void)
{
    net_mgmt_init_event_callback(&wifi_cb, net_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT |
                                     NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_init_event_callback(&ipv4_cb, net_event_handler,
                                 NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_cb);
    net_mgmt_add_event_callback(&ipv4_cb);

    wifi_connect();
    k_sem_take(&wifi_connected, K_FOREVER);
    k_sem_take(&ipv4_obtained, K_FOREVER);

    int listening_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listening_sock < 0)
    {
        LOG_ERR("Fail creating socket (%d)", listening_sock);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (zsock_bind(listening_sock, (const struct sockaddr*)&server_addr,
                   sizeof(server_addr)) < 0)
    {
        LOG_ERR("Failed binding the socket (%d)", errno);
        return;
    }

    if (zsock_listen(listening_sock, MAX_CLIENT_QUEUE) < 0)
    {
        LOG_ERR("Failed to listen on socket (%d)", errno);
        zsock_close(listening_sock);
        return;
    }
    LOG_INF("Listening on port %d...", SERVER_PORT);

    int client_sock = -1;
    while (true)
    {
        if (client_sock < 0)
        {
            LOG_DBG("Waiting for client connection...");
            client_sock = zsock_accept(listening_sock, NULL,
                                       NULL); // block until a client connects
            if (client_sock < 0)
            {
                LOG_ERR("Failed to accept client (%d)", errno);
                k_sleep(K_SECONDS(1));
                continue;
            }
            LOG_INF("Client accepted!");
            handle_client(client_sock);
            zsock_close(client_sock);
            client_sock = -1;
        }

        k_sleep(K_SECONDS(2));
    }
}

K_THREAD_DEFINE(tcp_server_thread_id, TCP_SERVER_STACK_SIZE, tcp_server_thread,
                NULL, NULL, NULL, TCP_SERVER_THREAD_PRIORITY, K_USER, 0);
