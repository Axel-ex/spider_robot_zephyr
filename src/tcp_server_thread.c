#include "tcp_command.h"
#include "zephyr/logging/log.h"
#include "zephyr/net/net_ip.h"
#include "zephyr/net/net_mgmt.h"
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
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
K_MSGQ_DEFINE(tcp_command_q, sizeof(struct tcp_command), 5, 1);

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

/**
 * @brief Network management event handler.
 *
 * Handles WiFi connect/disconnect and IPv4 address events.
 * Signals semaphores on successful connection or IP acquisition.
 *
 * @param event_cb Event callback structure.
 * @param mgmt_event Network management event type.
 * @param iface Network interface pointer.
 */
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
#define TCP_SERVER_STACK_SIZE 2048

void parse_rx_buffer(char* rx_buf, int rx_len, char* command, int* times)
{
    // PARSE buffer
    int i = 0;
    while (i < rx_len && rx_buf[i] != ' ' && rx_buf[i] != '\0' &&
           rx_buf[i] != '\r')
        i++;

    memcpy(command, rx_buf, i);
    command[i] = '\0';

    // Get the number after the command
    char num[2];
    if (rx_buf[i] == ' ' && isdigit(rx_buf[i + 1]))
    {
        i++;
        int j = 0;
        while (rx_buf[i] != '\0' && j < 2)
            num[j++] = rx_buf[i++];
        num[j] = '\0';
    }
    else
        strcpy(num, "1");

    *times = atoi(num);
    if (*times > 10)
        *times = 10;
}

void handle_client(int client_socket)
{
    int rx_len = 0;
    char rx_buf[RX_BUF_SIZE];
    char command_str[RX_BUF_SIZE];
    int times = 1;

    memset(rx_buf, '\0', sizeof(rx_buf));
    rx_len = zsock_recv(client_socket, rx_buf, sizeof(rx_buf), 0);
    if (rx_len < 0)
    {
        LOG_ERR("Error receiving client data");
        return;
    }

    parse_rx_buffer(rx_buf, rx_len, command_str, &times);
    struct tcp_command cmd = {.times = times};
    strcpy(cmd.command, command_str);
    if (k_msgq_put(&tcp_command_q, &cmd, K_NO_WAIT) < 0)
        LOG_DBG("Message queue is full");

    return;
}

int create_listening_socket(void)
{
    int listening_sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listening_sock < 0)
    {
        LOG_ERR("Fail creating socket (%d)", listening_sock);
        return listening_sock;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    int ret = zsock_bind(listening_sock, (const struct sockaddr*)&server_addr,
                         sizeof(server_addr));
    if (ret < 0)
    {
        LOG_ERR("Failed binding the socket (%d)", errno);
        return ret;
    }

    ret = zsock_listen(listening_sock, MAX_CLIENT_QUEUE);
    if (ret < 0)
    {
        LOG_ERR("Failed to listen on socket (%d)", errno);
        zsock_close(listening_sock);
        return ret;
    }
    LOG_INF("Listening on port %d...", SERVER_PORT);

    return listening_sock;
}

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

    int listening_sock = create_listening_socket();
    if (listening_sock < 0)
    {
        LOG_ERR("Couldn't start the tcp server");
        return;
    }

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
                k_sleep(K_MSEC(200));
                continue;
            }
            LOG_INF("Client accepted!");
            handle_client(client_sock);
            zsock_close(client_sock);
            client_sock = -1;
        }

        k_sleep(K_MSEC(500));
    }
}

K_THREAD_DEFINE(tcp_server_thread_id, TCP_SERVER_STACK_SIZE, tcp_server_thread,
                NULL, NULL, NULL, TCP_SERVER_THREAD_PRIORITY, K_USER, 0);
