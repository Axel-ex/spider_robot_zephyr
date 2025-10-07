#ifndef TCP_COMMAND_H
#define TCP_COMMAND_H

#define RX_BUF_SIZE 32

struct tcp_command
{
        char command[RX_BUF_SIZE];
        int times;
};
extern struct k_msgq tcp_command_q;

#endif // !TCP_COMMAND_H
