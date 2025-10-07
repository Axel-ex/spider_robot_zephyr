#ifndef TCP_COMMAND_H
#define TCP_COMMAND_H

struct tcp_command
{
        char* command;
        int times;
};

struct tcp_command tcp_command_builder(char command[32], int times);
#endif // !TCP_COMMAND_H
