#pragma once

#include "zmq.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>

//  Provide random number from 0..(num-1)
#define within(num) (int)((float)(num)*random() / (RAND_MAX + 1.0))

//  Receive 0MQ string from socket and convert into C string
//  Caller must free returned string.
static char *
s_recv(void *socket, int flag_ = 0)
{
    zmq_msg_t message;
    zmq_msg_init(&message);
    char *string = NULL;
    if (zmq_recvmsg(socket, &message, flag_) < 0)
        // exit(1); //  Context terminated, exit
        return string;

    int size = zmq_msg_size(&message);
    string = (char *)malloc(size + 1);
    memcpy(string, zmq_msg_data(&message), size);
    zmq_msg_close(&message);
    string[size] = 0;
    return (string);
}

//  Convert C string to 0MQ string and send to socket
static int
s_send(void *socket, char *string)
{
    int rc;
    zmq_msg_t message;
    zmq_msg_init_size(&message, strlen(string));
    memcpy(zmq_msg_data(&message), string, strlen(string));
    rc = zmq_sendmsg(socket, &message, 0);
    // assert(rc >= 0);
    zmq_msg_close(&message);
    return (rc);
}

//  Sends string as 0MQ string, as multipart non-terminal
static int
s_sendmore(void *socket, char *string)
{
    int rc;
    zmq_msg_t message;
    zmq_msg_init_size(&message, strlen(string));
    memcpy(zmq_msg_data(&message), string, strlen(string));
    rc = zmq_sendmsg(socket, &message, ZMQ_SNDMORE);
    zmq_msg_close(&message);
    // assert(rc >= 0);
    return (rc);
}

//  Receives all message parts from socket, prints neatly
//
static void
s_dump(void *socket)
{
    puts("----------------------------------------");
    while (1)
    {
        //  Process all parts of the message
        zmq_msg_t message;
        zmq_msg_init(&message);
        zmq_recvmsg(socket, &message, 0);

        //  Dump the message as text or binary
        char *data = (char *)zmq_msg_data(&message);
        int size = zmq_msg_size(&message);
        int is_text = 1;
        int char_nbr;
        for (char_nbr = 0; char_nbr < size; char_nbr++)
            if ((unsigned char)data[char_nbr] < 32 || (unsigned char)data[char_nbr] > 127)
                is_text = 0;

        printf("[%03d] ", size);
        for (char_nbr = 0; char_nbr < size; char_nbr++)
        {
            if (is_text)
                printf("%c", data[char_nbr]);
            else
                printf("%02X", (unsigned char)data[char_nbr]);
        }
        printf("\n");

        int more; //  Multipart detection
        size_t more_size = sizeof(more);
        zmq_getsockopt(socket, ZMQ_RCVMORE, &more, &more_size);
        zmq_msg_close(&message);
        if (!more)
            break; //  Last message part
    }
}

//  Set simple random printable identity on socket
//
static void
s_set_id(void *socket)
{
    char identity[10];
    sprintf(identity, "%04X-%04X", within(0x10000), within(0x10000));
    zmq_setsockopt(socket, ZMQ_IDENTITY, identity, strlen(identity));
}

//  Report 0MQ version number
//
static void
s_version(void)
{
    int major, minor, patch;
    zmq_version(&major, &minor, &patch);
    printf("Current 0MQ version is %d.%d.%d\n", major, minor, patch);
}
