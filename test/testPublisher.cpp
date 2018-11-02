//
// 自定义ROUTER-REQ路由
//
#include "libzmq/include/zhelpers.h"
#include <pthread.h>
#include <iostream>

#define NBR_WORKERS 10

std::string address = "ipc://routing.ipc";
static void *worker_task(void *args)
{
    void *context = zmq_init(1);
    void *worker = zmq_socket(context, ZMQ_SUB);

    // s_set_id()函数会根据套接字生成一个可打印的字符串，
    // 并以此作为该套接字的标识。
    std::string name = "worker0";
    zmq_setsockopt(worker, ZMQ_SUBSCRIBE, name.c_str(), name.size());
    zmq_connect(worker, address.c_str());

    int total = 0;
    while (1)
    {
        // 告诉ROUTER我已经准备好了
        s_send(worker, (char *)"ready");
        // 从ROUTER中获取工作，直到收到结束的信息
        char *workload = s_recv(worker);
        int finished = (strcmp(workload, "END") == 0);
        free(workload);
        if (finished)
        {
            printf("Processed: %d tasks\n", total);
            break;
        }
        total++;
        //等待一段时间
        sleep(1);
    }
    zmq_close(worker);
    zmq_term(context);
    return NULL;
}

int main(void)
{
    void *context = zmq_init(1);
    void *client = zmq_socket(context, ZMQ_ROUTER);
    zmq_bind(client, "ipc://routing.ipc");

    int worker_nbr;
    for (worker_nbr = 0; worker_nbr < NBR_WORKERS; worker_nbr++)
    {
        pthread_t worker;
        pthread_create(&worker, NULL, worker_task, NULL);
    }
    int task_nbr;
    std::cout << "main:start rcv!" << std::endl;
    for (task_nbr = 0; task_nbr < NBR_WORKERS * 10; task_nbr++)
    {
        //接受一条来自req的请求
        char *address = s_recv(client);
        char *empty = s_recv(client);
        free(empty);
        char *ready = s_recv(client);
        free(ready);

        std::cout << "address:" << address << std::endl;
        //应答
        s_sendmore(client, address);
        s_sendmore(client, (char *)"");
        s_send(client, (char *)"This is the workload");
        free(address);
    }

    // 通知所有REQ套接字结束工作
    for (worker_nbr = 0; worker_nbr < NBR_WORKERS; worker_nbr++)
    {
        //接受
        char *address = s_recv(client);
        char *empty = s_recv(client);
        free(empty);
        char *ready = s_recv(client);
        free(ready);

        //应答
        s_sendmore(client, address);
        s_sendmore(client, (char *)"");
        s_send(client, (char *)"END");
        free(address);
    }
    zmq_close(client);
    zmq_term(context);
    return 0;
}