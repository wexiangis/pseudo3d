/*
 *  接收来自android设备的姿态参数
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "tcpServer.h"
#include "mpu6050.h"
#include "delayus.h"

#define EPOLL_RESPOND_NUM 10000 //epoll最大同时管理句柄数量
#define PKG_MAX 10240           //服务器收包缓冲区大小

typedef struct TcpServer
{
    int fd;
    int port;
    //接入客户端fd缓存数组, [0]/fd [1]/使能标志
    int client_array[EPOLL_RESPOND_NUM][2];
    char buff[PKG_MAX];
    //服务器接收数据回调函数
    void (*callBack)(struct TcpServer *ts, unsigned char *buff, unsigned int buffLen);
    //数据
    float data[3];
} Tcp_Server;

int arrayAdd(int array[][2], int arraySize, int value)
{
    int i;
    for (i = 0; i < arraySize; i++)
    {
        if (array[i][1] == 0)
        {
            array[i][0] = value;
            array[i][1] = 1;
            return 0;
        }
    }
    return -1;
}
int arrayRemove(int array[][2], int arraySize, int value)
{
    int i;
    for (i = 0; i < arraySize; i++)
    {
        if (array[i][0] == value)
        {
            array[i][0] = 0;
            array[i][1] = 0;
            return 0;
        }
    }
    return -1;
}

void server_thread(void *arge)
{
    int ret, count;
    int accept_fd;
    socklen_t socAddrLen;
    struct sockaddr_in acceptAddr;
    struct sockaddr_in serverAddr;
    Tcp_Server *ts = (Tcp_Server *)arge;
    memset(&serverAddr, 0, sizeof(serverAddr)); // 数据初始化--清零
    serverAddr.sin_family = AF_INET;            // 设置为IP通信
    serverAddr.sin_addr.s_addr = INADDR_ANY;    // 服务器IP地址
    serverAddr.sin_port = htons(ts->port);      // 服务器端口号
    socAddrLen = sizeof(struct sockaddr_in);

    //socket init
    ts->fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ts->fd <= 0)
    {
        printf("server cannot create socket !\r\n");
        return;
    }

    //设置为非阻塞接收
    ret = fcntl(ts->fd, F_GETFL, 0);
    fcntl(ts->fd, F_SETFL, ret | O_NONBLOCK);

    //bind sockfd & addr
    count = 0;
    while (bind(ts->fd, (struct sockaddr *)&serverAddr, sizeof(struct sockaddr)) < 0)
    {
        if (++count > 1000)
        {
            printf("bind timeout\r\n");
            return;
        }
        delayms(1);
    }

    //listen sockfd
    ret = listen(ts->fd, 0);
    if (ret < 0)
    {
        printf("server cannot listen request\r\n");
        close(ts->fd);
        return;
    }

    // 创建一个epoll句柄
    int epoll_fd;
    epoll_fd = epoll_create(EPOLL_RESPOND_NUM);
    if (epoll_fd < 0)
    {
        printf("server epoll_create failed\r\n");
        return;
    }

    int nfds;              // epoll监听事件发生的个数
    struct epoll_event ev; // epoll事件结构体
    struct epoll_event events[EPOLL_RESPOND_NUM];
    ev.events = EPOLLIN | EPOLLET; // 	EPOLLIN		EPOLLET;    监听事件类型
    ev.data.fd = ts->fd;
    // 向epoll注册server_sockfd监听事件
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ts->fd, &ev) < 0)
    {
        printf("server epll_ctl : ts->fd register failed\r\n");
        close(epoll_fd);
        exit(1);
    }

    printf("========== server start ! ==========\r\n");
    while (1)
    {
        // 等待事件发生
        nfds = epoll_wait(epoll_fd, events, EPOLL_RESPOND_NUM, -1); // -1表示阻塞、其它数值为超时
        if (nfds < 0)
        {
            printf("server start epoll_wait failed\r\n");
            close(epoll_fd);
            exit(1);
        }

        for (count = 0; count < nfds; count++)
        {
            //===================epoll错误 ===================
            if ((events[count].events & EPOLLERR) || (events[count].events & EPOLLHUP))
            {
                printf("accept close : %d\r\n", events[count].data.fd); //与客户端连接出错, 主动断开当前 连接
                                                                        //向epoll删除client_sockfd监听事件
                                                                        //ev.events = EPOLLIN|EPOLLET;
                ev.data.fd = events[count].data.fd;
                if (epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[count].data.fd, &ev) < 0)
                {
                    printf("server epoll_ctl : EPOLL_CTL_DEL failed !\r\n");
                    close(epoll_fd);
                    exit(1);
                }
                arrayRemove(ts->client_array, EPOLL_RESPOND_NUM, events[count].data.fd); //从数组剔除fd
                close(events[count].data.fd);                                            //关闭通道
            }
            //===================新通道接入事件===================
            else if (events[count].data.fd == ts->fd)
            {
                //轮巡可能接入的新通道 并把通道号记录在accept_fd[]数组中
                accept_fd = accept(ts->fd, (struct sockaddr *)&acceptAddr, &socAddrLen);
                if (accept_fd >= 0)
                {
                    //向epoll注册client_sockfd监听事件
                    //ev.events = EPOLLIN|EPOLLET;
                    ev.data.fd = accept_fd;
                    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, accept_fd, &ev) < 0)
                    {
                        printf("server epoll_ctl : EPOLL_CTL_ADD failed !\r\n");
                        close(epoll_fd);
                        exit(1);
                    }
                    printf("server fd/%d : accept\r\n", accept_fd);
                    arrayAdd(ts->client_array, EPOLL_RESPOND_NUM, accept_fd);
                }
            }
            //===================接收数据事件===================
            else if (events[count].events & EPOLLIN)
            {
                //接收数据
                memset(ts->buff, 0, sizeof(ts->buff));
                ret = recv(events[count].data.fd, ts->buff, sizeof(ts->buff), MSG_NOSIGNAL);
                //回调
                if (ret > 0)
                {
                    if (ts->callBack)
                        ts->callBack(ts, (unsigned char *)ts->buff, ret);
                }
                //检查异常
                else
                {
                    if (errno > 0 && (errno == EAGAIN || errno == EINTR))
                        ;
                    else
                    {
                        printf("accept close : %d / check error: %d\r\n", events[count].data.fd, errno);
                        //向epoll删除client_sockfd监听事件
                        //ev.events = EPOLLIN|EPOLLET;
                        ev.data.fd = events[count].data.fd;
                        if (epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[count].data.fd, &ev) < 0)
                        {
                            printf("server epoll_ctl : EPOLL_CTL_DEL failed !\r\n");
                            close(epoll_fd);
                            exit(1);
                        }
                        arrayRemove(ts->client_array, EPOLL_RESPOND_NUM, events[count].data.fd); //从数组剔除fd
                        close(events[count].data.fd);                                            //关闭通道
                    }
                }
            }
            //===================发送数据事件===================
            else if (events[count].events & EPOLLOUT)
                ;
        }
    }
    //关闭epoll句柄
    close(epoll_fd);
    //关闭socket
    close(ts->fd);
}

void server_callBack(Tcp_Server *ts, unsigned char *buff, unsigned int buffLen)
{
    float *pFloat;
    //printf("port/%d recv/%d\r\n", ts->port, buffLen);
    //3个float数大小
    while (buffLen >= 12)
    {
        pFloat = (float *)buff;
        ts->data[0] = pFloat[0];
        ts->data[1] = pFloat[1];
        ts->data[2] = pFloat[2];
        buff += 12;
        buffLen -= 12;
    }
}

static Tcp_Server *tsGyr, *tsAcc, *tsPry;

/*
 *  返回0正常
 */
int tcpServer_get(float *pry, float *gyro, float *accel)
{
    pthread_t th;
    if (!tsGyr || !tsAcc || !tsPry)
    {
        tsGyr = (Tcp_Server *)calloc(1, sizeof(Tcp_Server));
        tsAcc = (Tcp_Server *)calloc(1, sizeof(Tcp_Server));
        tsPry = (Tcp_Server *)calloc(1, sizeof(Tcp_Server));
        tsGyr->port = TCPSERVER_PORT_GYR;
        tsAcc->port = TCPSERVER_PORT_ACC;
        tsPry->port = TCPSERVER_PORT_PRY;
        tsGyr->callBack = &server_callBack;
        tsAcc->callBack = &server_callBack;
        tsPry->callBack = &server_callBack;
        pthread_create(&th, NULL, (void *)&server_thread, (void *)tsGyr);
        pthread_create(&th, NULL, (void *)&server_thread, (void *)tsAcc);
        pthread_create(&th, NULL, (void *)&server_thread, (void *)tsPry);
        return 1;
    }
    //
    if (pry)
    {
        pry[0] = tsPry->data[0];
        pry[1] = tsPry->data[1];
        pry[2] = tsPry->data[2];
    }
    if (gyro)
    {
        gyro[0] = tsGyr->data[0];
        gyro[1] = tsGyr->data[1];
        gyro[2] = tsGyr->data[2];
    }
    if (accel)
    {
        accel[0] = tsAcc->data[0];
        accel[1] = tsAcc->data[1];
        accel[2] = tsAcc->data[2];
    }
    return 0;
}
