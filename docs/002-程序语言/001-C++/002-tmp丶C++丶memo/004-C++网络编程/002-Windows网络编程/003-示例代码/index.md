
```C++
#include <iostream>
#include <WinSock2.h>
#include <MSWSock.h>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>


#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")

#include "HXnet.h"
#include <string>
#include <map>

#define STR_MAX_SIZE 1024


// IOCP 事件类型
enum IocpEventType {
    ACCEPT, // 等待连接
    RECV,   // 接收
    SEND    // 发送
};

typedef struct HXData
{
    OVERLAPPED Overlapped;            // 重叠结构体必需要放到第一个位置
    SOCKET cli_socket;                // 客户端套接字
    CHAR str_data[STR_MAX_SIZE];    // 收/发 的数据
    WSABUF buf_data;                // 用于描述数据缓冲区
    IocpEventType eventType;        // 事件类型
    DWORD bufferCount;                // 接收的字节数
    DWORD lFlags;                    // 作用未知

public:
    // 构造函数
    HXData(SOCKET cli_socket, IocpEventType eventType) {
        memset(this, 0, sizeof(HXData));
        this->cli_socket = cli_socket;
        this->buf_data.buf = this->str_data;
        this->buf_data.len = sizeof(this->str_data);
        this->bufferCount = 0;
        this->lFlags = 0;
        this->eventType = eventType;
    }
} HXData;


typedef struct _send_cli_package
{
    std::map<SOCKET, std::string> cli_map;
    CHAR send_data[STR_MAX_SIZE];
public:
    _send_cli_package() : cli_map(), send_data() {}
} SendCliPackage;


void accepHX(SOCKET ser_socket, HANDLE sev_iocp);
void recvHX(HXData* data);

// 等待连接
void accepHX(SOCKET ser_socket, HANDLE sev_iocp)
{
    // 准备一个未关联的, 用于连接客户端的套接字
    SOCKET cli_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    // 关联完成端口
    if (!CreateIoCompletionPort((HWND)cli_socket, sev_iocp, NULL, 0)) {
        HX::tools::HXprint::getHXprint()->ptr_pError("新建客户端套接字, 关联完成端口时出错!");
        return;
    }

    HXData* data = new HXData(cli_socket, ACCEPT);

    char szBuff[STR_MAX_SIZE] = { 0 }; // 单纯只是缓冲区 (void *)
    DWORD dwRecved = 0;

    AcceptEx(
        ser_socket,
        cli_socket,
        szBuff,
        0,
        sizeof(sockaddr) + 16,
        sizeof(sockaddr) + 16,
        &dwRecved,
        (LPOVERLAPPED)data
    );
}

void recvHX(HXData *data)
{
    if (WSARecv(data->cli_socket, &data->buf_data, 1, &data->bufferCount, &data->lFlags, (LPOVERLAPPED)data, NULL)) {
        if (WSAGetLastError() != WSA_IO_PENDING)
            HX::tools::HXprint::getHXprint()->ptr_pError("创建接收任务时出现错误");
    }
}

void sendHX(SOCKET notSendCli, SendCliPackage* p)
{
    int len = strlen(p->send_data);
    for (auto it : p->cli_map)
    {
        if (it.first != notSendCli)
            send(it.first, p->send_data, len, 0);
    }
}

int main()
{
    HX::net::HXServer server("127.0.0.1", 11451, 6);

    // 创建完成端口
    HANDLE sev_iocp = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, NULL, 0);

    auto print = HX::tools::HXprint::getHXprint();

    // 关联完成端口
    if (!CreateIoCompletionPort((HANDLE)server.getSerSocket(), sev_iocp, NULL, 0)) {
        print->ptr_pError("关联完成端口时出错!");
    }

    // 投递一个等待连接的任务
    accepHX(server.getSerSocket(), sev_iocp);

    SendCliPackage sendCliarr;

    // 处理 主循环
    while (true)
    {
        DWORD dwBytesTranfered = 0;
        ULONG_PTR uKey;
        LPOVERLAPPED data_mae = NULL;
        GetQueuedCompletionStatus(sev_iocp, &dwBytesTranfered, &uKey, &data_mae, INFINITE);

        HXData* data = (HXData*)data_mae;
        switch (data->eventType)
        {
        case ACCEPT:
        {
            // 添加到接收数据任务
            data->eventType = RECV;
            recvHX(data);
            sendCliarr.cli_map.insert(std::make_pair(data->cli_socket, std::string("")));
            print->ptr_pInfo("客户端已连接!");
            // 再创建一个等待连接的任务
            accepHX(server.getSerSocket(), sev_iocp);
            break;
        }
        case RECV:
        {
            if (dwBytesTranfered == 0)
            {
                print->ptr_pInfo("客户端已断开连接!");
                sendCliarr.cli_map.erase(sendCliarr.cli_map.find(data->cli_socket));

                // 取消对完成端口的关联
                CreateIoCompletionPort((HANDLE)data->cli_socket, NULL, 0, 0);

                closesocket(data->cli_socket);
                delete data;
                continue;
            }

            strcpy(sendCliarr.send_data, data->str_data);
            auto p = sendCliarr.cli_map.find(data->cli_socket);
            if (p->second.size() == 0) {
                p->second = std::string(sendCliarr.send_data);
                sprintf(sendCliarr.send_data, "[%s] 加入了房间!", p->second.c_str());
            }
            else {
                sprintf(sendCliarr.send_data, "[%s]: %s", p->second.c_str(), data->str_data);
            }
            sendHX(data->cli_socket, &sendCliarr);

            print->ptr_pInfo("收到消息: %s", data->str_data);
            recvHX(data);
            break;
        }
        default:
            break;
        }
    }

    return 0;
}
```
