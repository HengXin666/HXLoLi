# 数据报Datagram
## 什么是数据报
> 数据报是通过网络发送的独立的自包含的消息，其是否到达、到达时间和内容无法得到保证。
>
> 即`UDP`

## 如何使用Datagram
### DatagramSocket常用构造方法

```java
// 构建一个绑定在任意端口的收发数据报的套接字
public DatagramSocket() throws SocketException;
// 构建一个绑定在给定端口的收发数据报的套接字
public DatagramSocket(int port) throws SocketException;
```

### DatagramSocket常用方法

```java
// 发送给定的数据包
public void send(DatagramPacket p) throws IOException;
// 接收数据至给定的数据包
public synchronized void receive(DatagramPacket p) throws IOException;
// 设置链接的超时时间，0表示不会超时
public synchronized void setSoTimeout(int timeout) throws SocketException;
```
### DatagramPacket常用构造方法

```java
// 构建一个接收数据的数据包
public DatagramPacket(byte buf[], int length);
// 构建一个发送数据的数据包
public DatagramPacket(byte buf[], int offset, int length,InetAddress address, int port);
```

### DatagramPacket常用方法

```java
// 获取发送数据的主机IP地址
public synchronized InetAddress getAddress();
// 获取发送数据的主机使用的端口
public synchronized int getPort();
// 获取数据包中数据的长度
public synchronized int getLength();
```

示例代码

```java
package datagram;

import java.io.IOException;
import java.net.*;

/**
 * UDP工具类
 */
public class DatagtamUtil {
    private static final int PACKET_MAX_SIZE = 8192;

    /**
     * 接收数据包
     * @param socket 接收的套接字
     * @return UDP数据包
     */
    public static DatagramPacket recv(DatagramSocket socket) {
        byte[] data = new byte[DatagtamUtil.PACKET_MAX_SIZE];
        DatagramPacket packet = new DatagramPacket(data, data.length);
        try {
            socket.receive(packet);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return packet;
    }

    /**
     * 发送数据包
     * @param socket 数据报套接字
     * @param msg 消息
     * @param ip 目标ip
     * @param port 目标端口
     */
    public static void send(DatagramSocket socket, String msg, String ip, int port) throws IOException {
        byte[] data = msg.getBytes();
        InetAddress address = InetAddress.getByName(ip);
        // 创建一个数据包用于发送
        DatagramPacket packet = new DatagramPacket(data, 0, data.length, address, port);
        socket.send(packet);
    }
}

package datagram;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class Ser {
    public static void main(String[] args) { // 测试使用
        try {
            Ser ser = new Ser(6666);
            ser.run();
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
    }

    private DatagramSocket serSocket;

    Ser(int port) throws SocketException {
        this.serSocket = new DatagramSocket(port);
    }

    public void run() {
        while (true) {
            // 接收
            DatagramPacket packet = DatagtamUtil.recv(this.serSocket);
            System.out.println("收到来自 " + packet.getAddress().toString() + " 的数据: " + (new String(packet.getData(), 0, packet.getLength())));

            // 回声
            try {
                DatagtamUtil.send(this.serSocket, "您好, 我已经收到了: " + (new String(packet.getData(), 0, packet.getLength())), packet.getAddress().getHostAddress(), packet.getPort());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}

package datagram;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class Cli {
    public static void main(String[] args) {
        try {
            Cli cli = new Cli();
            cli.run("127.0.0.1", 6666);
        } catch (SocketException e) {
            throw new RuntimeException(e);
        }
    }

    private DatagramSocket cliSocket;

    Cli() throws SocketException {
        this.cliSocket = new DatagramSocket(); // 绑定任意端口
    }

    public void run(String host, int port) {
        // 对目标分别发送两次消息
        try {
            DatagtamUtil.send(this.cliSocket, "Hello My Ser SaMa ?", host, port);

            DatagramPacket recv = DatagtamUtil.recv(this.cliSocket);
            System.out.println("收到消息: " + (new String(recv.getData(), 0, recv.getLength())));

            DatagtamUtil.send(this.cliSocket, "So Fxxk Easy ka?", host, port);
            recv = DatagtamUtil.recv(this.cliSocket);
            System.out.println("收到消息: " + (new String(recv.getData(), 0, recv.getLength())));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
```
