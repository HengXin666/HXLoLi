# 套接字Socket
## 什么是Socket
> 套接字是网络上运行的两个程序之间双向通讯链接的一个端点。 套接字类用于表示客户端程序和服务器程序之间的连接。`java.net`包提供了两个类`Socket`和`ServerSocket`分别实现连接的客户端和连接的服务器。

## 如何使用Socket
> 通常，服务器在特定计算机上运行，并具有绑定到特定端口号的套接字。 服务器只是等待，侦听套接字以请求客户端发出连接请求。

> 在客户端上：客户端知道服务器在其上运行的计算机的主机名以及服务器在其上侦听的端口号。 为了发出连接请求，客户端尝试在服务器的机器和端口上与服务器会合。 客户端还需要向服务器标识自己，以便客户端绑定到在此连接期间将使用的本地端口号。 这通常是由系统分配的。

### Socket常用构造方法

```java
// 创建一个套接字，连向给定IP的主机，并与该主机给定端口的应用通信
public Socket(String host, int port) throws UnknownHostException, IOException;
// 创建一个套接字，连向给定IP信息的主机，并与该主机给定端口的应用通信
public Socket(InetAddress address, int port) throws IOException;
```

### Socket常用方法

```java
// 获取读取数据的通道
public InputStream getInputStream() throws IOException;
// 获取输出数据的通道
public OutputStream getOutputStream() throws IOException;
// 设置链接的超时时间，0表示不会超时
public synchronized void setSoTimeout(int timeout) throws SocketException;
// 标识输入通道不再接收数据，如果再次向通道中读取数据，则返回-1，表示读取到末尾
public void shutdownInput() throws IOException;
// 禁用输出通道，如果再次向通道中输入数据，则会报IOException
public void shutdownOutput() throws IOException;
// 关闭套接字，相关的数据通道都会被关闭
public synchronized void close() throws IOException;
```

### ServerSocket常用构造方法

```java
// 创建一个服务器套接字并占用给定的端口
public ServerSocket(int port) throws IOException;
```

### ServerSocket常用方法

```java
// 侦听与此套接字建立的连接并接受它。 该方法将阻塞，直到建立连接为止。
public Socket accept() throws IOException;
// 设置链接的超时时间，0表示不会超时
public synchronized void setSoTimeout(int timeout) throws SocketException;
```

示例

```java
import java.io.*;
import java.net.InetAddress;
import java.net.Socket;

/**
 * 网络工具类: 用于收/发信息
 */
public class NetUtil {
    public static StringBuilder recv(Socket socket) throws IOException {
        // 读取传输的消息
        InputStream inputStream = socket.getInputStream();
        InputStreamReader isr = new InputStreamReader(inputStream);
        BufferedReader buffer = new BufferedReader(isr);
        String line;
        StringBuilder res = new StringBuilder();
        while ((line = buffer.readLine()) != null) {
            res.append(line);
        }

        socket.shutdownInput(); // 一般不是这样关的!
        return res;
    }

    public static void send(Socket socket, String msg) throws IOException {
        // 获取socket的输出通道
        OutputStream outputStream = socket.getOutputStream();
        OutputStreamWriter osw = new OutputStreamWriter(outputStream);
        BufferedWriter bw = new BufferedWriter(osw);
        bw.write(msg);
        bw.flush(); // 强制将缓冲区中的数据刷新到输出流中, 即发送到客户端

        socket.shutdownOutput(); // 一般不是这样关的!
    }
}

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * 服务端
 */
public class SerNet {
    private ServerSocket serSocket;

    SerNet(int port) throws IOException {
        this.serSocket = new ServerSocket(port);
    }

    /**
     * 服务器启动
     */
    public void start() {
        while (true) {
            try {
                // 阻塞, 等待客户端连接
                System.out.println("阻塞等待接收");
                Socket cli_socket = this.serSocket.accept();
                for (int i = 0; i < 2; ++i) {
                    // 接收消息, 并且回声回去
                    StringBuilder recv = NetUtil.recv(cli_socket);
                    System.out.println("收到[客户端]消息: " + recv);
                    NetUtil.send(cli_socket, "114514" + recv);
                }
                // 关闭客户端
                cli_socket.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}

import java.io.IOException;
import java.net.Socket;

// 客户端
public class CliNet {
    private Socket socket;

    CliNet(String host, int port) throws IOException {
        this.socket = new Socket(host, port);
    }

    public void stact() {
        try {
            for (int i = 0; i < 2; ++i) {
                // 发送
                NetUtil.send(this.socket, "Hello My Ser Men!" + i);
                System.out.println("发送了消息: Hello My Ser Men!" + i);
                // 接收
                System.out.println("收到消息: " + NetUtil.recv(this.socket));
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        finally {
            try {
                this.socket.close();
                System.out.println("[info]: 客户端已关闭!");
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}

import java.io.IOException;

public class SerMain {
    public static void main(String[] args) {
        try {
            SerNet ser = new SerNet(6666);
            ser.start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

import java.io.IOException;

public class CliMain {
    public static void main(String[] args) {
        try {
            // 本机表示的方式: localhost 127.0.0.1
            CliNet cli = new CliNet("localhost", 6666);
            cli.stact();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
```

上面代码有问题!!! for 循环要改 i < 1 才可以运行!

不能多次对同一个套接字发送! 因为 `关流` 了

下面的可以!

```java
package xxx;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

public class SocketServer {
    public static void main(String[] args) throws IOException {

        // 端口号
        int port = 6666;
        // 在端口上创建一个服务器套接字
        ServerSocket serverSocket = new ServerSocket(port);
        // 监听来自客户端的连接
        Socket socket = serverSocket.accept();

        DataInputStream dis = new DataInputStream(
                new BufferedInputStream(socket.getInputStream()));

        DataOutputStream dos = new DataOutputStream(
                new BufferedOutputStream(socket.getOutputStream()));

        do {
            double length = dis.readDouble();
            System.out.println("服务器端收到的边长数据为：" + length);
            double result = length * length;
            dos.writeDouble(result);
            dos.flush();
        } while (dis.readInt() != 0);

        socket.close();
        serverSocket.close();
    }
}
```

```java
package xxx;

import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Scanner;

public class SocketClient {
    public static void main(String[] args) throws UnknownHostException, IOException, IOException {

        int port = 6666;
        String host = "localhost";

        // 创建一个套接字并将其连接到指定端口号
        Socket socket = new Socket(host, port);

        // 数据流
        DataInputStream dis = new DataInputStream(
                new BufferedInputStream(socket.getInputStream()));

        DataOutputStream dos = new DataOutputStream(
                new BufferedOutputStream(socket.getOutputStream()));

        Scanner sc = new Scanner(System.in);

        boolean flag = false;

        while (!flag) {

            System.out.println("请输入正方形的边长:");
            double length = sc.nextDouble();

            dos.writeDouble(length);
            dos.flush();

            double area = dis.readDouble();

            System.out.println("服务器返回的计算面积为:" + area);

            while (true) {

                System.out.println("继续计算？(Y/N)");

                String str = sc.next();

                if (str.equalsIgnoreCase("N")) {
                    dos.writeInt(0); // 通过发送标志0来断开连接
                    dos.flush();
                    flag = true;
                    break;
                } else if (str.equalsIgnoreCase("Y")) {
                    dos.writeInt(1);
                    dos.flush();
                    break;
                }
            }
        }
        socket.close();
    }
}
```