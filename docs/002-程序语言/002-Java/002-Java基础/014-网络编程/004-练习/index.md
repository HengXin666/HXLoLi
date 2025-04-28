# 练习
使用网络通信完成注册和登录功能。要求注册的用户信息必须进行存档，登录时需要从存档的数据检测是否能够登录。

## 分析
1. 使用TCP完成，因为TCP是可靠性数据传输，可以保证信息能够被接收
2. 用户属于客户端操作，服务器端需要区分用户的行为
3. 为了区分客户端的行为，需要设计一个消息类，然后使用序列化的方式来进行传输
4. 用户注册信息需要存档，因此需要设计一个用户类，为了方便使用，也可以直接将一个集合使用序列化的方式存储在文档中


## 代码

没有做任何的输入限制处理! 很容易有八嘎!

```java
import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

// 数据库类, 用于读取账号组和写入账号组
class DemoDB {
    private static final File dbFile = new File("./dp.txt"); // 数据库位置

    // init
    static {
        // 如果目录不存在，则创建目录
        File parentDir = DemoDB.dbFile.getParentFile();
        if (!parentDir.exists()) {
            parentDir.mkdirs();
        }

        // 不存在则创建文件
        if (!DemoDB.dbFile.exists()) {
            try {
                DemoDB.dbFile.createNewFile();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }

    // 读取数据库内容
    public static HashMap<String, String> getDBData() {
        // 存放数据格式: 账号|密码
        HashMap<String, String> map = new HashMap<>();
        try (BufferedReader br = new BufferedReader(new FileReader(DemoDB.dbFile))) {
            String str;
            while ((str = br.readLine()) != null) {
                String[] tmp = str.split("\\|");
                map.put(tmp[0], tmp[1]);
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        return map;
    }

    // 更新数据库内容
    public static void updateDBData(HashMap<String, String> map) {
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(DemoDB.dbFile, false))) { // 覆盖
            for (Map.Entry<String, String> entry : map.entrySet()) {
                String s1 = entry.getKey();
                String s2 = entry.getValue();
                bw.write(s1 + "|" + s2);
                bw.newLine();
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

/**
 * 网络工具类: 用于收/发信息
 */
class DemoNetUtil {
    public static String recv(Socket socket) throws IOException {
        // 读取传输的消息
        InputStream inputStream = socket.getInputStream();
        InputStreamReader isr = new InputStreamReader(inputStream);
        BufferedReader buffer = new BufferedReader(isr);

        return buffer.readLine();
    }

    public static void send(Socket socket, String msg) throws IOException {
        // 获取socket的输出通道
        OutputStream outputStream = socket.getOutputStream();
        OutputStreamWriter osw = new OutputStreamWriter(outputStream);
        BufferedWriter bw = new BufferedWriter(osw);
        bw.write(msg);
        bw.flush(); // 强制将缓冲区中的数据刷新到输出流中, 即发送到客户端
    }
}

/**
 * 服务端
 */
class DemoSerNet {
    private ServerSocket serSocket;

    DemoSerNet(int port) throws IOException {
        this.serSocket = new ServerSocket(port);
    }

    /**
     * 服务器启动
     */
    public void start() {
        while (true) {
            HashMap<String, String> dbData = DemoDB.getDBData();
            try {
                // 阻塞, 等待客户端连接
                System.out.println("阻塞等待接收");
                Socket cli_socket = this.serSocket.accept();
                System.out.println("有人连接了!");
                boolean tag = true;
                /**
                 * 通讯暗号: 类型#正文
                 * 类型是单个字符: [服务端接收]
                 *              1 注册 --> 1#Heng_Xin|loli114514
                 *              2 登录 --> 2#账号|密码
                 *              3 终止 --> 3#
                 *
                 *              [客户端接收]
                 *              4 登录成功 --> 4#内容
                 *              5 登录失败 --> 5#内容
                 *              6 注册成功 --> 6#内容
                 *              7 注册失败 --> 7#内容
                 */
                do {
                    String recvStr = DemoNetUtil.recv(cli_socket);
                    String[] data = recvStr.split("#");
                    switch (data[0].charAt(0)) {
                        case '1': {
                            // 注册
                            String[] user = data[1].split("\\|");
                            System.out.printf("有人注册了账号[ %s ], 偷偷告诉你密码是: { *%s* }\n", user[0], user[1]);
                            if (dbData.containsKey(user[0])) {
                                // 存在重复的键, 用户名已经被占用了!
                                String error = "[ERROR]: 这个账号名已经存在!";
                                System.out.println(error);
                                DemoNetUtil.send(cli_socket, "7#" + error + "\n");
                            } else {
                                dbData.put(user[0], user[1]);
                                DemoDB.updateDBData(dbData);
                                String ok = "[INFO]: 注册成功!";
                                System.out.println(ok);
                                DemoNetUtil.send(cli_socket, "6#" + ok + "\n");
                            }
                            break;
                        }
                        case '2': {
                            String[] user = data[1].split("\\|");
                            System.out.printf("有人尝试登录账号[ %s ], 偷偷告诉你密码是: { *%s* }\n", user[0], user[1]);
                            if (dbData.containsKey(user[0]) && dbData.get(user[0]).equals(user[1])) {
                                String ok = "[INFO]: 登录成功!";
                                System.out.println(ok);
                                DemoNetUtil.send(cli_socket, "4#" + ok + "\n");
                            } else {
                                String error = "[ERROR]: 账号或密码错误!";
                                System.out.println(error);
                                DemoNetUtil.send(cli_socket, "5#" + error + "\n");
                            }
                            break;
                        }
                        case '3': {
                            System.out.println("[INFO]: 下线了一个老哥~");
                            tag = false;
                            break;
                        }
                        default: {
                            System.err.println("接收到错误的暗号!");
                            break;
                        }
                    }
                } while (tag);
                cli_socket.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}

class DemoCliNet {
    private Socket socket;
    private Scanner sc = new Scanner(System.in);

    DemoCliNet(String host, int port) throws IOException {
        this.socket = new Socket(host, port);
    }

    private String loing() {
        String res;
        System.out.print("请输入账号: ");
        res = sc.next() + "|";
        System.out.print("请输入密码: ");
        res += sc.next();
        return res;
    }

    public void stact() {
        try {
            boolean tag = true;
            boolean state = false; // 当前状态: 是否登录
            ALL: do {
                if (!state)
                    System.out.println("{当前状态: 未登录}请输入数字: 1[注册], 2[登录], 3[退出]");
                else
                    System.out.println("{当前状态: 已登录}请输入数字: 3[退出]");
                int choice = sc.nextInt();
                switch (choice) {
                    case 1: {
                        // 注册
                        System.out.println("=== 注 册 ===");
                        String data = this.loing();
                        DemoNetUtil.send(this.socket, "1#" + data + "\n");
                        break;
                    }
                    case 2: {
                        // 登录
                        System.out.println("=== 登 录 ===");
                        String data = this.loing();
                        DemoNetUtil.send(this.socket, "2#" + data + "\n");
                        break;
                    }
                    case 3: {
                        DemoNetUtil.send(this.socket, "3#退出" + "\n");
                        System.out.println("我直接退出!");
                        break ALL;
                    }
                    default: {
                        System.err.println("接收到错误的暗号!");
                        break;
                    }
                }

                String recv = DemoNetUtil.recv(this.socket);
                String[] data = recv.split("#");
                if (data.length >= 1)
                    System.out.println(data[1]);
                switch (data[0].charAt(0)) {
                    case '4': { // 登录成功
                        state = true;
                        break;
                    }
                    case '5': { // 登录失败
                        break;
                    }
                    case '6': { // 注册成功
                        break;
                    }
                    case '7': { // 注册失败
                        break;
                    }
                    default: {
                        System.out.println("内部错误!");
                        break;
                    }
                }
            } while (tag);
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

// 制作一个网络通信完成注册和登录功能。要求注册的用户信息必须进行存档，登录时需要从存档的数据检测是否能够登录。

public class Demo {
    public static void main(String[] args) {
        // 服务端
        try {
            DemoSerNet ser = new DemoSerNet(6666);
            ser.start();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}

import java.io.IOException;

public class DemoCliMain {
    public static void main(String[] args) {
        try {
            DemoCliNet cli = new DemoCliNet("127.0.0.1", 6666);
            cli.stact();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
```

这里回复一下[[套接字Socket](../002-套接字Socket/index.md)]中`示例1`的八嘎(*BUG*)

为什么不能接受, 因为`.readLine()`是读取一行, 即需要`\n`换行符, 其次还是没有结束标志, 因为没有完的概念, 但是传输实际上可以看成是传输一行, 所以读取一次即可, 如果还是使用`while`会没有终止!