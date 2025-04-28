# 该死的windows
## 事发原因
场景, 我编写的 基于win api 的 iopc 的 类, (已经没问题并且很完善的)

在添加到我的项目中时 (一个 HieasyX库(win32)控件/绘图库), 出现了tm的连接属性错误:

```VS2022 DEBUG
严重性    代码    说明    项目    文件    行    禁止显示状态    详细信息
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    443        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    442        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    444        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    451        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    457        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    458        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    459        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    475        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    485        
错误(活动)    E0040    应输入标识符    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h    487        
错误(活动)    E0338    重载函数 "__WSAFDIsSet" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    141        
错误(活动)    E0338    重载函数 "accept" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1631        
错误(活动)    E0338    重载函数 "bind" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1653        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "closesocket" (已声明 所在行数:749，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1674        
错误(活动)    E0338    重载函数 "connect" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1691        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "ioctlsocket" (已声明 所在行数:756，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1712        
错误(活动)    E0338    重载函数 "getpeername" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1735        
错误(活动)    E0338    重载函数 "getsockname" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1756        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "getsockopt" (已声明 所在行数:771，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1777        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "htonl" (已声明 所在行数:778，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1802        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "htons" (已声明 所在行数:780，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1819        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "inet_addr" (已声明 所在行数:782，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1837        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "inet_ntoa" (已声明 所在行数:784，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1855        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "listen" (已声明 所在行数:786，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1955        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "ntohl" (已声明 所在行数:790，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1974        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "ntohs" (已声明 所在行数:792，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    1991        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "recv" (已声明 所在行数:794，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    2008        
错误(活动)    E0338    重载函数 "recvfrom" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    2031        
错误(活动)    E0338    重载函数 "select" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    2058        
错误(活动)    E1389    重新声明无法将 dllexport/dllimport 添加到 "send" (已声明 所在行数:815，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    2083        
错误(活动)    E0338    重载函数 "sendto" 的多个实例包含“C”链接    序列化    C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h    2106        
错误(活动)    E1389	重新声明无法将 dllexport/dllimport 添加到 "setsockopt" (已声明 所在行数:829，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2133		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "shutdown" (已声明 所在行数:836，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2158		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "socket" (已声明 所在行数:840，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2178		
错误(活动)	E0311	无法重载仅按返回类型区分的函数	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2203		
错误(活动)	E0311	无法重载仅按返回类型区分的函数	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2225		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "gethostname" (已声明 所在行数:854，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2242		
错误(活动)	E0311	无法重载仅按返回类型区分的函数	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2282		
错误(活动)	E0311	无法重载仅按返回类型区分的函数	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2301		
错误(活动)	E0311	无法重载仅按返回类型区分的函数	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2320		
错误(活动)	E0311	无法重载仅按返回类型区分的函数	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2337		
错误(活动)	E0338	重载函数 "WSAStartup" 的多个实例包含“C”链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2357		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSACleanup" (已声明 所在行数:876，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2377		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSASetLastError" (已声明 所在行数:878，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2394		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAGetLastError" (已声明 所在行数:880，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2411		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAIsBlocking" (已声明 所在行数:882，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2432		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAUnhookBlockingHook" (已声明 所在行数:884，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2450		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSASetBlockingHook" (已声明 所在行数:886，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2468		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSACancelBlockingCall" (已声明 所在行数:888，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2486		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncGetServByName" (已声明 所在行数:890，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2504		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncGetServByPort" (已声明 所在行数:898，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2532		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncGetProtoByName" (已声明 所在行数:906，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2560		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncGetProtoByNumber" (已声明 所在行数:913，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2586		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncGetHostByName" (已声明 所在行数:920，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2612		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncGetHostByAddr" (已声明 所在行数:927，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2638		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSACancelAsyncRequest" (已声明 所在行数:936，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2668		
错误(活动)	E1389	重新声明无法将 dllexport/dllimport 添加到 "WSAAsyncSelect" (已声明 所在行数:938，所属文件:"C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\winsock.h")	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2686		
错误	C2011	“sockaddr”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	240		
错误	C2143	语法错误: 缺少“}”(在“常数”的前面)	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	442		
错误	C2059	语法错误:“常数”	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	442		
错误	C2143	语法错误: 缺少“;”(在“}”的前面)	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	496		
错误	C4430	缺少类型说明符 - 假定为 int。注意: C++ 不支持默认 int	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	496		
错误	C4430	缺少类型说明符 - 假定为 int。注意: C++ 不支持默认 int	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	496		
错误	C2011	“sockaddr_in”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\ws2def.h	638		
错误	C2011	“fd_set”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	136		
错误	C2011	“timeval”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	180		
错误	C2011	“hostent”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	236		
错误	C2011	“netent”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	249		
错误	C2011	“servent”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	256		
错误	C2011	“protoent”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	268		
错误	C2011	“WSAData”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	364		
错误	C2011	“sockproto”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	462		
错误	C2011	“linger”:“struct”类型重定义	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	504		
错误	C2375	“accept”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1631		
错误	C2375	“bind”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1653		
错误	C2375	“closesocket”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1674		
错误	C2375	“connect”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1691		
错误	C2375	“ioctlsocket”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1712		
错误	C2375	“getpeername”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1735		
错误	C2375	“getsockname”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1756		
错误	C2375	“getsockopt”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1777		
错误	C2375	“htonl”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1802		
错误	C2375	“htons”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1819		
错误	C2375	“inet_addr”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1837		
错误	C2375	“inet_ntoa”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1855		
错误	C2375	“listen”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1955		
错误	C2375	“ntohl”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1974		
错误	C2375	“ntohs”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	1991		
错误	C2375	“recv”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2008		
错误	C2375	“recvfrom”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2031		
错误	C2375	“select”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2058		
错误	C2375	“send”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2083		
错误	C2375	“sendto”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2106		
错误	C2375	“setsockopt”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2133		
错误	C2375	“shutdown”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2158		
错误	C2375	“socket”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2178		
错误	C2375	“gethostbyaddr”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2203		
错误	C2375	“gethostbyname”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2225		
错误	C2375	“gethostname”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2242		
错误	C2375	“getservbyport”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2282		
错误	C2375	“getservbyname”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2301		
错误	C2375	“getprotobynumber”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2320		
错误	C2375	“getprotobyname”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2337		
错误	C2375	“WSAStartup”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2357		
错误	C2375	“WSACleanup”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2377		
错误	C2375	“WSASetLastError”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2394		
错误	C2375	“WSAGetLastError”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2411		
错误	C2375	“WSAIsBlocking”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2432		
错误	C2375	“WSAUnhookBlockingHook”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2450		
错误	C2375	“WSASetBlockingHook”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2468		
错误	C2375	“WSACancelBlockingCall”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2486		
错误	C2375	“WSAAsyncGetServByName”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2504		
错误	C2375	“WSAAsyncGetServByPort”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2532		
错误	C2375	“WSAAsyncGetProtoByName”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2560		
错误	C2375	“WSAAsyncGetProtoByNumber”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2586		
错误	C2375	“WSAAsyncGetHostByName”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2612		
错误	C2375	“WSAAsyncGetHostByAddr”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2638		
错误	C2375	“WSACancelAsyncRequest”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2668		
错误	C2375	“WSAAsyncSelect”: 重定义；不同的链接	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	2686		
错误	C2059	语法错误:“}”	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	4318		
错误	C2143	语法错误: 缺少“;”(在“}”的前面)	序列化	C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\um\WinSock2.h	4318		
错误	C2143	语法错误: 缺少“;”(在“{”的前面)	序列化	D:\command\cc++\C++\序列化\序列化\main.cpp	9		
错误	C2447	“{”: 缺少函数标题(是否是老式的形式表?)	序列化	D:\command\cc++\C++\序列化\序列化\main.cpp	9		
```

查阅网上, 众人云之, `#define WIN32_LEAN_AND_MEAN`置于`#include <Windows.h>`之前则ojbk, 我套你猴子的, 还有什么 于`右击项目>>属性>>配置属性>>C/C++>>预处理器，添加WIN32_LEAN_AND_MEAN`(什么pl还用`》》`写的?)则妙手也, 我dnmd, 还是不行~~...~~

## 排查
新建一个项目, 单独运行 我的iocp, 问题滴大大滴妹有!~

*(我的头文件:)*
```C++
#include <cstdio>
#include <WinSock2.h>
#include <MSWSock.h>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")
```

把`HieasyX`库 #include, 马上报错!

只包含在项目, 不#include, 也tm报错, 这就奇了怪了, 我按照提示, 把有问题的文件(3个)统统排除于项目, 然后就运行成功了

然后我就一个一个的把有关文件的.cpp/.h的#include都写的我的iocp文件里面, 不断尝试下发现:

## 结果

```C++
#include <graphics.h> // (单独也)导入后出错
#include <WinUser.h>  // (单独也)导入后出错
```


分析源码后发现:

```C++
#include <graphics.h>
 |
 +-- #include <easyx.h>
      |
      +-- #include <windows.h> // 没错就是他
```

本身`如下`是没有问题的

```C++
#include <windows.h>
#include <windows.h>
```

但是好巧不巧, 这样就报错了

```C++
#include <windows.h> // 添加于此

#include <cstdio>
#include <WinSock2.h>
#include <MSWSock.h>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")
```

为什么呢? 原来这就是众说纷纭的那个原因

```C++
// --- 如下是报错的 ---
#include <windows.h>
#include <WinSock2.h>

// --- 如下是没有问题的 ---
#include <WinSock2.h>
#include <windows.h>
```

## 解决

写到下面去就没问题了 =-=

```C++
#include <WinSock2.h>
#include <MSWSock.h>

#include <Windows.h>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")

// --- 真下头 ---
#include <graphics.h>
#include <WinUser.h>
```

# 梅开二度

把头文件都这么写后:

```C++
#pragma once
#ifndef _DE_BUG_H_
#define _DE_BUG_H_

#include <WinSock2.h>
#include <MSWSock.h>
#include <Windows.h>
// --- 真下头 ---
#include <graphics.h>
#include <windowsx.h>
#include <WinUser.h>
#include <vector>
#include <list>
#include <string>
#include <gdiplus.h>
#include <ctime>
#include <thread>
#include <stdio.h>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Mswsock.lib")

#endif
```

~~并且修改了后, 发现还是tm的不行, 但是排除了控件库又tm的可以...~~(好像有个地方删除了但是忘记导入我的万能头文件了...)

基本上是解决了！！！