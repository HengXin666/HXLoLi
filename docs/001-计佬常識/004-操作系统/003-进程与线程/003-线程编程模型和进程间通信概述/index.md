# 线程编程模型和进程间通信概述
## 0. 进程线程的区别 (面试)

应该从`调度能力`上来说:

- 进程: 资源独立
- 线程: 资源共享

调度体通信

- 进程间通信
- 线程间通信

## 1. 线程编程模型 & 实现
### 1.1 线程系统调用

进程在其生命周期中，通常从一个初始的单线程启动，该线程作为进程的主执行流。随后，通过调用特定的库函数（例如`thread_create`），该主线程可以创建新的线程来执行并发任务。在`thread_create`函数的调用过程中，通常需要指定新线程的某些属性，包括一个用于标识新线程的名称（尽管在实践中，线程标识符与线程名称可能有所不同，但此处为简化说明，我们假设它们直接相关）。调用成功后，`thread_create`会返回一个唯一的线程标识符，该标识符通常用于后续对线程的引用和管理，而非直接作为线程的名字使用。

线程在完成其预定的任务后，应调用相应的退出函数（如`thread_exit`）来优雅地结束其执行。一旦线程退出，其状态将转变为终止，此后该线程将不再参与系统的调度，其占用的资源也会被回收。

在多线程环境中，线程间的同步和协调是重要的考虑因素。`thread_join`函数便是一种常用的同步机制，它允许一个线程（称为等待线程）暂停执行，直至另一个特定线程（被等待线程）退出。这种机制有效地实现了线程间的依赖管理和资源回收，确保了程序的正确性和稳定性。通过`thread_join`，线程的创建和终止过程在逻辑上与进程的创建和终止过程存在诸多相似之处，都涉及到了资源的申请、使用和释放。

线程同步是指在多线程环境下，通过一定的机制来协调各个线程的执行顺序，以保证它们能够按照一定的规则来访问共享资源或执行特定的任务。同步的目的是确保数据的一致性和程序的正确性。

此外，为了提高系统的并发性和响应性，线程之间需要合理地共享CPU资源。`thread_yield`函数提供了一种机制，允许当前运行的线程主动放弃CPU，将执行机会让给同一进程内的其他线程或系统内的其他进程。这一机制在需要实现公平调度或避免饥饿时尤为重要，因为与进程不同，线程通常不会因时钟中断而自动放弃CPU（尽管这取决于具体的操作系统和调度策略）。通过`thread_yield`的调用，开发者可以更精细地控制线程的执行顺序和优先级，从而提升程序的性能和用户体验。

### 1.2 实现

- 直接看: [线程相关函数](https://blog.HXLoLi.com/blog/#/articles?articleId=20305 "##20305##") (POSIX线程(简称pthreads)

- 然后需要注意线程同步(线程通信) [线程同步序言](https://blog.HXLoLi.com/blog/#/articles?articleId=20308 "##20308##")

比如:

```C++
int v = 0;

void threadTask() {
    for (int i = 0; i < 1e4; ++i)
          ++v;
}

// 创建两个子线程
runThreadTask(threadTask);
runThreadTask(threadTask);

std::print("v = ", v); // v 的取值为 [2, 2e4]
```

为什么 $v \in [2, 2 \times 10^4]$ 呢?

你想想, `++v`实际上是几个操作?

```arm
;; 伪汇编

;; 1. 去 v 的值到寄存器 r0
get r0 v

;; 2. v0 + 1 存储到 r0
add r0 r0 1

;; 3. 从寄存器写入回变量
set r0 v
```

显然这三条指令之间任何时刻是不是都可以被`时间片`分割

结果为 $0$ 的情况:

- 例如:
    - 当线程 $t_1$ 处于`1.`状态时候, 时间片来了 (此时 $v = 0$ )
    - 此时 $t_2$ 获取了 $v$, 执行了多次`++v`了 (比如结束的时候, $v = 9999$ 了)
    - 终于回到了 $t_1$, 此时自增后写入, (此时 $v$ 从 $9999$ 变为了 $1$ 了)
    - 然后 $t_2$ 获取了 $v = 1$, 同时时间片又来了
    - 这次轮到 $t_1$ 把剩下 $9999$ 次加完, 然后 $t_1$ 线程结束
    - 而此时剩下的 $t_2$ 就把 $v + 1 \to v$, 此时 即为最小值: $2$

最大值就是最理想的互不干扰模式.

剩下的就是这两种情况的交叉了.

### 1.3 strace 跟踪进程使用的syscall

线程的:

```C++
 root@Loli  ~/HXcode/zero-start strace ./app
execve("./app", ["./app"], 0x7ffca8065100 /* 41 vars */) = 0
brk(NULL)                               = 0x64c77dc48000
access("/etc/ld.so.preload", R_OK)      = -1 ENOENT (No such file or directory)
openat(AT_FDCWD, "/etc/ld.so.cache", O_RDONLY|O_CLOEXEC) = 3
fstat(3, {st_mode=S_IFREG|0644, st_size=28871, ...}) = 0
mmap(NULL, 28871, PROT_READ, MAP_PRIVATE, 3, 0) = 0x7c5364754000
close(3)                                = 0
openat(AT_FDCWD, "/usr/lib/libc.so.6", O_RDONLY|O_CLOEXEC) = 3
read(3, "\177ELF\2\1\1\3\0\0\0\0\0\0\0\0\3\0>\0\1\0\0\0\340_\2\0\0\0\0\0"..., 832) = 832
pread64(3, "\6\0\0\0\4\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0"..., 784, 64) = 784
fstat(3, {st_mode=S_IFREG|0755, st_size=2014520, ...}) = 0
mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) = 0x7c5364752000
pread64(3, "\6\0\0\0\4\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0"..., 784, 64) = 784
mmap(NULL, 2034616, PROT_READ, MAP_PRIVATE|MAP_DENYWRITE, 3, 0) = 0x7c5364561000
mmap(0x7c5364585000, 1511424, PROT_READ|PROT_EXEC, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, 3, 0x24000) = 0x7c5364585000
mmap(0x7c53646f6000, 319488, PROT_READ, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, 3, 0x195000) = 0x7c53646f6000
mmap(0x7c5364744000, 24576, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, 3, 0x1e3000) = 0x7c5364744000
mmap(0x7c536474a000, 31672, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_FIXED|MAP_ANONYMOUS, -1, 0) = 0x7c536474a000
close(3)                                = 0
mmap(NULL, 12288, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) = 0x7c536455e000
arch_prctl(ARCH_SET_FS, 0x7c536455e740) = 0
set_tid_address(0x7c536455ea10)         = 3490
set_robust_list(0x7c536455ea20, 24)     = 0
rseq(0x7c536455f060, 0x20, 0, 0x53053053) = 0
mprotect(0x7c5364744000, 16384, PROT_READ) = 0
mprotect(0x64c779097000, 4096, PROT_READ) = 0
mprotect(0x7c5364796000, 8192, PROT_READ) = 0
prlimit64(0, RLIMIT_STACK, NULL, {rlim_cur=8192*1024, rlim_max=RLIM64_INFINITY}) = 0
munmap(0x7c5364754000, 28871)           = 0
rt_sigaction(SIGRT_1, {sa_handler=0x7c53645f22b0, sa_mask=[], sa_flags=SA_RESTORER|SA_ONSTACK|SA_RESTART|SA_SIGINFO, sa_restorer=0x7c536459e1d0}, NULL, 8) = 0
rt_sigprocmask(SIG_UNBLOCK, [RTMIN RT_1], NULL, 8) = 0
mmap(NULL, 8392704, PROT_NONE, MAP_PRIVATE|MAP_ANONYMOUS|MAP_STACK, -1, 0) = 0x7c5363c00000
mprotect(0x7c5363c01000, 8388608, PROT_READ|PROT_WRITE) = 0
getrandom("\x83\xbd\xe6\x44\x97\xb7\x6c\x42", 8, GRND_NONBLOCK) = 8
brk(NULL)                               = 0x64c77dc48000
brk(0x64c77dc69000)                     = 0x64c77dc69000
rt_sigprocmask(SIG_BLOCK, ~[], [], 8)   = 0
clone3({flags=CLONE_VM|CLONE_FS|CLONE_FILES|CLONE_SIGHAND|CLONE_THREAD|CLONE_SYSVSEM|CLONE_SETTLS|CLONE_PARENT_SETTID|CLONE_CHILD_CLEARTID, child_tid=0x7c5364400990, parent_tid=0x7c5364400990, exit_signal=0, stack=0x7c5363c00000, stack_size=0x7fff80, tls=0x7c53644006c0} => {parent_tid=[3491]}, 88) = 3491
[子线程]: 0
[子线程]: 1
[子线程]: 2
[子线程]: 3
[子线程]: 4

子线程id: 136697606047424
rt_sigprocmask(SIG_SETMASK, [], NULL, 8) = 0
write(1, "\345\255\220\347\272\277\347\250\213\345\210\233\345\273\272\346\210\220\345\212\237: id \344\270\272 13"..., 46子线程创建成功: id 为 136697606047424
) = 46
write(1, "[\347\210\266\347\272\277\347\250\213]: 0\n", 15[父线程]: 0
) = 15
write(1, "[\347\210\266\347\272\277\347\250\213]: 1\n", 15[父线程]: 1
) = 15
write(1, "[\347\210\266\347\272\277\347\250\213]: 2\n", 15[父线程]: 2
) = 15
write(1, "[\347\210\266\347\272\277\347\250\213]: 3\n", 15[父线程]: 3
) = 15
write(1, "[\347\210\266\347\272\277\347\250\213]: 4\n", 15[父线程]: 4
) = 15
write(1, "\n", 1
)                       = 1
write(1, "\347\210\266\347\272\277\347\250\213id: 136697607481152\n", 29父线程id: 136697607481152
) = 29
exit_group(0)                           = ?
+++ exited with 0 +++
```

进程的:

```C++
 root@Loli  ~/HXcode/zero-start strace ./app
execve("./app", ["./app"], 0x7ffc3130b340 /* 41 vars */) = 0
brk(NULL)                               = 0x5745a6ef8000
access("/etc/ld.so.preload", R_OK)      = -1 ENOENT (No such file or directory)
openat(AT_FDCWD, "/etc/ld.so.cache", O_RDONLY|O_CLOEXEC) = 3
fstat(3, {st_mode=S_IFREG|0644, st_size=28871, ...}) = 0
mmap(NULL, 28871, PROT_READ, MAP_PRIVATE, 3, 0) = 0x787a1c35d000
close(3)                                = 0
openat(AT_FDCWD, "/usr/lib/libc.so.6", O_RDONLY|O_CLOEXEC) = 3
read(3, "\177ELF\2\1\1\3\0\0\0\0\0\0\0\0\3\0>\0\1\0\0\0\340_\2\0\0\0\0\0"..., 832) = 832
pread64(3, "\6\0\0\0\4\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0"..., 784, 64) = 784
fstat(3, {st_mode=S_IFREG|0755, st_size=2014520, ...}) = 0
mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) = 0x787a1c35b000
pread64(3, "\6\0\0\0\4\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0@\0\0\0\0\0\0\0"..., 784, 64) = 784
mmap(NULL, 2034616, PROT_READ, MAP_PRIVATE|MAP_DENYWRITE, 3, 0) = 0x787a1c16a000
mmap(0x787a1c18e000, 1511424, PROT_READ|PROT_EXEC, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, 3, 0x24000) = 0x787a1c18e000
mmap(0x787a1c2ff000, 319488, PROT_READ, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, 3, 0x195000) = 0x787a1c2ff000
mmap(0x787a1c34d000, 24576, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, 3, 0x1e3000) = 0x787a1c34d000
mmap(0x787a1c353000, 31672, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_FIXED|MAP_ANONYMOUS, -1, 0) = 0x787a1c353000
close(3)                                = 0
mmap(NULL, 12288, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) = 0x787a1c167000
arch_prctl(ARCH_SET_FS, 0x787a1c167740) = 0
set_tid_address(0x787a1c167a10)         = 3621
set_robust_list(0x787a1c167a20, 24)     = 0
rseq(0x787a1c168060, 0x20, 0, 0x53053053) = 0
mprotect(0x787a1c34d000, 16384, PROT_READ) = 0
mprotect(0x5745783f3000, 4096, PROT_READ) = 0
mprotect(0x787a1c39f000, 8192, PROT_READ) = 0
prlimit64(0, RLIMIT_STACK, NULL, {rlim_cur=8192*1024, rlim_max=RLIM64_INFINITY}) = 0
munmap(0x787a1c35d000, 28871)           = 0
fstat(1, {st_mode=S_IFCHR|0620, st_rdev=makedev(0x88, 0), ...}) = 0
getrandom("\x5f\x60\xd0\x8a\xbc\x96\xbb\x35", 8, GRND_NONBLOCK) = 8
brk(NULL)                               = 0x5745a6ef8000
brk(0x5745a6f19000)                     = 0x5745a6f19000
write(1, "fork test...\n", 13fork test...
)          = 13
clone(child_stack=NULL, flags=CLONE_CHILD_CLEARTID|CLONE_CHILD_SETTID|SIGCHLD, child_tidptr=0x787a1c167a10) = 3622
getpid(子进程 3622
)                                = 3621
--- SIGCHLD {si_signo=SIGCHLD, si_code=CLD_EXITED, si_pid=3622, si_uid=0, si_status=0, si_utime=0, si_stime=0} ---
write(1, "\347\210\266\350\277\233\347\250\213 3621\n", 15父进程 3621
) = 15
exit_group(0)                           = ?
+++ exited with 0 +++
```

可以看到:

```C++
// 线程
clone3({flags=CLONE_VM|CLONE_FS|CLONE_FILES|CLONE_SIGHAND|CLONE_THREAD|CLONE_SYSVSEM|CLONE_SETTLS|CLONE_PARENT_SETTID|CLONE_CHILD_CLEARTID, child_tid=0x7c5364400990, parent_tid=0x7c5364400990, exit_signal=0, stack=0x7c5363c00000, stack_size=0x7fff80, tls=0x7c53644006c0} => {parent_tid=[3491]}, 88) = 3491

// 进程
clone(child_stack=NULL, flags=CLONE_CHILD_CLEARTID|CLONE_CHILD_SETTID|SIGCHLD, child_tidptr=0x787a1c167a10) = 3622
```

可以观察到, 线程分配资源的(系统调用`clone`)的时候, 其`stack`是有 **地址** 的! 也就是说, 线程需要用已经存在的栈空间作为自己的栈 (即线程是共享栈空间(共享虚拟内存))

而进程则是`child_stack=NULL`, 不指定栈空间. 这是什么意思? 我们知道进程肯定是需要有一片内存空间的, 你不指定给我, 那只能我自己申请了, 所以进程就新开辟了一块独属于它的内存空间!

从这个现象, 你也可以观察出`# 0.`的内容

## 2. 进程间通信
进程间的交互是系统运作中不可或缺的一环，特别是在复杂的应用场景中，如shell管道操作，其中第一个进程的输出数据必须准确无误地传递给第二个进程，并依此类推，确保数据流的连续性。这种需求凸显了进程间通信（Inter-Process Communication, IPC）机制的重要性，它要求采用一种稳定且不易被中断的数据结构来支撑进程间的信息传递。

关于进程间通信，我们主要面临以下三大挑战：
1. **消息传递机制**：首要问题是如何有效地将一个进程的信息或数据传递给另一个进程。这要求设计并实现一种可靠的通信协议或机制，确保数据能够准确无误地从源进程传输到目标进程。

2. **同步与互斥**：第二个挑战在于如何确保多个进程（或线程）在访问共享资源时不会相互干扰，从而避免数据竞争和不一致性。以航空公司售票系统为例，当两家航空公司同时尝试为不同顾客预订同一航班的最后一个座位时，必须有一种机制来确保只有一个操作能够成功，避免超卖现象。

3. **数据顺序与同步**：第三个问题是关于数据处理的顺序性。在分布式系统或并发环境中，确保数据按照正确的顺序被处理至关重要。例如，在进程A生成数据而进程B负责打印的场景中，必须保证进程B在打印之前已经接收到来自进程A的全部数据，且这些数据按照正确的顺序排列。

值得注意的是，上述后两个问题不仅限于进程间，同样适用于线程间的通信与协作。尽管线程间通信在某些方面（如共享内存访问）上更为直接和高效，因为它们共享同一地址空间，但这并不意味着线程间的同步与互斥问题可以忽视。相反，由于线程间的高耦合性，这些问题可能更加复杂和难以调试。

对于线程间的通信，虽然它们共享运行时环境，使得数据交换看起来更为直接，但正确管理共享资源、避免死锁和竞态条件仍然是开发者需要面对的重要挑战。我们将在后续讨论中深入探讨这些问题及其解决方案，以期为读者提供一个全面而深入的理解。