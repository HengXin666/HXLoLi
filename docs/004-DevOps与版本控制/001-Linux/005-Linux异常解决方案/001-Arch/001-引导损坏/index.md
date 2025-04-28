# 引导损坏

> grub引导不小心被重新生成了... 然后只有win, linux不见了...
>
> (大家不要听信ai的鬼话, 然后就瞎鸡巴cv命令...🌿)

解决方案:

使用U盘重新烧录arch镜像

然后: [重建linux系统的grub启动项](https://blog.csdn.net/qq_66594410/article/details/129346611) 即:

从u盘启动, 使用`Arch Install`后

```sh
fdisk -l

# 如果太长了, 可以
fdisk -l | head -n 20
```

然后依次, 挂载 (请找到对应)

```sh
mount /dev/nvme0n1p8 /mnt # 挂载根分区，第二个参数/mnt可以自己命名。
mount /dev/nvme0n1p1 /mnt/boot # 将Windows的EFI分区挂载到根分区的boot目录下，保证根分区名称对应。
arch-chroot /mnt # 切换到根分区。意味着之后'/mnt'就是你的'/'路径，访问'/'就等于访问'/dev/nvme0n1p8'。输入exit可以退出。
```

安装linux内核

```sh
pacman -S linux
```

其中`bootloader-id`是名称

```sh
grub-install --target=x86_64-efi --efi-directory=/boot --bootloader-id=arch_grub
```

生成引导

```sh
grub-mkconfig -o /boot/grub/grub.cfg
```

然后`exit`+`reboot`即可.

---

如果重建后没有win的引导, 可以看看: [Arch Linux与Win11双系统修复grub引导](https://bestoko.cc/p/grubfix/) 即:

(不需要u盘, 在arch上即可操作)

```sh
sudo pacman -S os-prober
```

然后

```sh
sudo grub-mkconfig -o /boot/grub/grub.cfg
```

`os-prober`会自动识别到`win`的引导.