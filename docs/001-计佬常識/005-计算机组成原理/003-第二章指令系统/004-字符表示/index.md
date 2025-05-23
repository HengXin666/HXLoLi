# 字符表示
> 发明计算机是为了数字计算，不过计算机很快被用于商业方面的文字处理。

**ASCII(American Standard Code for Information Interchange，美国信息交换标准代码)** 是一种早期广泛采用的字符编码标准，它使用7位二进制数（一个字节的最低7位）来表示128种不同的字符，包括英文大小写字母、数字、标点符号和其他控制字符。后来，为了满足更多的字符需求，出现了扩展ASCII和其他编码方案，如ISO-8859系列和Unicode等，它们可以利用8位或更多位来表示更广泛的字符集，包括非拉丁字母、特殊符号和表情符等。

现代计算机系统普遍使用8位作为基本的数据单位——字节，并在此基础上通过组合多个字节形成更大的数据类型（如16位的短整型、32位的整型和64位的长整型），以适应各种复杂的应用场景。而如今在文本处理领域，Unicode编码已经成为主流标准，能够支持世界上几乎所有的书写系统和数千种字符。

字符通常被组合为字符数目可变的字符串。表示一个字符串的方式有三种选择:

1. **长度前置**: 在字符串的第一个位置存储字符串的长度。这种方式允许直接通过第一个字节或几个字节快速获取字符串长度，不需要遍历整个字符串来计算长度。

2. **附加长度变量**: 在字符串数据之后额外存储一个变量（通常是整数类型），用来记录字符串的实际长度。

3. **使用特定字符作为结束符**: 如C语言中采用的方式，即在字符串的最后一个有效字符后面添加一个特殊的字符（通常是ASCII码值为0的空字符`'\0'`）来标识字符串的结尾。

每种方法都有其优缺点，选择哪种方式取决于具体的应用场景、性能需求以及内存管理策略等因素。
