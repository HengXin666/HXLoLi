# 修改数据包发送速率的限制
新安装的客户端自动合成+快捷键mod (itemscroller), 一使用, 在某些情况下会被服务器封..

一顿排查, 发现是`viafabric`有发包限制

在`viafabric`的config文件里有一个`tracking-period`和`max-pps`。把这两个设置成`-1`就好了