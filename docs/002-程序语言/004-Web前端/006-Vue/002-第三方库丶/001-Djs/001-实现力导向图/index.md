# 使用 D3.js 实现力导向图

## 安装

```sh
npm install d3
```

## 代码
```html
<template>
  <div>
    <svg id="topology" width="1000" height="600" style="background-color: rgb(24, 24, 24)"></svg>
  </div>
</template>

<script>
import * as d3 from 'd3';

export default {
  name: "topology",
  data() {
    return {
      nodes: [
        { id: 1, name: 'Node 1' },
        { id: 2, name: 'Node 2' },
        { id: 3, name: 'Node 3' },
        { id: 4, name: 'Node 4' },
        { id: 5, name: 'Node 5' }
      ],
      links: [
        { source: 1, target: 2 },
        { source: 1, target: 3 },
        { source: 1, target: 4 },
        { source: 5, target: 4 },
      ],
    };
  },
  mounted() {
    const svg = d3.select('#topology');
    const width = svg.attr('width');
    const height = svg.attr('height');

     /**
     * forceLink这里要处理一下，绑定node.id。不然会按照node的索引来进行，这样设置tick的时候会非常不方便
     * 仅进行力模拟结点的位置不会进行实时更新，必须要有simulation.on(tick)才会把力模拟的结果反映到图元上
     */
    const simulation = d3.forceSimulation(this.nodes)
      .force('link', d3.forceLink(this.links).id(d => d.id).distance(150))
      .force('charge', d3.forceManyBody())
      .force('center', d3.forceCenter(width / 2, height / 2));

    // 让 g 移动, 从而固定 svg
    const g = svg.append('g'); // 创建一个 <g> 元素用于包裹所有的图形元素

    // 在 <g> 元素下选择线条元素
    const link = g.selectAll('line')
      .data(this.links)
      .enter()
      .append('line')
      .attr('stroke', '#ccc')
      .attr('stroke-width', 1);

    // 在 <g> 元素下选择圆形元素
    const node = g.selectAll('circle')
      .data(this.nodes)
      .enter()
      .append('circle')
      .attr('r', 10)
      .attr('fill', 'red')
      .call(d3.drag()
        .on('start', dragstarted)
        .on('drag', dragged)
        .on('end', dragended));

    // 在 <g> 元素下选择文本元素
    const label = g.selectAll('.label')
      .data(this.nodes)
      .enter()
      .append('text')
      .attr('class', "label")
      .text(function (d) { return d.name; })
      .attr("dx", 12)
      .attr("dy", ".35em");

    node.append('title')
      .text(d => d.name);

    simulation.on('tick', () => {
      link
        .attr('x1', d => d.source.x)
        .attr('y1', d => d.source.y)
        .attr('x2', d => d.target.x)
        .attr('y2', d => d.target.y);

      node
        .attr('cx', d => d.x)
        .attr('cy', d => d.y);
      label
        .attr('x', d => d.x)
        .attr('y', d => d.y);
    });

    // 定义拖拽开始、拖拽过程和拖拽结束的函数
    function dragstarted(event, d) {
      if (!event.active) // 设置衰减系数，对节点位置移动过程的模拟，数值越高移动越快，数值范围[0，1]
        simulation.alphaTarget(0.1).restart();
        // restart(): 重新启动仿真的内部定时器并且返回仿真。
        // 与 simulation*.alphaTarget 或 simulation*.alpha结合使用，
        // 这个方法可以在交互期间再次激活仿真，比如拖拽节点或者在使用 simulation.stop临时暂停仿真后使用。
      d.fx = d.x;
      d.fy = d.y;
    }

    function dragged(event, d) {
      d.fx = event.x;
      d.fy = event.y;
    }

    function dragended(event, d) {
      if (!event.active)
        simulation.alphaTarget(0);
      // 让它回到原来的位置
      d.fx = null;
      d.fy = null;
    }

    // 添加缩放功能
    const zoom = d3.zoom()
      .extent([[0, 0], [width, height]]) // 范围, 不过没什么用
      .scaleExtent([0.5, 1.5]) // 缩放范围
      // 缩放时更新 SVG 中的元素位置和大小
      .on('zoom', event => g.attr('transform', event.transform)); // 使用 event.transform 更新 <g> 元素的变换

    svg.call(zoom);

    // 右键拖拽画布内容
    svg.on('contextmenu', () => {
      d3.event.preventDefault();
      svg.call(zoom.transform, d3.zoomIdentity.translate(0, 0).scale(1));
    });
  },
};
</script>
```
