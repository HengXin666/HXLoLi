import React, { useState, useRef, useEffect } from 'react';
import ReactDOM from 'react-dom';
import styles from './Tooltip.module.css';

type TriggerElement = React.ReactElement<
  React.HTMLAttributes<HTMLElement> & React.RefAttributes<HTMLElement>
>;

interface TooltipProps {
  children: TriggerElement;
  content: React.ReactNode;
  selectable?: boolean;
  className?: string;
  style?: React.CSSProperties;
}

interface Coords {
  top: number;
  left: number;
}

const Tooltip: React.FC<TooltipProps> = ({
  children,
  content,
  selectable = false,
  className = '',
  style = {},
}) => {
  const [hoverVisible, setHoverVisible] = useState<boolean>(false);
  const [selected, setSelected] = useState<boolean>(false);
  const [coords, setCoords] = useState<Coords>({ top: 0, left: 0 });

  const triggerRef = useRef<HTMLElement | null>(null);
  const tooltipRef = useRef<HTMLDivElement | null>(null);

  const updateCoords = (): void => {
    if (!triggerRef.current) return;
    const rect = triggerRef.current.getBoundingClientRect();
    setCoords({
      top: rect.top + window.scrollY,
      left: rect.left + rect.width / 2,
    });
  };

  const handleMouseEnter = (): void => {
    // 如果已经选中, 就不再处理 hover 逻辑, 避免冲突
    if (selected) return;
    updateCoords();
    setHoverVisible(true);
  };

  const handleMouseLeave = (): void => {
    if (selected) return;
    setHoverVisible(false);
  };

  const handleClick = (event: React.MouseEvent): void => {
    if (!selectable) return;
    // 阻止触发元素本身的点击事件冒泡(视需求而定, 通常需要)
    event.stopPropagation();

    updateCoords();
    // 点击时关闭 hover 状态, 转为 selected 状态
    setHoverVisible(false);
    setSelected((prev) => !prev);
  };

  // 3. 核心: 阻止事件冒泡通用函数
  // 这防止了 Tooltip 内部的操作(点击、悬浮)冒泡到 React 树的上层组件
  // 从而解决了 "内容在另一个可悬浮的tip文本上, 导致悬浮文本又触发他的tip窗口" 的问题
  const stopPropagation = (e: React.SyntheticEvent) => {
    e.stopPropagation();
  };

  useEffect(() => {
    if (!selectable || !selected) return;

    const handleDocumentClick = (event: MouseEvent) => {
      const targetNode = event.target as Node;
      // 2. 这里的逻辑已经涵盖了 "点击tip窗口不会消失"
      if (
        !triggerRef.current?.contains(targetNode) &&
        !tooltipRef.current?.contains(targetNode)
      ) {
        setSelected(false);
      }
    };

    document.addEventListener('click', handleDocumentClick, true);
    return () => {
      document.removeEventListener('click', handleDocumentClick, true);
    };
  }, [selected, selectable]);

  const shouldShowTooltip: boolean = selected || hoverVisible;

  const triggerWithRef = React.cloneElement(children, {
    ref: triggerRef,
    onMouseEnter: handleMouseEnter,
    onMouseLeave: handleMouseLeave,
    onClick: handleClick,
  });

  return (
    <>
      {triggerWithRef}
      {shouldShowTooltip &&
        ReactDOM.createPortal(
          <div
            ref={tooltipRef}
            className={`${styles.tooltipPortal} ${className}`.trim()}
            // 3. 在 Portal 容器层阻断所有常见的鼠标事件冒泡
            onMouseDown={stopPropagation}
            onMouseUp={stopPropagation}
            onClick={stopPropagation}
            onMouseEnter={stopPropagation}
            onMouseLeave={stopPropagation}
            onWheel={stopPropagation} // 如果内容有滚动条, 防止滚动穿透
            style={{
              top: `${coords.top}px`,
              left: `${coords.left}px`,
              position: 'absolute', // 确保 Portal 定位正确
              pointerEvents: 'auto', // 确保鼠标事件被此层捕获, 不穿透到 DOM 底层元素
            }}
          >
            {/* 1. & 4. 内容层: 合并样式, 处理 TabIndex */}
            <div
              className={styles.tooltipContent}
              // 1. 允许聚焦
              tabIndex={-1}
              style={{
                ...style,               // 应用用户自定义样式
              }}
            >
              {content}
            </div>
          </div>,
          document.body
        )}
    </>
  );
};

export default Tooltip;
