import React, { useCallback, useEffect, useId, useLayoutEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom';
import styles from './Tooltip.module.css';

/**
 * 全站统一的悬浮卡片. 首页把原生 title 全部换成它, 番剧页沿用原来的 selectable 用法.
 *
 * - 触发: 鼠标悬停 / 键盘聚焦 (focus) / 触屏轻点 (非链接时切换开关); Esc 关闭.
 * - 无障碍: 触发元素带 aria-describedby, 卡片 role="tooltip".
 * - 定位: position: fixed, 默认在上方; 上方放不下翻到下方, 左右贴边时夹在视口内 8px, 小箭头跟着触发点走.
 * - 滚动或窗口尺寸变化时重新定位. prefers-reduced-motion 下不做淡入位移.
 * 另有 useHoverCard 给一个容器里成百上千个格子共用一张卡片 (WakaTime 热力格).
 * 定位层 FloatingLayer 也导出给弹层类组件复用 (common/Tooltip/Select 的下拉列表), 不另起一套定位.
 * .agents/notes/implemented/feature/2026-10-31-homepage-profile-and-project-index.md
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
type TriggerElement = React.ReactElement<
  React.HTMLAttributes<HTMLElement> & React.RefAttributes<HTMLElement>
>;

interface TooltipProps {
  children: TriggerElement;
  content: React.ReactNode;
  /** 点击后常驻, 卡片里的内容可以交互 (番剧页用) */
  selectable?: boolean;
  className?: string;
  /** 作用在卡片内容层上, 例如 { width: 'auto' } */
  style?: React.CSSProperties;
  /** 卡片宽度, 默认 300 (番剧页原样); 'auto' 时按内容宽, 最宽 320 */
  width?: number | 'auto';
}

const GAP = 10;
const EDGE = 8;

type Placement = 'top' | 'bottom';

interface FloatingProps {
  id: string;
  anchor: HTMLElement | null;
  content: React.ReactNode;
  className?: string;
  style?: React.CSSProperties;
  width?: number | 'auto';
  interactive?: boolean;
  cardRef?: React.MutableRefObject<HTMLDivElement | null>;
  /** 外层 role, 默认 tooltip; 弹层自己带 listbox 等角色时传 presentation */
  role?: string;
  /** 优先放上方还是下方, 放不下再翻; 默认 top (tooltip 原样) */
  prefer?: Placement;
  /** 横向对齐: center 以触发点居中 (默认), start 左缘对齐触发元素 */
  align?: 'center' | 'start';
  /** 是否画小箭头, 默认画 */
  arrow?: boolean;
  /** 挂载点, 默认 document.body; 在模态 <dialog> 里要挂进那个 dialog, 否则在顶层之下点不到 */
  container?: Element | null;
}

/**
 * 真正画出来的那张卡片: 量完自己的尺寸再定位, 定位前不可见, 避免闪到左上角.
 * prefer / align / arrow / container / role 给下拉列表这类弹层复用 (common/Tooltip/Select), 默认值保持 tooltip 原样.
 * 单选下拉 Select 的决策见同目录 Select.tsx 顶部注释
 */
export function FloatingLayer({
  id, anchor, content, className = '', style, width = 300, interactive = false, cardRef,
  role = 'tooltip', prefer = 'top', align = 'center', arrow = true, container,
}: FloatingProps) {
  const ref = useRef<HTMLDivElement | null>(null);
  const [pos, setPos] = useState<{ top: number; left: number; arrow: number; placement: Placement } | null>(null);

  const place = useCallback(() => {
    const el = ref.current;
    if (!el || !anchor || !anchor.isConnected) return;
    const a = anchor.getBoundingClientRect();
    const w = el.offsetWidth;
    const h = el.offsetHeight;
    const vw = document.documentElement.clientWidth;
    const vh = window.innerHeight;
    const roomTop = a.top - GAP - h >= EDGE;
    const roomBottom = a.bottom + GAP + h <= vh - EDGE;
    const placement: Placement = prefer === 'bottom'
      ? (roomBottom || !roomTop ? 'bottom' : 'top')
      : (roomTop || !roomBottom ? 'top' : 'bottom');
    const top = placement === 'top' ? Math.max(EDGE, a.top - GAP - h) : a.bottom + GAP;
    const center = a.left + a.width / 2;
    const wanted = align === 'start' ? a.left : center - w / 2;
    const left = Math.min(Math.max(EDGE, wanted), Math.max(EDGE, vw - w - EDGE));
    const arrow = Math.min(Math.max(12, center - left), w - 12);
    setPos({ top, left, arrow, placement });
  }, [anchor, prefer, align]);

  useLayoutEffect(() => {
    place();
  }, [place, content]);

  useEffect(() => {
    const onMove = () => place();
    window.addEventListener('scroll', onMove, true);
    window.addEventListener('resize', onMove);
    return () => {
      window.removeEventListener('scroll', onMove, true);
      window.removeEventListener('resize', onMove);
    };
  }, [place]);

  const widthStyle: React.CSSProperties = width === 'auto' ? { width: 'max-content', maxWidth: 320 } : { width };

  return ReactDOM.createPortal(
    <div
      ref={(el) => {
        ref.current = el;
        if (cardRef) cardRef.current = el;
      }}
      id={id}
      role={role}
      className={`${styles.tooltipPortal} ${className}`.trim()}
      data-placement={pos?.placement ?? prefer}
      data-arrow={arrow ? undefined : 'false'}
      data-ready={pos ? 'true' : 'false'}
      style={{
        top: pos?.top ?? 0,
        left: pos?.left ?? 0,
        pointerEvents: interactive ? 'auto' : 'none',
        ['--tip-arrow-x' as string]: `${pos?.arrow ?? 0}px`,
      }}
      onClick={(e) => e.stopPropagation()}
      onMouseDown={(e) => e.stopPropagation()}
    >
      <div className={styles.tooltipContent} style={{ ...widthStyle, ...style }}>
        {content}
      </div>
    </div>,
    container ?? document.body,
  );
}

const Tooltip: React.FC<TooltipProps> = ({
  children,
  content,
  selectable = false,
  className = '',
  style,
  width = 300,
}) => {
  const id = `hx-tip-${useId().replace(/:/g, '')}`;
  const [hover, setHover] = useState(false);
  const [focus, setFocus] = useState(false);
  const [pinned, setPinned] = useState(false);
  const [anchor, setAnchor] = useState<HTMLElement | null>(null);
  const cardRef = useRef<HTMLDivElement | null>(null);
  const hideTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const lastPointer = useRef<string>('mouse');

  const childProps = children.props;
  // 触发元素自己的 ref (站点是 React 19, ref 在 props 上), 与本组件的锚点 ref 合并; 回调保持稳定, 不会每次渲染都重挂
  const ownRef = useRef<React.Ref<HTMLElement> | undefined>(undefined);
  ownRef.current = (childProps as { ref?: React.Ref<HTMLElement> }).ref;
  const setRef = useCallback((el: HTMLElement | null) => {
    setAnchor(el);
    const own = ownRef.current;
    if (typeof own === 'function') own(el);
    else if (own && typeof own === 'object') (own as React.MutableRefObject<HTMLElement | null>).current = el;
  }, []);
  const isLink = typeof children.type === 'string' && (children.type === 'a' || children.type === 'button');

  const open = pinned || hover || focus;

  useEffect(() => {
    if (!open) return undefined;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        setHover(false);
        setFocus(false);
        setPinned(false);
      }
    };
    const onDown = (e: PointerEvent) => {
      const t = e.target as Node;
      if (anchor?.contains(t) || cardRef.current?.contains(t)) return;
      setPinned(false);
      setHover(false);
    };
    window.addEventListener('keydown', onKey);
    document.addEventListener('pointerdown', onDown, true);
    return () => {
      window.removeEventListener('keydown', onKey);
      document.removeEventListener('pointerdown', onDown, true);
    };
  }, [open, anchor]);

  useEffect(() => () => {
    if (hideTimer.current) clearTimeout(hideTimer.current);
  }, []);

  const trigger = React.cloneElement(children, {
    ref: setRef,
    'aria-describedby': [childProps['aria-describedby'], open ? id : undefined].filter(Boolean).join(' ') || undefined,
    onPointerEnter: (e: React.PointerEvent<HTMLElement>) => {
      childProps.onPointerEnter?.(e);
      lastPointer.current = e.pointerType;
      if (e.pointerType !== 'mouse' && e.pointerType !== 'pen') return;
      if (hideTimer.current) clearTimeout(hideTimer.current);
      setHover(true);
    },
    onPointerLeave: (e: React.PointerEvent<HTMLElement>) => {
      childProps.onPointerLeave?.(e);
      if (e.pointerType !== 'mouse' && e.pointerType !== 'pen') return;
      // 可交互的卡片留 120ms 给鼠标移进去
      hideTimer.current = setTimeout(() => setHover(false), selectable ? 120 : 0);
    },
    onPointerDown: (e: React.PointerEvent<HTMLElement>) => {
      childProps.onPointerDown?.(e);
      lastPointer.current = e.pointerType;
    },
    onFocus: (e: React.FocusEvent<HTMLElement>) => {
      childProps.onFocus?.(e);
      setFocus(true);
    },
    onBlur: (e: React.FocusEvent<HTMLElement>) => {
      childProps.onBlur?.(e);
      setFocus(false);
    },
    onClick: (e: React.MouseEvent<HTMLElement>) => {
      childProps.onClick?.(e);
      if (selectable) {
        e.stopPropagation();
        setPinned((v) => !v);
        return;
      }
      // 触屏没有悬停: 非链接的触发元素轻点一下切换卡片; 链接照常跳转, 它的要点已经写在正文里
      if (lastPointer.current === 'touch' && !isLink) setPinned((v) => !v);
    },
  } as Partial<React.HTMLAttributes<HTMLElement>> & React.RefAttributes<HTMLElement>);

  return (
    <>
      {trigger}
      {open && anchor && (
        <FloatingLayer
          id={id}
          anchor={anchor}
          content={content}
          className={className}
          style={style}
          width={width}
          interactive={selectable}
          cardRef={cardRef}
        />
      )}
    </>
  );
};

/**
 * 一个容器里很多触发点共用一张卡片: open(锚点元素, 内容) / close().
 * 容器自己负责键盘 (例如方向键移动当前格) 并把 describedBy 挂到容器上.
 */
export function useHoverCard(options: { width?: number | 'auto'; className?: string } = {}) {
  const id = `hx-tip-${useId().replace(/:/g, '')}`;
  const [state, setState] = useState<{ anchor: HTMLElement; content: React.ReactNode } | null>(null);
  const openCard = useCallback((anchor: HTMLElement, content: React.ReactNode) => setState({ anchor, content }), []);
  const close = useCallback(() => setState(null), []);
  const element = state ? (
    <FloatingLayer
      id={id}
      anchor={state.anchor}
      content={state.content}
      width={options.width ?? 'auto'}
      className={options.className}
    />
  ) : null;
  return { id, isOpen: state !== null, open: openCard, close, element };
}

export default Tooltip;
