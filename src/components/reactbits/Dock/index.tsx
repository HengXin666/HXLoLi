import {
  AnimatePresence,
  type MotionValue,
  motion,
  useMotionValue,
  useSpring,
  useTransform,
  type SpringOptions,
} from 'framer-motion';
import React, {
  Children,
  type ReactNode,
  isValidElement,
  useEffect,
  useRef,
  useState,
} from 'react';

import styles from './styles.module.css';

export interface DockItemData {
  icon: ReactNode;
  label: ReactNode;
  onClick: () => void;
  className?: string;
}

interface DockProps {
  items?: DockItemData[];
  children?: ReactNode;
  className?: string;
  distance?: number;
  panelHeight?: number;
  baseItemSize?: number;
  magnification?: number;
  spring?: SpringOptions;
  ariaLabel?: string;
}

interface DockItemProps {
  item: DockItemData;
  mouseX: MotionValue<number>;
  spring: SpringOptions;
  distance: number;
  baseItemSize: number;
  magnification: number;
}

interface DockSlotProps {
  children: ReactNode;
  mouseX: MotionValue<number>;
  spring: SpringOptions;
  distance: number;
  baseItemSize: number;
  magnification: number;
}

interface DockLabelProps {
  children: ReactNode;
  isHovered: MotionValue<number>;
}

function DockLabel({ children, isHovered }: DockLabelProps): ReactNode {
  const [visible, setVisible] = useState(false);

  useEffect(() => {
    const unsubscribe = isHovered.on('change', (latest) => {
      setVisible(latest === 1);
    });
    return () => unsubscribe();
  }, [isHovered]);

  return (
    <AnimatePresence>
      {visible && (
        <motion.span
          className={styles.dockLabel}
          role="tooltip"
          initial={{ opacity: 0, y: -3, x: '-50%' }}
          animate={{ opacity: 1, y: 0, x: '-50%' }}
          exit={{ opacity: 0, y: -3, x: '-50%' }}
          transition={{ duration: 0.16 }}
        >
          {children}
        </motion.span>
      )}
    </AnimatePresence>
  );
}

function DockItem({
  item,
  mouseX,
  spring,
  distance,
  baseItemSize,
  magnification,
}: DockItemProps): ReactNode {
  const ref = useRef<HTMLButtonElement>(null);
  const isHovered = useMotionValue(0);

  const mouseDistance = useTransform(mouseX, (value) => {
    const rect = ref.current?.getBoundingClientRect() ?? {
      x: 0,
      width: baseItemSize,
    };
    return value - rect.x - rect.width / 2;
  });
  const targetSize = useTransform(
    mouseDistance,
    [-distance, 0, distance],
    [baseItemSize, magnification, baseItemSize],
  );
  const size = useSpring(targetSize, spring);

  const handleKeyDown = (event: React.KeyboardEvent<HTMLButtonElement>) => {
    if (event.key !== 'Enter' && event.key !== ' ') return;
    event.preventDefault();
    item.onClick();
  };

  return (
    <motion.button
      ref={ref}
      type="button"
      className={`${styles.dockItem} ${item.className ?? ''}`}
      style={{ width: size, height: size }}
      onHoverStart={() => isHovered.set(1)}
      onHoverEnd={() => isHovered.set(0)}
      onFocus={() => isHovered.set(1)}
      onBlur={() => isHovered.set(0)}
      onClick={item.onClick}
      onKeyDown={handleKeyDown}
      aria-label={typeof item.label === 'string' ? item.label : undefined}
    >
      <span className={styles.dockIcon}>{item.icon}</span>
      <DockLabel isHovered={isHovered}>{item.label}</DockLabel>
    </motion.button>
  );
}

function DockSlot({
  children,
  mouseX,
  spring,
  distance,
  baseItemSize,
  magnification,
}: DockSlotProps): ReactNode {
  const ref = useRef<HTMLDivElement>(null);

  const mouseDistance = useTransform(mouseX, (value) => {
    const rect = ref.current?.getBoundingClientRect() ?? {
      x: 0,
      width: baseItemSize,
    };
    return value - rect.x - rect.width / 2;
  });
  const targetScale = useTransform(
    mouseDistance,
    [-distance, 0, distance],
    [1, magnification / baseItemSize, 1],
  );
  const scale = useSpring(targetScale, spring);

  return (
    <motion.div
      ref={ref}
      className={styles.dockSlot}
      style={{ scale }}
    >
      {children}
    </motion.div>
  );
}

export default function Dock({
  items,
  children,
  className = '',
  spring = { mass: 0.1, stiffness: 150, damping: 12 },
  magnification = 42,
  distance = 90,
  panelHeight = 34,
  baseItemSize = 28,
  ariaLabel = 'Dock controls',
}: DockProps): ReactNode {
  const mouseX = useMotionValue(Infinity);
  const childItems = Children.toArray(children).filter((child) => (
    isValidElement(child) || typeof child === 'string'
  ));

  return (
    <div
      className={styles.dockOuter}
      style={{ '--dock-panel-height': `${panelHeight}px` } as React.CSSProperties}
    >
      <motion.div
        className={`${styles.dockPanel} ${className}`}
        onMouseMove={({ clientX }) => mouseX.set(clientX)}
        onMouseLeave={() => mouseX.set(Infinity)}
        role="toolbar"
        aria-label={ariaLabel}
      >
        {items?.map((item, index) => (
            <DockItem
              key={index}
              item={item}
              mouseX={mouseX}
              spring={spring}
              distance={distance}
              magnification={magnification}
              baseItemSize={baseItemSize}
            />
          ))}
        {childItems.map((child, index) => (
          <DockSlot
            key={index}
            mouseX={mouseX}
            spring={spring}
            distance={distance}
            magnification={magnification}
            baseItemSize={baseItemSize}
          >
            {child}
          </DockSlot>
        ))}
      </motion.div>
    </div>
  );
}
