import React, { useState, useRef, useEffect } from 'react';
import ReactDOM from 'react-dom';

import styles from './Tooltip.module.css';

type TriggerElement = React.ReactElement<
    React.HTMLAttributes<HTMLElement> & React.RefAttributes<HTMLElement>
>;

// 1. 在 Props 接口中添加 style 属性
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
    style = {}, // 2. 解构 style prop，并提供默认值
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
        event.stopPropagation();
        updateCoords();
        setHoverVisible(false);
        setSelected(prev => !prev);
    };

    useEffect(() => {
        if (!selectable || !selected) return;
        const handleDocumentClick = (event: MouseEvent) => {
            const targetNode = event.target as Node;
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
                        // 3. 将内部定位样式和外部传入的 style 对象合并
                        style={{
                            top: `${coords.top}px`,
                            left: `${coords.left}px`
                        }}
                    >
                        <div className={styles.tooltipContent} style={{...style}}>
                            {content}
                        </div>
                    </div>,
                    document.body,
                )}
        </>
    );
};

export default Tooltip;
