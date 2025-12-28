import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';

// 定义 Props 类型, 继承自原生 img 标签的所有属性
type ZoomImageProps = React.ImgHTMLAttributes<HTMLImageElement> & {
    /** 可选: 全屏时的自定义类名 */
    overlayClassName?: string;
    /** 可选: 全屏时图片的自定义样式 */
    fullScreenImageStyle?: React.CSSProperties;
};

const ZoomImage: React.FC<ZoomImageProps> = ({
    style,
    className,
    overlayClassName,
    fullScreenImageStyle,
    onClick, // 解构出 onClick 以便组合逻辑
    ...props
}) => {
    const [isOpen, setIsOpen] = useState(false);
    // 用于动画或过渡状态(可选优化)
    const [isAnimating, setIsAnimating] = useState(false);

    // 处理打开
    const handleOpen = (e: React.MouseEvent<HTMLImageElement>) => {
        // 如果用户传入了 onClick, 先执行用户的逻辑
        if (onClick) {
            onClick(e);
        }
        setIsOpen(true);
        // 简单的延时以允许 CSS 动画介入(如果需要淡入效果)
        requestAnimationFrame(() => setIsAnimating(true));
    };

    // 处理关闭
    const handleClose = () => {
        setIsAnimating(false);
        setTimeout(() => setIsOpen(false), 300); // 延迟关闭以播放淡出动画
    };

    // 监听 ESC 键关闭
    useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.key === 'Escape' && isOpen) {
                handleClose();
            }
        };

        if (isOpen) {
            window.addEventListener('keydown', handleKeyDown);
            // 锁定 Body 滚动
            document.body.style.overflow = 'hidden';
        }

        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            // 恢复 Body 滚动
            document.body.style.overflow = '';
        };
    }, [isOpen]);

    // 缩略图样式: 默认鼠标变为放大镜
    const thumbnailStyle: React.CSSProperties = {
        cursor: 'zoom-in',
        ...style, // 允许用户覆盖样式
    };

    return (
        <>
            {/* 原始图片: 渲染在当前文档流中 */}
            <img
                {...props}
                className={className}
                style={thumbnailStyle}
                onClick={handleOpen}
            />

            {/* 全屏遮罩: 使用 Portal 渲染到 Body 底部, 打破父级 overflow 限制 */}
            {isOpen &&
                ReactDOM.createPortal(
                    <div
                        className={`zoom-image-overlay ${overlayClassName || ''}`}
                        onClick={handleClose}
                        style={{
                            position: 'fixed',
                            top: 0,
                            left: 0,
                            width: '100vw',
                            height: '100vh',
                            backgroundColor: 'rgba(0, 0, 0, 0.85)',
                            zIndex: 9999,
                            display: 'flex',
                            alignItems: 'center',
                            justifyContent: 'center',
                            opacity: isAnimating ? 1 : 0,
                            transition: 'opacity 0.3s ease-in-out',
                            backdropFilter: 'blur(5px)', // 可选: 毛玻璃效果
                        }}
                    >
                        <img
                            src={props.src}
                            alt={props.alt}
                            style={{
                                maxWidth: '95vw',
                                maxHeight: '95vh',
                                objectFit: 'contain',
                                cursor: 'zoom-out',
                                transform: isAnimating ? 'scale(1)' : 'scale(0.9)',
                                transition: 'transform 0.3s ease-in-out',
                                boxShadow: '0 10px 30px rgba(0,0,0,0.5)',
                                userSelect: 'none',
                                ...fullScreenImageStyle,
                            }}
                        // 阻止点击图片时触发遮罩层的关闭事件(如果你希望点击图片不关闭, 取消注释下面这行)
                        // onClick={(e) => e.stopPropagation()}
                        />

                        {/* 关闭按钮(可选) */}
                        <button
                            onClick={handleClose}
                            style={{
                                position: 'absolute',
                                top: '20px',
                                right: '20px',
                                background: 'transparent',
                                border: 'none',
                                color: 'white',
                                fontSize: '30px',
                                cursor: 'pointer',
                                zIndex: 10000,
                            }}
                            aria-label="Close"
                        >
                            &times;
                        </button>
                    </div>,
                    document.body // 挂载目标
                )}
        </>
    );
};

export default ZoomImage;
