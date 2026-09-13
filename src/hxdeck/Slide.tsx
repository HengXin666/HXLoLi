import React from 'react';

/**
 * 一屏内容 + 它的导航元数据.
 *
 * Deck 通过读取 Slide 的 props 来生成左侧章节栏与右侧进度点,
 * 因此导航不需要使用者手工维护第二份清单.
 */
export interface SlideProps {
    /** 左侧栏显示的本屏标题 */
    title?: string;
    /** 所属章节, 用于左侧栏分组; 相同 chapter 的屏归为一组 */
    chapter?: string;
    children: React.ReactNode;
}

export function Slide({ children }: SlideProps): React.ReactElement {
    // Deck 会把内容取出来自己渲染; 这里只是承载 props 的声明式外壳
    return <>{children}</>;
}

export default Slide;
