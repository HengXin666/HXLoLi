import React, { useEffect, useState } from 'react';
import Giscus from '@giscus/react';
import { useThemeConfig, useColorMode } from '@docusaurus/theme-common';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// 增加 description 属性
const HXGiscus: React.FC<{ term?: string, description?: string }> = ({ term: propTerm, description }) => {
    const { siteConfig } = useDocusaurusContext();
    const location = useLocation();
    const { giscus } = useThemeConfig() as any;
    const { colorMode } = useColorMode();

    // 用于控制渲染时机, 确保修改完 meta 后再加载 Giscus
    const [isReady, setIsReady] = useState(false);

    const { theme = 'light', darkTheme = 'dark_dimmed' } = giscus || {};
    const giscusTheme = colorMode === 'dark' ? darkTheme : theme;

    // 1. 计算 Term (保持你原有的逻辑或简化版)
    const currentPath = location.pathname;
    const baseUrl = siteConfig.baseUrl;
    let calculatedTerm = currentPath;
    if (currentPath.startsWith(baseUrl)) {
        calculatedTerm = currentPath.substring(baseUrl.length);
    }
    const finalTerm = propTerm || calculatedTerm;

    return (
        <BrowserOnly fallback={<div>Loading Comments...</div>}>
            {() => {
                // 使用 useEffect 强制劫持 meta 标签
                useEffect(() => {
                    // 先设置为未准备好, 卸载旧的 Giscus
                    setIsReady(false);

                    // 1. 找到页面上的 meta description
                    let metaDesc = document.querySelector("meta[name='description']");

                    // 2. 如果没找到(有时候 Docusaurus 还没生成), 就自己造一个
                    if (!metaDesc) {
                        metaDesc = document.createElement('meta');
                        metaDesc.setAttribute('name', 'description');
                        document.head.appendChild(metaDesc);
                    }

                    // 3. 【核心步骤】强制修改它的内容!
                    // 优先使用传入的 description, 否则使用通用文案, 防止 Giscus 抓取正文第一段
                    const contentToSet = description || `欢迎在 ${finalTerm} 留下你的评论`;
                    metaDesc.setAttribute('content', contentToSet);

                    // 4. 稍微延迟一下, 确保 DOM 生效后再渲染 Giscus
                    // (虽然 React 的 setState 是异步的, 但这里的 timeout 能解决大部分竞态问题)
                    const timer = setTimeout(() => {
                        setIsReady(true);
                    }, 50);

                    return () => clearTimeout(timer);
                }, [finalTerm, description]); // 当路径或描述变化时重新执行

                // 如果还没准备好(正在修改 meta), 就不渲染 Giscus
                if (!isReady) return <div style={{ paddingTop: 50 }}>Updating metadata...</div>;

                return (
                    <div id="comment" style={{ paddingTop: 50 }}>
                        <Giscus
                            // 使用 term 作为 key, 确保路由切换时组件彻底重建
                            key={finalTerm}
                            id="comments"
                            mapping="specific"
                            strict="0"
                            reactionsEnabled="1"
                            emitMetadata="1"
                            inputPosition="bottom"
                            lang="zh-CN"
                            loading="lazy"
                            term={finalTerm}
                            {...giscus}
                            theme={giscusTheme}
                        />
                    </div>
                );
            }}
        </BrowserOnly>
    );
};

export default HXGiscus;
