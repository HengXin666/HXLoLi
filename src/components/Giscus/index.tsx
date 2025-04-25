import React from 'react';
import Giscus from '@giscus/react';
import {
    useThemeConfig,
    useColorMode,
    ThemeConfig
} from '@docusaurus/theme-common';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useLocation } from '@docusaurus/router';
import config from '@generated/docusaurus.config';

type GiscusProps = React.ComponentProps<typeof Giscus>;

interface CustomThemeConfig extends ThemeConfig {
    giscus: GiscusProps & { darkTheme: string };
}

// 记录上一次的url, 如果url不同, 那么就初始化 useStore
let maeLocation: string = '';
let title: string = '';

/**
 * 初始化页面, 如果url更新了, 那么会初始化变量
 * @returns 
 */
function initComponent () {
    const location = useLocation();
    if (maeLocation != location.pathname) {
        // console.log(maeLocation, document.title); // 发现: document.title 是有延迟更新的!
        maeLocation = location.pathname;
        if (maeLocation.startsWith(config.baseUrl)) {
            title = maeLocation.replace(config.baseUrl, '');
        }
    }
}

/**
 * 定义一个组件, 会判断当前颜色主题和用户配置, 然后生成`Giscus`组件.
 * @returns 
 */
const HXGiscus: React.FC = () => {
    initComponent();
    const { giscus } = useThemeConfig() as CustomThemeConfig;
    const { colorMode } = useColorMode();
    const { theme = 'light', darkTheme = 'dark_dimmed' } = giscus;
    const giscusTheme = colorMode === 'dark' ? darkTheme : theme;

    return (
        <BrowserOnly fallback={<div>Loading Comments...</div>}>
            {() => {
                return (
                    <div id="comment" style={{ paddingTop: 50 }}>
                        <Giscus
                            id="comments"
                            mapping="specific"
                            strict="1"
                            reactionsEnabled="1"
                            emitMetadata="0"
                            inputPosition="bottom"
                            lang="zh-CN"
                            loading="lazy"
                            term={title}
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