import React from 'react';
import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import config from '@generated/docusaurus.config';

import './index.css';

// 模块卡片组件
function FeatureCard ({
    title, description, icon, link
}: {
    title: string;
    description: string;
    icon: ReactNode;
    link: string;
}) {
    return (
        <div className="animated-box">
            <Link
                style={{textDecoration: 'none'}}
            >
                <div>{icon}</div>
                <h3 style={{ color: '#E0E0D8' }}>{title}</h3>
                <p style={{ color: '#929AA1' }}>{description}</p>
            </Link>
        </div>
    );
}

function HomepageHeader () {
    const { siteConfig } = useDocusaurusContext();
    return (
        <header
            style={{
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                padding: '40px 0',
                marginLeft: '100px',
                marginRight: '100px',
            }}
        >
            <div className="container" style={{ flex: 1, textAlign: 'center' }}>
                <Heading as="h1" className="hero__title" style={{ color: '#8CFF00' }}>
                    {siteConfig.title}
                </Heading>
                <p className="hero__subtitle" style={{ color: '#A9A9A9' }}>
                    {siteConfig.tagline}
                </p>
                <div>
                    <Link
                        className="button button--lg"
                        style={{
                            backgroundColor: '#FFDC33',
                            color: 'black',
                            borderRadius: '8px',
                        }}
                        to="/docs/HXLoLiHub使用说明/简介"
                    >
                        快速开始
                    </Link>
                </div>
            </div>
            <div className="image-container" style={{ flex: 1 }}>
                <img
                    src={`${config.baseUrl}/img/main_menu_misaka.png`}
                    alt="Image"
                    style={{
                        width: '100%',
                        height: 'auto',
                        borderRadius: '10px',
                        boxShadow: '0px 5px 10px 1px #6BE4F6'
                    }}
                />
            </div>
        </header>
    );
}

export default function Home (): ReactNode {
    const { siteConfig } = useDocusaurusContext();
    return (
        <Layout
            title={`欢迎来到 ${siteConfig.title}`}
            description="Description will go into a meta tag in <head />"
        >
            <HomepageHeader />
            <main>
                <div className="container">
                    <div className="row justify-content-center">
                        {/* 功能模块卡片 */}
                        <FeatureCard
                            title="Github Page 一键部署"
                            description="简化部署流程，无需额外配置服务器，可直接将项目一键部署至 Github Pages，快速上线。"
                            icon={<i className="fas fa-cogs" />}
                            link="/docs/github-page"
                        />
                        <FeatureCard
                            title="以编辑预览"
                            description="支持网页端工作台编辑 .drawio.svg 文件，方便预览，而不是仅放大图片。"
                            icon={<i className="fas fa-image" />}
                            link="/docs/drawio"
                        />
                        <FeatureCard
                            title="支持编辑代码块"
                            description="内嵌VsCode同款编辑器，在网页端可以编辑代码，而不是仅复制。支持一键还原到原本文本。"
                            icon={<i className="fas fa-tasks" />}
                            link="/docs/code"
                        />
                        <FeatureCard
                            title="高度自定义"
                            description="基于 Docusaurus 定制开发，支持高度自定义，用户可以在此基础上根据需求进行二次开发，扩展功能和设计。"
                            icon={<i className="fas fa-users" />}
                            link="/docs/todo"
                        />
                        <FeatureCard
                            title="版本管理"
                            description="项目可直接推送至 Github 进行版本控制，独立的纯文件项目能够轻松实现 Git 版本管理，保证代码与文档的一致性和可追溯性。"
                            icon={<i className="fas fa-cogs" />}
                            link="/docs/git"
                        />
                        <FeatureCard
                            title="响应式博客 / 移动端支持"
                            description="Docusaurus 内置强大的响应式设计，自动适配不同尺寸的屏幕，无论是桌面还是移动端，用户都能获得流畅的浏览体验。"
                            icon={<i className="fas fa-mobile-alt" />}
                            link="/docs/responsive"
                        />
                    </div>
                </div>
            </main>
        </Layout >
    );
}
