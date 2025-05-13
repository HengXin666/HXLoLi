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
                        to="/docs/关于"
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
            description="<head />"
        >
            <HomepageHeader />
            <main>
                <div className="container">
                    <h2>这是我的个人博客~ 会把我学习的笔记、日常博客分享上来~ 喜欢的可以给项目点点Start~</h2>
                </div>
            </main>
        </Layout >
    );
}
