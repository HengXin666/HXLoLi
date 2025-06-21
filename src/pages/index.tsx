import React from 'react';
import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import config from '@generated/docusaurus.config';
import { stats } from '@site/data/wordStats';
import WordCountChart from '../components/WordCountChart';
import HXLink from '../components/HXLink';
import BlogWithCats from '../components/BlogWithCats';
import ProjectCarousel from '../components/ProjectCarousel';

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
                style={{ textDecoration: 'none' }}
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
                <div className="image-container"
                    style={{
                        position: 'relative', // 让按钮以这个为定位参考
                        maxWidth: '85%', // 控制最大宽度
                        margin: '0 auto', // 水平居中
                        borderRadius: '10px',
                        overflow: 'hidden', // 防止按钮超出边界
                        boxShadow: '0px 5px 10px 1px #6BE4F6',
                    }}
                >
                    <img
                        src={`${config.baseUrl}/img/main_menu_misaka.png`}
                        alt="Image"
                        style={{
                            width: '100%',
                            height: 'auto',
                            display: 'block'
                        }}
                    />
                    <Link
                        className="button button--lg"
                        style={{
                            backgroundColor: '#FFDC33',
                            color: 'black',
                            borderRadius: '8px',
                            position: 'absolute',
                            bottom: '20px',           // 距离底部 20px
                            left: '50%',              // 居中
                            transform: 'translateX(-50%)', // 精确水平居中
                            zIndex: 10
                        }}
                        to="/docs/关于"
                    >
                        快速开始
                    </Link>
                </div>

            </div>
            <div style={{ flex: 1 }}>
                <div className="p-4 max-w-4xl mx-auto">
                    <WordCountChart rawData={stats} />
                </div>
            </div>
        </header>
    );
}

export default function Home (): ReactNode {
    const { siteConfig } = useDocusaurusContext();
    return (
        <Layout
            title={`いにゃさい ${siteConfig.title}`}
            description="<head />"
        >
            <HomepageHeader />
            <BlogWithCats style={{textAlign: 'center', padding: '20px', backgroundColor: '#2b2b2b'}}>
                <div className="container">
                    <h2 style={{color: '#E3E3E3'}}>
                        这里是 <HXLink title='Heng_Xin' url='https://github.com/HengXin666' /> 的个人博客~, 可以去<HXLink title='仓库' url='https://github.com/HengXin666/HXLoLi' />点一个 star 支持一下吗?
                    </h2>
                    <div style={{height: '320px'}}>
                        <ProjectCarousel />
                    </div>
                    <a
                        target="_blank"
                        rel="noopener noreferrer nofollow"
                        href="https://camo.githubusercontent.com/6ee8861d02fe0210e7f0bc225cecea80c3e23bd7e68c1324ebacdf9f2e3a4e51/68747470733a2f2f6769746875622d726561646d652d73746174732d666c616d652d70692d37302e76657263656c2e6170702f6170693f757365726e616d653d48656e6758696e3636362673686f775f69636f6e733d74727565267468656d653d7472616e73706172656e74266c6f63616c653d6a61267469746c655f636f6c6f723d39393030393926686964655f626f726465723d747275652669636f6e5f636f6c6f723d46374345343526746578745f636f6c6f723d443137323737"
                    >
                        <img
                            width={400}
                            src="https://camo.githubusercontent.com/6ee8861d02fe0210e7f0bc225cecea80c3e23bd7e68c1324ebacdf9f2e3a4e51/68747470733a2f2f6769746875622d726561646d652d73746174732d666c616d652d70692d37302e76657263656c2e6170702f6170693f757365726e616d653d48656e6758696e3636362673686f775f69636f6e733d74727565267468656d653d7472616e73706172656e74266c6f63616c653d6a61267469746c655f636f6c6f723d39393030393926686964655f626f726465723d747275652669636f6e5f636f6c6f723d46374345343526746578745f636f6c6f723d443137323737"
                            title="GitHub Stats"
                            data-canonical-src="https://github-readme-stats-flame-pi-70.vercel.app/api?username=HengXin666&show_icons=true&theme=transparent&locale=ja&title_color=990099&hide_border=true&icon_color=F7CE45&text_color=D17277"
                            style={{ maxWidth: "100%" }}
                        />
                    </a>
                    <a
                        target="_blank"
                        rel="noopener noreferrer nofollow"
                        href="https://camo.githubusercontent.com/3576a8c4f899103b6bdc86df72ccb4637cc3baafa87e4c72f5745e454bc6ba1f/68747470733a2f2f6769746875622d726561646d652d73747265616b2d73746174732d74776f2d636f72616c2d32342e76657263656c2e6170703f757365723d48656e6758696e363636267468656d653d7261646963616c26686964655f626f726465723d7472756526626f726465725f7261646975733d3130266c6f63616c653d6a612673686f72745f6e756d626572733d66616c736525433225413025433225413025453625393725413025453625393525383826646174655f666f726d61743d253542592e2535446e2e6a"
                    >
                        <img
                            width={400}
                            src="https://camo.githubusercontent.com/3576a8c4f899103b6bdc86df72ccb4637cc3baafa87e4c72f5745e454bc6ba1f/68747470733a2f2f6769746875622d726561646d652d73747265616b2d73746174732d74776f2d636f72616c2d32342e76657263656c2e6170703f757365723d48656e6758696e363636267468656d653d7261646963616c26686964655f626f726465723d7472756526626f726465725f7261646975733d3130266c6f63616c653d6a612673686f72745f6e756d626572733d66616c736525433225413025433225413025453625393725413025453625393525383826646174655f666f726d61743d253542592e2535446e2e6a"
                            title="GitHub Streak"
                            data-canonical-src="https://github-readme-streak-stats-two-coral-24.vercel.app?user=HengXin666&theme=radical&hide_border=true&border_radius=10&locale=ja&short_numbers=false%C2%A0%C2%A0%E6%97%A0%E6%95%88&date_format=%5BY.%5Dn.j"
                            style={{ maxWidth: "100%" }}
                        />
                    </a>
                    <p dir="auto">
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/0b10f30898c209d90f970934f5eece1ebdd0926df240cfe5369081290c1a6432/68747470733a2f2f6769746875622d726561646d652d61637469766974792d67726170682e76657263656c2e6170702f67726170683f757365726e616d653d48656e6758696e3636362673686f775f69636f6e733d74727565267468656d653d6769746875622d636f6d70616374266c6f63616c653d6a61267469746c655f636f6c6f723d3939303039392669636f6e5f636f6c6f723d46374345343526746578745f636f6c6f723d44313732373726686964655f626f726465723d74727565"
                        >
                            <img
                                src="https://camo.githubusercontent.com/0b10f30898c209d90f970934f5eece1ebdd0926df240cfe5369081290c1a6432/68747470733a2f2f6769746875622d726561646d652d61637469766974792d67726170682e76657263656c2e6170702f67726170683f757365726e616d653d48656e6758696e3636362673686f775f69636f6e733d74727565267468656d653d6769746875622d636f6d70616374266c6f63616c653d6a61267469746c655f636f6c6f723d3939303039392669636f6e5f636f6c6f723d46374345343526746578745f636f6c6f723d44313732373726686964655f626f726465723d74727565"
                                alt="Activity Graph"
                                data-canonical-src="https://github-readme-activity-graph.vercel.app/graph?username=HengXin666&show_icons=true&theme=github-compact&locale=ja&title_color=990099&icon_color=F7CE45&text_color=D17277&hide_border=true"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                    </p>
                    <p dir="auto">
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/4a309eb2a40a5588fea63c42bfec65ad770d61f1d048abef9fb15ffc7fb716d5/68747470733a2f2f6769746875622d726561646d652d73746174732d666c616d652d70692d37302e76657263656c2e6170702f6170692f77616b6174696d653f757365726e616d653d48656e675f58696e267468656d653d7472616e73706172656e7426686964655f626f726465723d74727565266c61796f75743d636f6d70616374266c616e67735f636f756e743d313134353134266c6f63616c653d6a61267469746c655f636f6c6f723d39393030393926746578745f636f6c6f723d443137323737"
                        >
                            <img
                                src="https://camo.githubusercontent.com/4a309eb2a40a5588fea63c42bfec65ad770d61f1d048abef9fb15ffc7fb716d5/68747470733a2f2f6769746875622d726561646d652d73746174732d666c616d652d70692d37302e76657263656c2e6170702f6170692f77616b6174696d653f757365726e616d653d48656e675f58696e267468656d653d7472616e73706172656e7426686964655f626f726465723d74727565266c61796f75743d636f6d70616374266c616e67735f636f756e743d313134353134266c6f63616c653d6a61267469746c655f636f6c6f723d39393030393926746578745f636f6c6f723d443137323737"
                                alt="WakaTime Stats"
                                data-canonical-src="https://github-readme-stats-flame-pi-70.vercel.app/api/wakatime?username=Heng_Xin&theme=transparent&hide_border=true&layout=compact&langs_count=114514&locale=ja&title_color=990099&text_color=D17277"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/8f1b9122ccbc56dd3ef02ac2e6a858dd17286e4e497c3910d86d53ee59c92ac5/68747470733a2f2f6769746875622d726561646d652d73746174732d666c616d652d70692d37302e76657263656c2e6170702f6170692f746f702d6c616e67732f3f757365726e616d653d48656e6758696e363636267468656d653d7472616e73706172656e7426686964655f626f726465723d74727565266c61796f75743d646f6e75742d766572746963616c266c616e67735f636f756e743d313134353134266c6f63616c653d6a61267469746c655f636f6c6f723d39393030393926746578745f636f6c6f723d443137323737"
                        >
                            <img
                                src="https://camo.githubusercontent.com/8f1b9122ccbc56dd3ef02ac2e6a858dd17286e4e497c3910d86d53ee59c92ac5/68747470733a2f2f6769746875622d726561646d652d73746174732d666c616d652d70692d37302e76657263656c2e6170702f6170692f746f702d6c616e67732f3f757365726e616d653d48656e6758696e363636267468656d653d7472616e73706172656e7426686964655f626f726465723d74727565266c61796f75743d646f6e75742d766572746963616c266c616e67735f636f756e743d313134353134266c6f63616c653d6a61267469746c655f636f6c6f723d39393030393926746578745f636f6c6f723d443137323737"
                                alt="Top Langs"
                                data-canonical-src="https://github-readme-stats-flame-pi-70.vercel.app/api/top-langs/?username=HengXin666&theme=transparent&hide_border=true&layout=donut-vertical&langs_count=114514&locale=ja&title_color=990099&text_color=D17277"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                    </p>
                    <p dir="auto">
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/494f76132d612633f256e65dc49a78ada7142f28d1b7ba77b999d005b448f893/68747470733a2f2f736b696c6c69636f6e732e6465762f69636f6e733f693d6769742c6769746875622c632c6370702c636d616b652c71742c6c696e75782c617263682c646f636b65722c70792c6a6176612c737072696e672c6d7973716c2c72656469732c6d6f6e676f64622c68746d6c2c6373732c6a732c74732c7675652c63662c77696e646f77732c6d64267468656d653d6c69676874"
                        >
                            <img
                                src="https://camo.githubusercontent.com/494f76132d612633f256e65dc49a78ada7142f28d1b7ba77b999d005b448f893/68747470733a2f2f736b696c6c69636f6e732e6465762f69636f6e733f693d6769742c6769746875622c632c6370702c636d616b652c71742c6c696e75782c617263682c646f636b65722c70792c6a6176612c737072696e672c6d7973716c2c72656469732c6d6f6e676f64622c68746d6c2c6373732c6a732c74732c7675652c63662c77696e646f77732c6d64267468656d653d6c69676874"
                                alt="Skills"
                                data-canonical-src="https://skillicons.dev/icons?i=git,github,c,cpp,cmake,qt,linux,arch,docker,py,java,spring,mysql,redis,mongodb,html,css,js,ts,vue,cf,windows,md&theme=light"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                    </p>
                    <p dir="auto">
                        <a
                            href="https://learn.microsoft.com/zh-cn/cpp/cpp/welcome-back-to-cpp-modern-cpp"
                            rel="nofollow"
                        >
                            <img
                                src="https://camo.githubusercontent.com/f08bbb0a403752f561af3a561186bcdfab9d402d7527dc8b3bdecb40b18270ed/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f436f64652d4d6f6465726e253230432b2b2d626c7565"
                                alt="Modern C++"
                                data-canonical-src="https://img.shields.io/badge/Code-Modern%20C++-blue"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                    </p>
                    <p dir="auto">
                        <a href="https://github.com/HengXin666">
                            <img
                                src="https://camo.githubusercontent.com/41902cb0326df49a67405a1c86afbfb06fd432d25edbef58c06b0fd0db0ec9ac/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4769744875622d48656e6758696e3636362d626c75653f6c6f676f3d676974687562"
                                alt="GitHub"
                                data-canonical-src="https://img.shields.io/badge/GitHub-HengXin666-blue?logo=github"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                        <a href="https://space.bilibili.com/478917126" rel="nofollow">
                            <img
                                src="https://camo.githubusercontent.com/fecefb00c8fcf1a9e97758faa2236201d9301b0012391070d27f2faab14ff5fc/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f2545352539332539342545352539332541392545352539332539342545352539332541392d48656e675f5f58696e2d70696e6b3f6c6f676f3d62696c6962696c69"
                                alt="Bilibili"
                                data-canonical-src="https://img.shields.io/badge/%E5%93%94%E5%93%A9%E5%93%94%E5%93%A9-Heng__Xin-pink?logo=bilibili"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/7ff5acd88c8249dcf1d2ea0cceffff78c2bc7aef7150fc7cee3137c5be15c233/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f51512d3238323030303530302d677265656e3f6c6f676f3d74656e63656e747171"
                        >
                            <img
                                src="https://camo.githubusercontent.com/7ff5acd88c8249dcf1d2ea0cceffff78c2bc7aef7150fc7cee3137c5be15c233/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f51512d3238323030303530302d677265656e3f6c6f676f3d74656e63656e747171"
                                alt="QQ"
                                data-canonical-src="https://img.shields.io/badge/QQ-282000500-green?logo=tencentqq"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                        <a href="https://leetcode.cn/u/heng_xin/" rel="nofollow">
                            <img
                                src="https://camo.githubusercontent.com/cd1da1f88f31e2b1eb9700cfc98e750800424e131ac03e110c49a8be010b6f7a/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4c656574436f64652d48656e675f5f58696e2d7267622839392c30302c3939293f6c6f676f3d6c656574636f6465"
                                alt="LeetCode"
                                data-canonical-src="https://img.shields.io/badge/LeetCode-Heng__Xin-rgb(99,00,99)?logo=leetcode"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/90c14675730ad6338b5025083b1ac6f75da3b63eb58d667b7b4694d772d1ef90/68747470733a2f2f6b6f6d617265762e636f6d2f67687076632f3f757365726e616d653d48656e6758696e3636362661626272657669617465643d7472756526636f6c6f723d79656c6c6f77"
                        >
                            <img
                                src="https://camo.githubusercontent.com/90c14675730ad6338b5025083b1ac6f75da3b63eb58d667b7b4694d772d1ef90/68747470733a2f2f6b6f6d617265762e636f6d2f67687076632f3f757365726e616d653d48656e6758696e3636362661626272657669617465643d7472756526636f6c6f723d79656c6c6f77"
                                alt="Profile Views"
                                data-canonical-src="https://komarev.com/ghpvc/?username=HengXin666&abbreviated=true&color=yellow"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                        <a
                            target="_blank"
                            rel="noopener noreferrer nofollow"
                            href="https://camo.githubusercontent.com/b1d547e9282d29938bdbdc285dec0728d5086fe437d9519f51a5ee6ccc33519b/68747470733a2f2f77616b6174696d652e636f6d2f62616467652f757365722f32656162653238612d626261322d346436382d393332612d3465613433356264386463332e737667"
                        >
                            <img
                                src="https://camo.githubusercontent.com/b1d547e9282d29938bdbdc285dec0728d5086fe437d9519f51a5ee6ccc33519b/68747470733a2f2f77616b6174696d652e636f6d2f62616467652f757365722f32656162653238612d626261322d346436382d393332612d3465613433356264386463332e737667"
                                alt="WakaTime Badge"
                                data-canonical-src="https://wakatime.com/badge/user/2eabe28a-bba2-4d68-932a-4ea435bd8dc3.svg"
                                style={{ maxWidth: "100%" }}
                            />
                        </a>
                    </p>
                </div>
            </BlogWithCats>
        </Layout >
    );
}
