import React, { type ReactNode, useMemo } from "react";
import Link from "@docusaurus/Link";
import Translate, { translate } from "@docusaurus/Translate";
import { PageMetadata } from "@docusaurus/theme-common";
import Layout from "@theme/Layout";
import BlogWithCats from "@site/src/components/BlogWithCats";
import { type Variants, motion } from "framer-motion";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { useActorMap, useAnimeRecordMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import useQuery from "@site/src/utils/url/useQuery";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import styles from "./CvPage.module.css";
import Tooltip from "@site/src/components/common/Tooltip";
import MDXA from "@site/src/theme/MDXComponents/A";
import HXGiscus from "@site/src/components/Giscus";
import ZoomImage from "@site/src/components/common/ZoomImage";

// ================= 类型定义 (复用原结构以适配组件) =================

// 模拟 ArchiveBlogPost 的结构, 以便复用 Year 组件
type RolePost = {
    metadata: {
        anime_id: number;
        kyara_ids: number[];
        date: string;
    };
};

type YearProp = {
    year: string;
    posts: RolePost[];
};

// ================= 动画配置 =================

const variants: Variants = {
    from: { opacity: 0.01, y: 50 },
    to: (i) => ({
        opacity: 1,
        y: 0,
        transition: {
            type: "spring",
            damping: 25,
            stiffness: 100,
            bounce: 0.2,
            duration: 0.3,
            delay: i * 0.1,
        },
    }),
};

// ================= 辅助函数 =================

const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    if (isNaN(date.getTime())) return dateString; // 处理无效日期
    const month = String(date.getMonth() + 1).padStart(2, "0");
    const day = String(date.getDate()).padStart(2, "0");
    return `${month}-${day}`;
};

// 将扁平的角色列表按年份分组
function listRolesByYears (rolePosts: RolePost[]): YearProp[] {
    const postsByYear = rolePosts.reduceRight((posts, post) => {
        const year = post.metadata.date.split("-")[0]!;
        const yearPosts = posts.get(year) ?? [];
        return posts.set(year, [post, ...yearPosts]);
    }, new Map<string, RolePost[]>());

    return Array.from(postsByYear, ([year, posts]) => ({
        year,
        posts,
    })).sort((a, b) => b.year.localeCompare(a.year)); // 年份(最新的在上面)
}

// ================= 组件 =================

function Year ({ posts }: YearProp) {
    const { siteConfig } = useDocusaurusContext();
    const animeRecordMap = useAnimeRecordMap(siteConfig.baseUrl);
    return (
        <>
            <ul className={styles.archiveList}>
                {posts.map((post, i) => (
                    <motion.li
                        key={`${post.metadata.anime_id}-${post.metadata.kyara_ids.join("-")}`} // 加上索引避免同一个番剧多个角色时key重复
                        className={styles.archiveItem}
                        custom={i}
                        initial="from"
                        animate="to"
                        variants={variants}
                        viewport={{ once: true, amount: 0.8 }}
                    >
                        <Link to={`${siteConfig.baseUrl}anime/details?id=${post.metadata.anime_id}`}>
                            <time className={styles.archiveTime}>
                                {formatDate(post.metadata.date)}
                            </time>
                            {(() => {
                                const animeRecord = animeRecordMap.get(post.metadata.anime_id);
                                if (!animeRecord) return "未知动画";
                                return (
                                    <Tooltip
                                style={{ width: "240px" }}
                                content={
                                    <>
                                        <ZoomImage
                                            src={toAnimeCdnUrl(`/data/anime/${post.metadata.anime_id}.jpg`)}
                                            loading="lazy"
                                            decoding="async"
                                            style={{
                                                width: "240px",
                                                height: "auto",
                                                objectFit: "cover",
                                                display: "block",
                                                marginLeft: "auto",
                                                marginRight: "auto",
                                                marginBottom: "1rem",
                                                borderRadius: "4px"
                                            }}
                                        />
                                        <p
                                            style={{
                                                marginTop: "8px",     // 图片和名字的间距
                                                marginBottom: "0",    // 去除 p 标签默认下边距
                                                textAlign: "center",  // 文字居中
                                                fontSize: "14px",     // 调整字号
                                                fontWeight: "bold",
                                                lineHeight: "1.2",
                                                width: "100%",
                                            }}
                                        >
                                            「{animeRecord.anime_data.name}」
                                        </p>
                                    </>
                                }
                            >
                                <span>「{animeRecord.anime_data.name_cn || animeRecord.anime_data.name}」</span>
                            </Tooltip>
                                );
                            })()}
                        </Link>
                        {post.metadata.kyara_ids.map((kyaraId, idx) => {
                            const animeRecord = animeRecordMap.get(post.metadata.anime_id);
                            if (!animeRecord) return "未知角色";
                            const character = animeRecord.anime_data.characters.find(c => c.id === kyaraId);
                            return (
                                // 垂直居中
                                <span style={{
                                    display: "flex",
                                    alignItems: "center",
                                    color: "var(--ifm-text-color)",
                                }}>
                                    {idx > 0 && "、"}
                                    <Tooltip
                                        style={{ width: "240px" }}
                                        content={
                                            <>
                                                <ZoomImage
                                                    src={toAnimeCdnUrl(`/data/kyara/${kyaraId}.jpg`)}
                                                    loading="lazy"
                                                    decoding="async"
                                                    style={{
                                                        width: "240px",
                                                        height: "auto",
                                                        objectFit: "cover",
                                                        display: "block",
                                                        marginLeft: "auto",
                                                        marginRight: "auto",
                                                        marginBottom: "1rem",
                                                        borderRadius: "4px"
                                                    }}
                                                />
                                                <p
                                                    style={{
                                                        marginTop: "8px",     // 图片和名字的间距
                                                        marginBottom: "0",    // 去除 p 标签默认下边距
                                                        textAlign: "center",  // 文字居中
                                                        fontSize: "14px",     // 调整字号
                                                        fontWeight: "bold",
                                                        lineHeight: "1.2",
                                                        width: "100%",
                                                    }}
                                                >
                                                    <MDXA href={`https://bgm.tv/character/${kyaraId}`}>{`「${character?.name || "とあるキャラ"}」`}</MDXA>
                                                </p>
                                            </>
                                        }
                                        selectable={true}
                                    >
                                        <span key={idx}>
                                            {character?.name_cn || character?.name || "未知角色"}
                                        </span>
                                    </Tooltip>
                                </span>
                            )
                        })}
                    </motion.li>
                ))}
            </ul>
        </>
    );
}

function YearsSection ({ years }: { years: YearProp[] }) {
    return (
        <div className="margin-top--md">
            {years.map((_props, idx) => (
                <motion.div
                    key={_props.year}
                    initial="from"
                    animate="to"
                    custom={idx}
                    variants={variants}
                >
                    <div className={styles.archiveYear}>
                        <h3 className={styles.archiveYearTitle}>{_props.year}</h3>
                        <span>
                            <i>{_props.posts.reduce((acc, post) => acc + post.metadata.kyara_ids.length, 0)} </i>
                            <Translate id="theme.blog.archive.posts.unit">个角色</Translate>
                        </span>
                    </div>
                    <Year {..._props} />
                </motion.div>
            ))}
        </div>
    );
}

// ================= 主页面组件 =================

export default function CvPage (): ReactNode {
    const query = useQuery();
    const idParam = query.get("id");
    const actorId = idParam ? parseInt(idParam, 10) : null;

    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);
    const actorMap = useActorMap(siteConfig.baseUrl);

    // 获取当前声优信息
    const currentActor = actorId ? actorMap.get(actorId) : null;

    // 核心逻辑: 筛选并转换数据
    const years = useMemo(() => {
        if (!actorId || !records) {
            return [];
        }

        const allRoles: Map<number, RolePost> = new Map();

        records.forEach((record) => {
            const anime = record.anime_data;

            // 在该番剧中查找此声优配音的角色
            const matchedCharacters = anime.characters.filter((char) =>
                char.actor_ids.includes(actorId)
            );

            matchedCharacters.forEach((char) => {
                const post = allRoles.get(anime.id);
                if (post) {
                    // 更新现有条目
                    post.metadata.kyara_ids.push(char.id || 0);
                } else {
                    // 创建新条目
                    allRoles.set(anime.id, {
                        metadata: {
                            anime_id: anime.id,
                            kyara_ids: char.id ? [char.id] : [],
                            date: anime.date || "1970-01-01", // 处理缺失日期
                        },
                    });
                }
            });
        });

        // 分组
        return listRolesByYears(Array.from(allRoles.values()));
    }, [actorId, records]);

    // 页面标题配置
    const title = currentActor
        ? `「${currentActor.name}」の出演記録`
        : translate({ message: "声优档案" });

    const description = currentActor
        ? `查看声优 ${currentActor.name} 的所有出演番剧与角色列表`
        : "声优出演记录归档";

    return (
        <>
            <PageMetadata title={title} description={description} />
            <Layout>
                <header className="hero hero--primary">
                    <div className="container">
                        <h2 className={styles.archiveTitle}>{title}</h2>
                        <p>
                            {currentActor ? (
                                <>
                                    <Translate values={{ total: years.reduce((acc, y) => acc + y.posts.reduce((acc, p) => acc + p.metadata.kyara_ids.length, 0), 0) }}>
                                        {"共出演了 {total} 个角色"}
                                    </Translate>
                                    {/* 如果有声优头像也可以在这里展示 */}
                                </>
                            ) : (
                                "请在 URL 中指定 ?id=xxx 以查看声优信息"
                            )}
                        </p>
                    </div>
                </header>

                <BlogWithCats style={{
                    backgroundColor: "#2b2b2b",
                    padding: "20px",
                    display: "flex",
                    flexDirection: "column",
                    marginTop: "2rem",
                    marginBottom: "2rem",
                }}>
                    <div className="container" style={{ width: "95%" }}>
                        {currentActor &&
                            <>
                                <ZoomImage
                                    src={toAnimeCdnUrl(`/data/cv/${currentActor.id}.jpg`)}
                                    alt={currentActor.name}
                                    loading="lazy"
                                    decoding="async"
                                    style={{
                                        width: "160px",
                                        height: "auto",
                                        objectFit: "cover",
                                        display: "block",
                                        marginLeft: "auto",
                                        marginRight: "auto",
                                        marginBottom: "1rem",
                                        borderRadius: "4px"
                                    }}
                                />
                                <h3 style={{ textAlign: "center", marginBottom: "1rem" }}>
                                    <MDXA href={`https://bgm.tv/person/${currentActor.id}`}>{`「${currentActor.name}」`}</MDXA>
                                </h3>
                            </>}
                        {years.length > 0 ? (
                            <YearsSection years={years} />
                        ) : (
                            <div className="text--center margin-top--lg">
                                {actorId
                                    ? "暂无该声优的出演记录"
                                    : "无效的声优 ID 或未指定 ID"}
                            </div>
                        )}

                        {currentActor && <HXGiscus term={`anime/cv?id=${currentActor.id} ${title}`} />}
                    </div>
                </BlogWithCats>
            </Layout>
        </>
    );
}
