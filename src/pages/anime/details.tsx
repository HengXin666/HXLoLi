import React from "react";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";
import { useLocation } from "react-router-dom";
import useBaseUrl from "@docusaurus/useBaseUrl";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import BlogWithCats from "@site/src/components/BlogWithCats";
import { useActorMap, useAnimeRecords } from "@site/src/utils/anime/animeStore";
import { toAnimeCdnUrl } from "@site/src/utils/cdn/linkJsDelivr";
import { ANiMeRecord, EpisodeType, WatchStatus } from "@site/src/utils/anime/types";
import styles from './AnimeDetailPage.module.css';
import MDXA from "@site/src/theme/MDXComponents/A";
import Tooltip from "@site/src/components/common/Tooltip";
import { formatMmSs } from "@site/src/utils/formatTime";
import { FaCodeBranch, FaRegGem } from "react-icons/fa";
import Tag from "@site/src/components/anime/Tag";
import HXGiscus from "@site/src/components/Giscus";
import useQuery from "@site/src/utils/url/useQuery";
import { PageMetadata } from "@docusaurus/theme-common";
import ZoomImage from "@site/src/components/common/ZoomImage";


// 将观看状态的枚举值转换为可读的文本
function getWatchStatusText (status: WatchStatus): string {
    switch (status) {
        case WatchStatus.WANT_TO_WATCH:
            return "想看";
        case WatchStatus.WATCHED:
            return "看过";
        case WatchStatus.WATCHING:
            return "在看";
        case WatchStatus.ON_HOLD:
            return "搁置";
        case WatchStatus.DROPPED:
            return "抛弃";
        default:
            return "未知";
    }
}

// 格式化番剧剧集type
function getEpisodeTypeText (type: EpisodeType): string {
    switch (type) {
        case EpisodeType.NORMAL:
            return "正片";
        case EpisodeType.SP:
            return "SP";
        case EpisodeType.OP:
            return "OP";
        case EpisodeType.ED:
            return "ED";
        case EpisodeType.PROMO:
            return "PV";
        case EpisodeType.OTHER:
            return "其他";
        default:
            return "未知";
    }
}

const RelationList = ({ record }: { record: ANiMeRecord }) => {
    const { siteConfig } = useDocusaurusContext();

    // 1. 定义期望的排序顺序
    const sortOrder = [
        "前传",
        "续集",
        "番外篇",
        "片头曲",
        "片尾曲",
        "书籍",
        "游戏",
        "角色曲",
        "广播剧",
    ];

    // 2. 数据分组 (与之前相同)
    const groupedRelations = record.anime_data.relations.reduce((acc: Record<string, typeof record.anime_data.relations>, rel) => {
        const key = rel.relation;
        if (!acc[key]) {
            acc[key] = [];
        }
        acc[key].push(rel);
        return acc;
    }, {});

    // 3. 核心: 获取分组的 key, 并根据我们定义的 sortOrder 进行排序
    const sortedGroupKeys = Object.keys(groupedRelations).sort((a, b) => {
        const indexA = sortOrder.indexOf(a);
        const indexB = sortOrder.indexOf(b);

        // 如果某个 key 不在我们的排序数组中, 就把它放到最后
        const effectiveIndexA = indexA === -1 ? Infinity : indexA;
        const effectiveIndexB = indexB === -1 ? Infinity : indexB;

        return effectiveIndexA - effectiveIndexB;
    });

    // 4. 渲染: 遍历排序后的 keys 数组, 而不是直接遍历对象
    return (
        <div className={styles.relationContainer}>
            {sortedGroupKeys.map(relation => {
                const items = groupedRelations[relation];
                return (
                    <section key={relation} className={styles.relationGroup}>
                        <Heading as="h4" id={relation} className={styles.relationGroupTitle}>{relation}</Heading>
                        <div className={styles.relationList}>
                            {items.map(rel => (
                                <Tooltip
                                    key={relation}
                                    content={<span>{rel.name_cn || rel.name}</span>}
                                    style={{ width: "auto" }}
                                >
                                    <div key={rel.id} className={styles.relationItem}>
                                        <ZoomImage src={toAnimeCdnUrl(`/data/relation/${rel.id}.jpg`)} alt={rel.name} className={styles.relationImage} />
                                        <div className={styles.relationInfo}>
                                            <MDXA href={`${siteConfig.baseUrl}anime/details?id=${rel.id}`}>{rel.name}</MDXA>
                                        </div>
                                    </div>
                                </Tooltip>
                            ))}
                        </div>
                    </section>
                );
            })}
        </div>
    );
};

export default function AnimeDetailPage (): React.ReactElement {
    const query = useQuery();
    const id = query.get("id");
    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);
    const actorMap = useActorMap(siteConfig.baseUrl);
    const notFoundImageUrl = useBaseUrl("/anime/img/misaka-404.png");

    const record =
        id && /^\d+$/.test(id)
            ? records.find((r) => r.anime_data.id === Number(id))
            : undefined;

    if (!record) {
        return (
            <Layout title={"アニメ 404 NOT FOUND"}>
                <div style={{ height: "60px" }} />
                <BlogWithCats
                    style={{
                        backgroundColor: "#2b2b2b",
                        padding: "20px",
                        display: "flex",
                        flexDirection: "column",
                    }}
                >
                    <div style={{ textAlign: "center" }}>
                        <Heading as="h2" id="404">
                            404 NOT FOUND
                        </Heading>
                        <p>请尝试: <MDXA href={`https://bgm.tv/subject/${id}`}>{`bgm.tv/subject/${id}`}</MDXA></p>
                        <ZoomImage src={notFoundImageUrl} alt="404" />
                    </div>
                </BlogWithCats>
                <div style={{ height: "60px" }} />
            </Layout>
        );
    }

    return (
        <>
            <PageMetadata title={record.anime_data.name} description={record.anime_data.summary} />
            <Layout title={record.anime_data.name}>
                <div style={{ height: "60px" }} />
                <BlogWithCats
                    style={{
                        backgroundColor: "#2b2b2b",
                        padding: "20px",
                    }}
                >
                    <div className={styles.pageContainer}>

                        <header className={styles.headerSection}>
                            <h1 className={styles.mainTitle}>{record.anime_data.name_cn}</h1>
                            <p className={styles.subTitle}>「{record.anime_data.name}」</p>
                        </header>

                        <main className={styles.mainContent}>
                            <aside>
                                <ZoomImage src={toAnimeCdnUrl(`/data/anime/${record.anime_data.id}.jpg`)} alt={record.anime_data.name_cn} className={styles.coverImage} />
                            </aside>

                            <section className={styles.infoColumn}>
                                <div className={styles.infoBox}>
                                    <Heading as="h4" className={styles.sectionTitle} id="basic">基本信息</Heading>
                                    <p><strong>评分:</strong> {record.anime_data.score} / 10</p>
                                    <p><strong>放送日期:</strong> {record.anime_data.date ?? "未知日期"}</p>
                                    <p><strong>集数:</strong> {(() => {
                                        let cnt: number = 0;
                                        for (const item of record.anime_data.episodes) {
                                            cnt += item.type === EpisodeType.NORMAL ? 1 : 0;
                                        }
                                        return cnt;
                                    })()} 话</p>
                                    <p><FaCodeBranch /><MDXA href={`${siteConfig.baseUrl}anime/graph?id=${record.anime_data.id}`}>「番剧-角色-声优」关系图</MDXA></p>
                                </div>
                                <div className={styles.infoBox}>
                                    <Heading as="h4" className={styles.sectionTitle} id="status">我的状态</Heading>
                                    <p><strong>观看状态:</strong> {getWatchStatusText(record.user_status.watch_status)}</p>
                                    <p><strong>观看进度:</strong> {record.user_status.watched_eps} / {record.anime_data.total_episodes}</p>
                                    {record.user_status.comment && <p><strong>简评:</strong> {record.user_status.comment}</p>}
                                    <p>
                                        <strong>标签: </strong>
                                        {record.user_status.tags.length > 0
                                            ? record.user_status.tags.map(tag => <Tag text={tag} style={{ marginRight: "5px" }} />)
                                            : <Tag text="暂无标签" style={{ opacity: 0.75 }} />
                                        }
                                    </p>
                                </div>
                            </section>
                        </main>

                        <div style={{ height: "20px" }}></div>

                        <div className={styles.infoBox}>
                            <Heading as="h4" className={styles.sectionTitle} id="eps">剧集</Heading>
                            <div className={styles.episodeList}>
                                {record.anime_data.episodes
                                    // 推荐按 sort 字段排序, 确保 "SP" 等特殊集数顺序正确
                                    .sort((a, b) => a.sort - b.sort)
                                    .map(ep => (
                                        <Tooltip
                                            key={ep.id}
                                            style={{
                                                width: "500px",
                                            }}
                                            selectable={true}
                                            content={
                                                <div
                                                    style={{
                                                        display: "flex",
                                                        flexDirection: "column", // 垂直排列
                                                        gap: "8px",              // 每一行元素的间距
                                                        textAlign: "left",       // 强制左对齐
                                                        width: "100%",
                                                    }}
                                                >
                                                    {/* --- 标题区域 --- */}
                                                    <div>
                                                        <p
                                                            className={styles.tooltipTitle}
                                                            style={{
                                                                margin: 0,
                                                                fontWeight: "bold",
                                                                fontSize: "16px",
                                                            }}
                                                        >
                                                            {`第 ${ep.sort} 集・${ep.name_cn}`}
                                                        </p>

                                                        {/* 如果没有原名, 这行可以不显示, 或者保持样式 */}
                                                        {ep.name && (
                                                            <p
                                                                className={styles.tooltipTitle}
                                                                style={{
                                                                    margin: "4px 0 0 0", // 稍微和上面拉开一点距离
                                                                    fontSize: "13px",
                                                                }}
                                                            >
                                                                「{ep.name}」
                                                            </p>
                                                        )}
                                                    </div>

                                                    {/* --- Meta 信息 (标签/日期/时长) --- */}
                                                    <div
                                                        className={styles.tooltipDate}
                                                        style={{
                                                            display: "flex",
                                                            alignItems: "center", // 垂直居中对齐 Tag 和文字
                                                            flexWrap: "wrap",     // 防止内容过长溢出
                                                            margin: 0
                                                        }}
                                                    >
                                                        <Tag text={getEpisodeTypeText(ep.type)} />
                                                        <span>・放送日期 {ep.air_date || '未知'}</span>
                                                        <span>・时长 {formatMmSs(ep.duration_seconds) || '未知'}</span>
                                                    </div>

                                                    {/* --- 分割线 (可选, 增加美观度) --- */}
                                                    <div style={{ height: "1px", background: "#eee", width: "100%" }} />

                                                    {/* --- 简介 (带滚动条) --- */}
                                                    <p
                                                        className={styles.tooltipDesc}
                                                        style={{
                                                            margin: 0,
                                                            fontSize: "13px",
                                                            lineHeight: "1.6",
                                                            // 核心滚动逻辑:
                                                            maxHeight: "320px",    // 设定最大高度, 超过此高度出现滚动条
                                                            overflowY: "auto",     // 允许Y轴滚动
                                                            whiteSpace: "pre-wrap", // 保留简介中的换行符
                                                            paddingRight: "4px"    // 防止滚动条挡住文字
                                                        }}
                                                    >
                                                        <span style={{ fontWeight: "bold"}}>简介: </span>
                                                        {ep.desc || '暂无简介'}
                                                    </p>
                                                </div>
                                            }
                                        >
                                            <div className={styles.episodeItem} tabIndex={0}>
                                                <span className={styles.episodeNumber}>{ep.sort}</span>
                                            </div>
                                        </Tooltip>
                                    ))
                                }
                            </div>
                        </div>

                        <div style={{ height: "20px" }}></div>

                        <section>
                            <Heading as="h3" className={styles.sectionTitle} id="summary">剧情简介</Heading>
                            <p className={styles.summaryText}>{record.anime_data.summary}</p>
                        </section>

                        {/* --- 角色介绍 --- */}
                        <section>
                            <Heading as="h3" className={styles.sectionTitle} id="characters">角色介绍</Heading>
                            <div className={styles.characterScrollContainer}>
                                {record.anime_data.characters.map(char => (
                                    <div key={char.id} className={styles.characterCard}>
                                        <Tooltip
                                            key={char.id}
                                            style={{
                                                width: "auto",
                                                textAlign: "center"
                                            }}
                                            content={
                                                <>
                                                    {(() => {
                                                        if (char.name_cn && (char.birth_month && char.birth_day)) {
                                                            return (
                                                                <span>
                                                                    「{char.name_cn}」 <br />
                                                                    生日: {char.birth_month && char.birth_day && `${char.birth_month}月${char.birth_day}日`}
                                                                </span>
                                                            )
                                                        } else if (char.name_cn) {
                                                            return (
                                                                <span>
                                                                    「{char.name_cn}」
                                                                </span>
                                                            )
                                                        } else if (char.birth_month && char.birth_day) {
                                                            return (
                                                                <span>
                                                                    生日: {char.birth_month}月{char.birth_day}日
                                                                </span>
                                                            )
                                                        } else {
                                                            return (
                                                                <span>
                                                                    {char.name}
                                                                </span>
                                                            )
                                                        }
                                                    })()}
                                                    <div style={{ maxWidth: "500px", textAlign: "left", color: "#b3b3b3" }}>
                                                        {char.summary}
                                                    </div>
                                                </>
                                            }>
                                            <a href={`https://bgm.tv/character/${char.id}`} target="_blank" className={styles.characterImageLink} title={char.name}>
                                                <span
                                                    className={styles.characterImageSpan}
                                                    style={{ backgroundImage: `url(${toAnimeCdnUrl(`/data/kyara/${char.id}.jpg`)})` }}
                                                ></span>
                                            </a>
                                        </Tooltip>
                                        <p className={styles.characterName}>{char.name}</p>
                                        <p className={styles.characterRelation}>{char.relation}</p>
                                        <p className={styles.characterActors}>
                                            CV {char.actor_ids
                                                .map((actorId, idx) => {
                                                    const actor = actorMap.get(actorId);
                                                    return actor ? (
                                                        <>
                                                            {idx === 0 ? null : <><br /> / </>}
                                                            <Tooltip
                                                                style={{ width: "500px" }}
                                                                selectable={true}
                                                                content={
                                                                    <div
                                                                        style={{
                                                                            display: "flex",         // 启用 Flex 布局
                                                                            alignItems: "flex-start", // 顶部对齐
                                                                            gap: "12px",             // 左右栏之间的间距
                                                                            textAlign: "left"        // 确保文字默认左对齐
                                                                        }}
                                                                    >
                                                                        {/* --- 左侧栏: 图片 + 名字 --- */}
                                                                        <div
                                                                            style={{
                                                                                display: "flex",
                                                                                flexDirection: "column", // 竖向排列
                                                                                alignItems: "center",    // 水平居中 (让名字在图片下方居中)
                                                                                width: "160px",          // 固定宽度, 与图片一致
                                                                                flexShrink: 0            // 防止被右侧内容挤压
                                                                            }}
                                                                        >
                                                                            <ZoomImage
                                                                                src={toAnimeCdnUrl(`/data/cv/${actor.id}.jpg`)}
                                                                                alt={actor.name}
                                                                                loading="lazy"
                                                                                decoding="async"
                                                                                style={{
                                                                                    width: "100%",
                                                                                    height: "auto",
                                                                                    objectFit: "cover",
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
                                                                                <MDXA href={`https://bgm.tv/person/${actor.id}`}>{`「${actor.name}」`}</MDXA>
                                                                            </p>
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
                                                                                <MDXA href={`${siteConfig.baseUrl}anime/cv?id=${actor.id}`}>出演番剧</MDXA>
                                                                            </p>
                                                                        </div>

                                                                        {/* --- 右侧栏: 简介 --- */}
                                                                        <div
                                                                            style={{
                                                                                flex: 1,      // 占据剩余空间
                                                                                minWidth: 0   // 关键: 防止 Flex 子项内容溢出容器
                                                                            }}
                                                                        >
                                                                            <p
                                                                                className={styles.tooltipDesc}
                                                                                style={{
                                                                                    margin: 0,             // 去除默认边距, 对齐顶部
                                                                                    maxHeight: "320px",    // 限制高度 (根据左侧高度大概调整)
                                                                                    overflowY: "auto",     // 超出滚动
                                                                                    fontSize: "13px",
                                                                                    lineHeight: "1.5",
                                                                                    whiteSpace: "pre-wrap" // 保留换行符(如果有)
                                                                                }}
                                                                            >
                                                                                {actor.short_summary || "暂无简介"}
                                                                            </p>
                                                                        </div>
                                                                    </div>
                                                                }
                                                            >
                                                                <span>{actor.name}</span>
                                                            </Tooltip>
                                                        </>
                                                    ) : null;
                                                })
                                                .filter(Boolean)}
                                        </p>
                                    </div>
                                ))}
                            </div>
                        </section>

                        <div style={{ height: "20px" }}></div>

                        <section>
                            <Heading as="h3" className={styles.sectionTitle} id="relations">关联条目</Heading>
                            <RelationList record={record} />
                        </section>

                        <div style={{ textAlign: "center" }}>
                            <FaRegGem /> 数据来源: <MDXA href={`https://bgm.tv/subject/${record.anime_data.id}`}>Bangumi.tv</MDXA>
                        </div>
                    </div>
                    <HXGiscus term={`anime/details?id=${record.anime_data.id}「${record.anime_data.name}」`} />
                </BlogWithCats>
                <div style={{ height: "60px" }} />
            </Layout>
        </>
    );
}
