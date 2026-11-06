import React from "react";
import { Numeral, Rise } from "@site/src/components/anime/stage/Stage";
import styles from "./Kyara.module.css";

const SAMPLE = [
    ["001", "初见", "画师A", "png"],
    ["002", "夏日祭", "画师B", "jpg"],
    ["003", "雨天", "", "webp"],
];

/**
 * kyara 还没有图时的上墙说明: 左边是目录示意 (文件名即顺序 / 标题 / 作者), 右边三种上墙方式.
 * 不传 name 时 (索引页) 用示例角色名.
 */
export default function KyaraGuide ({ name }: { name?: string }): React.ReactElement {
    const dir = name || "春日野穹";
    return (
        <Rise as="section" className={styles.guide}>
            <div className={styles.guideTree} aria-label="目录示意">
                <p className={styles.guideTreeHead}><span /><span /><span />static/img/kyara/{dir}/</p>
                <ul>
                    {SAMPLE.map(([seq, title, author, ext]) => (
                        <li key={seq}>
                            <b>{seq}</b><i>-</i><em>{title}</em>{author ? <><i>@</i><u>{author}</u></> : null}<i>.{ext}</i>
                        </li>
                    ))}
                    <li className={styles.guideMeta}><b>kyara.json</b><i> 可选: 显示名 / 作品 / 主色</i></li>
                </ul>
                <p className={styles.guideLegend}><b>序号</b> 决定顺序 · <em>标题</em> · <u>作者</u> (可省)</p>
            </div>
            <ol className={styles.guideSteps}>
                <li>
                    <Numeral n={1} />
                    <div>
                        <h3>放进仓库</h3>
                        <p>按上面的命名把图放进 <code>static/img/kyara/{dir}/</code>, 构建时自动生成这一页, 按序号渲染.</p>
                    </div>
                </li>
                <li>
                    <Numeral n={2} />
                    <div>
                        <h3>本地模式</h3>
                        <p>打开右上角「本地模式」, 直接拖图或粘贴. 图只存在这台浏览器里, 可以导出成命名好的 zip.</p>
                    </div>
                </li>
                <li>
                    <Numeral n={3} />
                    <div>
                        <h3>贴链接导入</h3>
                        <p>粘贴 pixiv 作品页或 x.com 推文链接, 自动解析出原图、标题和作者; 本机 dev 下还能一键写进仓库.</p>
                    </div>
                </li>
            </ol>
        </Rise>
    );
}
