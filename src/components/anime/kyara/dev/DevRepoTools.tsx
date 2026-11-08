import React from "react";
import { Reorder } from "framer-motion";
import type { KyaraData, KyaraImage, KyaraMeta } from "@site/src/utils/kyara/types";
import { kyaraRepo } from "./devClient";
import { seqLabel } from "../Lightbox";
import styles from "../Kyara.module.css";

interface Props {
    name: string;
    /** 仓库图 (src 已带 baseUrl) */
    images: KyaraImage[];
    meta: KyaraMeta;
    baseUrl: string;
    /** 写盘后拿到的最新目录内容 (不等热重扫) */
    onData: (data: KyaraData) => void;
}

const idsText = (ids?: number[]): string => (ids ?? []).join(", ");
const parseIds = (s: string): number[] => [...new Set(s.split(/[\s,，]+/).map((x) => Number.parseInt(x, 10)).filter((n) => Number.isInteger(n) && n > 0))];

/**
 * 本机 dev 下的仓库管理: 仓库图拖拽排序 (重写序号) / 删除 / 设封面, 以及编辑 kyara.json.
 * 全部经 plugins/kyara-gallery-plugin.mjs 的写入通道落盘, 写完立即从 dev 清单刷新页面.
 * .agents/notes/implemented/feature/2026-11-06-anime-kyara-pages.md
 */
export default function DevRepoTools ({ name, images, meta, baseUrl, onData }: Props): React.ReactElement {
    const [order, setOrder] = React.useState<KyaraImage[]>(images);
    React.useEffect(() => setOrder(images), [images]);
    const [busy, setBusy] = React.useState<string | null>(null);
    const [msg, setMsg] = React.useState<{ kind: "ok" | "error"; text: string } | null>(null);
    const [confirmDel, setConfirmDel] = React.useState<string | null>(null);
    const [form, setForm] = React.useState({
        displayName: meta.displayName,
        work: meta.work ?? "",
        accent: meta.accent ?? "",
        bangumiCharacterId: meta.bangumiCharacterId ? String(meta.bangumiCharacterId) : "",
        bangumiSubjectIds: idsText(meta.bangumiSubjectIds),
    });
    React.useEffect(() => {
        setForm({
            displayName: meta.displayName,
            work: meta.work ?? "",
            accent: meta.accent ?? "",
            bangumiCharacterId: meta.bangumiCharacterId ? String(meta.bangumiCharacterId) : "",
            bangumiSubjectIds: idsText(meta.bangumiSubjectIds),
        });
    }, [meta]);

    const run = async (label: string, job: () => Promise<string>): Promise<void> => {
        setBusy(label);
        setMsg(null);
        try {
            const text = await job();
            const fresh = await kyaraRepo.read(baseUrl, name);
            if (fresh) onData(fresh);
            setMsg({ kind: "ok", text });
        } catch (e) {
            setMsg({ kind: "error", text: `${label}失败: ${(e as Error).message}` });
        } finally {
            setBusy(null);
        }
    };

    const commitOrder = (): void => {
        const files = order.map((i) => i.file);
        if (files.join("\n") === images.map((i) => i.file).join("\n")) return;
        void run("排序", async () => {
            await kyaraRepo.reorder(name, files);
            return "仓库顺序已更新.";
        });
    };

    const saveMeta = (e: React.FormEvent): void => {
        e.preventDefault();
        const cid = Number.parseInt(form.bangumiCharacterId, 10);
        void run("保存", async () => {
            await kyaraRepo.putMeta(name, {
                displayName: form.displayName.trim() || null,
                work: form.work.trim() || null,
                accent: form.accent.trim() || null,
                bangumiCharacterId: Number.isInteger(cid) && cid > 0 ? cid : null,
                bangumiSubjectIds: parseIds(form.bangumiSubjectIds).length > 0 ? parseIds(form.bangumiSubjectIds) : null,
            });
            return "kyara.json 已保存.";
        });
    };

    const coverFile = (meta.cover ? images.find((i) => meta.cover!.endsWith(`/${encodeURIComponent(i.file)}`))?.file : undefined) ?? images[0]?.file;

    return (
        <div className={styles.repoTools}>
            <form className={styles.metaForm} onSubmit={saveMeta}>
                <label>
                    <span>显示名</span>
                    <input value={form.displayName} onChange={(e) => setForm({ ...form, displayName: e.target.value })} />
                </label>
                <label>
                    <span>作品</span>
                    <input value={form.work} onChange={(e) => setForm({ ...form, work: e.target.value })} placeholder="番剧名" />
                </label>
                <label>
                    <span>主色</span>
                    <input value={form.accent} onChange={(e) => setForm({ ...form, accent: e.target.value })} placeholder="#e86a92" />
                </label>
                <label>
                    <span>Bangumi 角色 id</span>
                    <input inputMode="numeric" value={form.bangumiCharacterId} onChange={(e) => setForm({ ...form, bangumiCharacterId: e.target.value })} />
                </label>
                <label className={styles.metaWide}>
                    <span>关联番剧 id (逗号分隔, 第一个为主作品)</span>
                    <input value={form.bangumiSubjectIds} onChange={(e) => setForm({ ...form, bangumiSubjectIds: e.target.value })} />
                </label>
                <div className={styles.metaWide}>
                    <button type="submit" className={styles.btnPrimary} disabled={!!busy}>保存 kyara.json</button>
                </div>
            </form>

            <div className={styles.localHead}>
                <span>仓库 <b>{images.length}</b> 张 · 拖动重排序号</span>
            </div>
            {order.length === 0 ? (
                <p className={styles.dim}>仓库里还没有图.</p>
            ) : (
                <Reorder.Group axis="y" values={order} onReorder={setOrder} className={styles.reorder}>
                    {order.map((img, i) => (
                        <Reorder.Item key={img.file} value={img} className={styles.reorderItem} onDragEnd={commitOrder} dragListener={!busy}>
                            <span className={styles.grip} aria-hidden>⋮⋮</span>
                            <img src={img.src} alt="" loading="lazy" decoding="async" />
                            <span className={styles.seq} data-static>{seqLabel(i + 1)}</span>
                            <span className={styles.reorderTitle}>{img.title}</span>
                            {img.author && <span className={styles.capAuthor}>@{img.author}</span>}
                            <button
                                type="button"
                                className={styles.btnGhost}
                                data-small
                                aria-pressed={coverFile === img.file}
                                disabled={!!busy || coverFile === img.file}
                                onClick={() => void run("设封面", async () => {
                                    await kyaraRepo.putMeta(name, { cover: img.file });
                                    return `封面已设为「${img.title}」.`;
                                })}
                            >
                                {coverFile === img.file ? "封面" : "设为封面"}
                            </button>
                            <button
                                type="button"
                                className={styles.iconBtn}
                                data-danger={confirmDel === img.file ? "true" : undefined}
                                aria-label={confirmDel === img.file ? `确认删除 ${img.title}` : `删除 ${img.title}`}
                                disabled={!!busy}
                                onBlur={() => setConfirmDel((c) => (c === img.file ? null : c))}
                                onClick={() => {
                                    if (confirmDel !== img.file) {
                                        setConfirmDel(img.file);
                                        return;
                                    }
                                    setConfirmDel(null);
                                    void run("删除", async () => {
                                        await kyaraRepo.deleteFile(name, img.file);
                                        return `已删除「${img.title}」.`;
                                    });
                                }}
                            >
                                {confirmDel === img.file ? "确认" : "✕"}
                            </button>
                        </Reorder.Item>
                    ))}
                </Reorder.Group>
            )}
            {(busy || msg) && (
                <p className={styles.dialogNotice} role="status" data-kind={msg?.kind}>{busy ? `${busy}中…` : msg?.text}</p>
            )}
        </div>
    );
}
