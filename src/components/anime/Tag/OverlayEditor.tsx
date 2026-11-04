import React from "react";
import Select from "@site/src/components/common/Tooltip/Select";
import { getOverlayEntry, setOverlayEntry } from "@site/src/utils/anime/animeStore";
import { STATUS_OPTIONS } from "@site/src/utils/anime/filters";
import { AnimeOverlayEntry, sanitizeOverlayEntry } from "@site/src/utils/anime/overlay";
import { getRecordTags, susumeDefOfTag } from "@site/src/utils/anime/tags";
import { ANiMeRecord, WatchStatus } from "@site/src/utils/anime/types";
import styles from "./OverlayEditor.module.css";

/**
 * 本人标注编辑器, **只在 dev 下存在**: 由 src/components/home/lazyChunks.js 按 NODE_ENV 常量导出,
 * 生产构建里那个 import() 在 webpack 解析阶段就被裁掉, 本文件与写入地址都不会进包.
 * 保存 = PUT /__hx/anime-overlay/<id> (plugins/anime-overlay-plugin.mjs 写 data/animeOverlay.json), 删除 = DELETE.
 * 只提交"已有 overlay 的字段"与"改过的字段", 没碰的字段继续跟随 CDN.
 * 2026-11-04 tag 重构: 这里**不再编辑标签** (animeOverlay.json 的 tags 已废弃不读), 只读展示人工标注 data/labels/<id>.json;
 * 改标签走 HXLoLi-ANiMe 的标注 PR / issue (hx-anime-tag-refactor-design.md 第 5-6 节). 保存时旧的 tags 字段不再写回.
 * .agents/notes/implemented/feature/2026-11-01-anime-local-overlay.md
 */
const ENDPOINT = "/__hx/anime-overlay/";
const RATE_OPTIONS = [{ value: 0, label: "未评分" }, ...Array.from({ length: 10 }, (_, i) => ({ value: 10 - i, label: `${10 - i} 分` }))];
const STATUS_SELECT = STATUS_OPTIONS.map((o) => ({ value: o.value as number, label: o.label }));

interface Form {
    watch_status: number;
    watched_eps: number;
    rate: number;
    comment: string;
}

function formOf (record: ANiMeRecord): Form {
    return {
        watch_status: record.user_status.watch_status,
        watched_eps: record.user_status.watched_eps,
        rate: record.user_status.rate ?? 0,
        comment: record.user_status.comment ?? "",
    };
}

async function call (method: "PUT" | "DELETE", id: number, body?: AnimeOverlayEntry): Promise<AnimeOverlayEntry | null> {
    const res = await fetch(`${ENDPOINT}${id}`, {
        method,
        headers: { "Content-Type": "application/json", "X-HX-Overlay": "1" },
        body: body ? JSON.stringify(body) : undefined,
    });
    const json = await res.json().catch(() => ({ ok: false, error: `HTTP ${res.status}` }));
    if (!res.ok || !json.ok) throw new Error(json.error ?? `HTTP ${res.status}`);
    return json.entry ? sanitizeOverlayEntry(json.entry) : null;
}

export default function OverlayEditor ({ record }: { record: ANiMeRecord }): React.ReactElement {
    const id = record.anime_data.id;
    const initial = React.useMemo(() => formOf(record), [record]);
    const [form, setForm] = React.useState<Form>(initial);
    const [busy, setBusy] = React.useState(false);
    const [msg, setMsg] = React.useState<{ ok: boolean; text: string } | null>(null);
    const existing = getOverlayEntry(id);
    const labelTags = getRecordTags(record);

    React.useEffect(() => { setForm(initial); }, [initial]);

    const patch = (p: Partial<Form>) => { setForm((f) => ({ ...f, ...p })); setMsg(null); };

    const body = (): AnimeOverlayEntry => {
        const out: AnimeOverlayEntry = {};
        const keep = (k: keyof AnimeOverlayEntry) => existing?.[k] !== undefined;
        if (keep("watch_status") || form.watch_status !== initial.watch_status) out.watch_status = form.watch_status as WatchStatus;
        if (keep("watched_eps") || form.watched_eps !== initial.watched_eps) out.watched_eps = form.watched_eps;
        if (form.rate > 0 && (keep("rate") || form.rate !== initial.rate)) out.rate = form.rate;
        if (keep("comment") || form.comment !== initial.comment) out.comment = form.comment;
        return out;
    };

    const save = async () => {
        const b = body();
        if (Object.keys(b).length === 0) { setMsg({ ok: true, text: "没有改动" }); return; }
        setBusy(true);
        try {
            const entry = await call("PUT", id, b);
            setOverlayEntry(id, entry);
            setMsg({ ok: true, text: "已写入 data/animeOverlay.json" });
        } catch (e) {
            setMsg({ ok: false, text: `保存失败: ${e instanceof Error ? e.message : String(e)}` });
        } finally { setBusy(false); }
    };

    const remove = async () => {
        setBusy(true);
        try {
            await call("DELETE", id);
            setOverlayEntry(id, null);
            setMsg({ ok: true, text: "已删除这部番的 overlay, 回到 CDN 数据" });
        } catch (e) {
            setMsg({ ok: false, text: `删除失败: ${e instanceof Error ? e.message : String(e)}` });
        } finally { setBusy(false); }
    };

    const dirty = Object.keys(body()).some((k) => {
        const b = body() as Record<string, unknown>;
        return JSON.stringify(b[k]) !== JSON.stringify((existing as Record<string, unknown> | undefined)?.[k]);
    });

    return (
        <section className={styles.box} aria-labelledby="overlay-title">
            <header className={styles.head}>
                <h4 id="overlay-title" className={styles.title}>本地标注</h4>
                <span className={styles.badge}>仅 dev</span>
                {existing && <span className={styles.on} title={existing.updated_at}>已有 overlay</span>}
            </header>

            <div className={styles.grid}>
                <span className={styles.label} id="ov-status">观看状态</span>
                <Select<number> ariaLabelledBy="ov-status" value={form.watch_status} options={STATUS_SELECT} onChange={(v) => patch({ watch_status: v })} />

                <label className={styles.label} htmlFor="ov-eps">进度</label>
                <span className={styles.inline}>
                    <input
                        id="ov-eps"
                        className={styles.input}
                        type="number"
                        min={0}
                        max={record.anime_data.total_episodes || undefined}
                        value={form.watched_eps}
                        onChange={(e) => patch({ watched_eps: Math.max(0, Math.floor(Number(e.target.value) || 0)) })}
                    />
                    <span className={styles.muted}>/ {record.anime_data.total_episodes}</span>
                </span>

                <span className={styles.label} id="ov-rate">我的评分</span>
                <Select<number> ariaLabelledBy="ov-rate" value={form.rate} options={RATE_OPTIONS} onChange={(v) => patch({ rate: v })} />

                <span className={styles.label}>标签</span>
                <div className={styles.tags}>
                    {labelTags.length === 0 && <span className={styles.muted}>未标注</span>}
                    {labelTags.map((t) => (
                        <span key={t} className={styles.chip} title={susumeDefOfTag(t) ? "推荐分类" : "普通标注"}>{t}</span>
                    ))}
                    <span className={styles.muted}>只读: 来自 HXLoLi-ANiMe data/labels, 改标签请走标注 PR</span>
                </div>

                <label className={styles.label} htmlFor="ov-comment">评论</label>
                <textarea id="ov-comment" className={styles.textarea} rows={3} value={form.comment} onChange={(e) => patch({ comment: e.target.value })} />
            </div>

            <footer className={styles.actions}>
                <button type="button" className={styles.primary} disabled={busy || !dirty} onClick={save}>保存</button>
                <button type="button" className={styles.ghost} disabled={busy} onClick={() => { setForm(initial); setMsg(null); }}>还原表单</button>
                <button type="button" className={styles.danger} disabled={busy || !existing} onClick={remove}>删除 overlay</button>
                {msg && <span className={msg.ok ? styles.ok : styles.err} role="status">{msg.text}</span>}
            </footer>
        </section>
    );
}
