import React from "react";
import { motion } from "framer-motion";
import { hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import { kyaraResolvers } from "@site/src/utils/kyara/resolvers";
import type { KyaraImage, KyaraImportItem, KyaraLocalStore, KyaraSourceResolver } from "@site/src/utils/kyara/types";
import type { KyaraDevModule } from "./devChunks";
import { useDialogFocus } from "./Lightbox";
import styles from "./Kyara.module.css";

type Target = "local" | "repo";

interface Row {
    url: string;
    resolver: KyaraSourceResolver["id"] | null;
    state: "pending" | "ok" | "server" | "error";
    items: KyaraImportItem[];
    error?: string;
}

const splitUrls = (text: string): string[] =>
    [...new Set(text.split(/[\s,]+/).map((s) => s.trim()).filter((s) => /^https?:\/\//i.test(s)))];

interface Props {
    name: string;
    /** 本地库 (IndexedDB) */
    store: KyaraLocalStore;
    /** 本机 dev 下的仓库写入能力; 线上为 null */
    dev: KyaraDevModule | null;
    onClose: () => void;
    /** 粘贴进面板的链接, 预填到输入框 */
    initialText?: string;
    /** 写进仓库的条目 (dev), 由调用方提示「等 dev 热更新」 */
    onRepoImported?: (images: KyaraImage[]) => void;
}

/**
 * 从链接导入: 粘贴多行 URL -> 逐条匹配解析器 (pixiv / x / direct) -> 预览 (标题可改) -> 确认.
 * 浏览器里 pixiv 会被 CORS / Referer 挡住: dev 下交给服务端导入 (devChunks), 线上提示先把图存下来再拖进本地.
 * x / 直链能在浏览器里拿到就进本地 IndexedDB; 拿不到 (CORS) 且在 dev 下, 同样回落到服务端导入.
 */
export default function ImportDialog ({ name, store, dev, onClose, initialText = "", onRepoImported }: Props): React.ReactElement {
    const reduced = useHxReducedMotion();
    const ref = React.useRef<HTMLDivElement>(null);
    const [text, setText] = React.useState(initialText);
    const [rows, setRows] = React.useState<Row[]>([]);
    const [busy, setBusy] = React.useState<"parse" | "commit" | null>(null);
    const [target, setTarget] = React.useState<Target>("local");
    const [notice, setNotice] = React.useState<string | null>(null);

    useDialogFocus(ref, () => {
        if (!busy) onClose();
    });

    const parse = async (): Promise<void> => {
        const urls = splitUrls(text);
        if (urls.length === 0) {
            setNotice("没有识别到 http(s) 链接, 一行一个就好.");
            return;
        }
        setNotice(null);
        setBusy("parse");
        const initial: Row[] = urls.map((url) => {
            const r = kyaraResolvers.find((x: KyaraSourceResolver) => x.match(url)) ?? null;
            return { url, resolver: r?.id ?? null, state: r ? "pending" : "error", items: [], error: r ? undefined : "不认识的链接 (支持 pixiv / x.com / 图片直链)" };
        });
        setRows(initial);
        const done = await Promise.all(initial.map(async (row): Promise<Row> => {
            if (row.state === "error") return row;
            if (row.resolver === "pixiv") {
                return dev
                    ? { ...row, state: "server" }
                    : { ...row, state: "error", error: "pixiv 的原图不允许在网页里直接下载. 先在 pixiv 上保存图片, 再拖进本地面板." };
            }
            const resolver = kyaraResolvers.find((x: KyaraSourceResolver) => x.id === row.resolver)!;
            try {
                const items = await resolver.resolve(row.url);
                if (items.length === 0) return { ...row, state: "error", error: "这条链接里没有图片" };
                return { ...row, state: "ok", items };
            } catch (e) {
                if (dev) return { ...row, state: "server" };
                return { ...row, state: "error", error: `浏览器里拿不到: ${(e as Error).message || "网络或跨域限制"}` };
            }
        }));
        setRows(done);
        setBusy(null);
    };

    const editTitle = (ri: number, ii: number, title: string): void => {
        setRows((rs) => rs.map((r, i) => (i !== ri ? r : { ...r, items: r.items.map((it, j) => (j === ii ? { ...it, title } : it)) })));
    };

    const ready = rows.filter((r) => r.state === "ok" || r.state === "server");
    const count = ready.reduce((s, r) => s + (r.state === "ok" ? r.items.length : 1), 0);

    const commit = async (): Promise<void> => {
        setBusy("commit");
        setNotice(null);
        const errors: string[] = [];
        const repoAdded: KyaraImage[] = [];
        let localAdded = 0;
        try {
            const toLocal = target === "local" ? ready.filter((r) => r.state === "ok").flatMap((r) => r.items) : [];
            if (toLocal.length > 0) {
                const added = await store.add(name, toLocal);
                localAdded = added.length;
            }
            // 服务端导入: pixiv / 浏览器拿不到的链接, 以及选了「写进仓库」时的全部链接
            const toServer = dev ? ready.filter((r) => r.state === "server" || target === "repo") : [];
            for (const r of toServer) {
                try {
                    repoAdded.push(...(await dev!.kyaraRepo.importUrl(name, r.url)));
                } catch (e) {
                    errors.push(`${r.url}: ${(e as Error).message}`);
                }
            }
        } catch (e) {
            errors.push((e as Error).message);
        }
        setBusy(null);
        if (repoAdded.length > 0) onRepoImported?.(repoAdded);
        if (errors.length > 0) {
            setNotice(`本地 ${localAdded} 张, 仓库 ${repoAdded.length} 张; 失败: ${errors.join("; ")}`);
            return;
        }
        onClose();
    };

    const t = reduced ? { duration: 0 } : { duration: 0.32, ease: hxEase };

    return (
        <motion.div className={styles.dialogLayer} initial={{ opacity: 0 }} animate={{ opacity: 1 }} exit={{ opacity: 0 }} transition={t}>
            <button type="button" className={styles.lbBackdrop} aria-label="关闭" tabIndex={-1} onClick={() => !busy && onClose()} />
            <motion.div
                ref={ref}
                className={styles.dialog}
                role="dialog"
                aria-modal="true"
                aria-labelledby="kyara-import-title"
                tabIndex={-1}
                initial={reduced ? false : { opacity: 0, y: 16, scale: 0.98 }}
                animate={{ opacity: 1, y: 0, scale: 1 }}
                exit={reduced ? { opacity: 0 } : { opacity: 0, y: 12, scale: 0.98 }}
                transition={t}
            >
                <header className={styles.dialogHead}>
                    <h2 id="kyara-import-title">从链接导入</h2>
                    <button type="button" className={styles.lbClose} onClick={onClose} disabled={!!busy} aria-label="关闭 (Esc)">✕</button>
                </header>

                <label className={styles.fieldLabel} htmlFor="kyara-import-urls">一行一个链接: pixiv 作品页、x.com / twitter 推文、或图片直链</label>
                <textarea
                    id="kyara-import-urls"
                    data-autofocus
                    className={styles.textarea}
                    rows={4}
                    value={text}
                    onChange={(e) => setText(e.target.value)}
                    placeholder={"https://www.pixiv.net/artworks/12345678\nhttps://x.com/someone/status/1234567890"}
                    spellCheck={false}
                />

                <div className={styles.dialogRow}>
                    <button type="button" className={styles.btn} onClick={parse} disabled={!!busy || text.trim() === ""}>
                        {busy === "parse" ? "解析中…" : "解析"}
                    </button>
                    {dev && (
                        <fieldset className={styles.segment} aria-label="导入到">
                            {(["local", "repo"] as Target[]).map((k) => (
                                <label key={k} data-on={target === k ? "true" : undefined}>
                                    <input type="radio" name="kyara-target" value={k} checked={target === k} onChange={() => setTarget(k)} />
                                    {k === "local" ? "本地" : "写进仓库"}
                                </label>
                            ))}
                        </fieldset>
                    )}
                </div>

                {rows.length > 0 && (
                    <ol className={styles.preview} aria-label="解析结果" aria-busy={busy === "parse"}>
                        {rows.map((r, ri) => (
                            <li key={r.url} className={styles.previewRow} data-state={r.state}>
                                <div className={styles.previewHead}>
                                    <span className={styles.resolverTag}>{r.resolver ?? "?"}</span>
                                    <span className={styles.previewUrl} title={r.url}>{r.url}</span>
                                    <span className={styles.previewState}>
                                        {r.state === "pending" && "解析中…"}
                                        {r.state === "ok" && `${r.items.length} 张`}
                                        {r.state === "server" && "由本机 dev 服务下载进仓库"}
                                        {r.state === "error" && "无法导入"}
                                    </span>
                                </div>
                                {r.error && <p className={styles.previewError}>{r.error}</p>}
                                {r.items.length > 0 && (
                                    <ul className={styles.previewItems}>
                                        {r.items.map((it, ii) => (
                                            <li key={it.imageUrl}>
                                                <img src={it.imageUrl} alt="" loading="lazy" referrerPolicy="no-referrer" />
                                                <input
                                                    aria-label="标题"
                                                    value={it.title}
                                                    onChange={(e) => editTitle(ri, ii, e.target.value)}
                                                />
                                                <span className={styles.capAuthor}>@{it.author}</span>
                                            </li>
                                        ))}
                                    </ul>
                                )}
                            </li>
                        ))}
                    </ol>
                )}

                {notice && <p className={styles.dialogNotice} role="status">{notice}</p>}

                <footer className={styles.dialogFoot}>
                    <span className={styles.dim}>{target === "repo" && dev ? "按下一个序号写进仓库, 命名为 NNN-标题@作者.ext" : "存进这个浏览器的本地画集, 排在已有图片之后"}</span>
                    <button type="button" className={styles.btnGhost} onClick={onClose} disabled={!!busy}>取消</button>
                    <button type="button" className={styles.btnPrimary} onClick={commit} disabled={!!busy || count === 0}>
                        {busy === "commit" ? "导入中…" : `导入 ${count} 项`}
                    </button>
                </footer>
            </motion.div>
        </motion.div>
    );
}
