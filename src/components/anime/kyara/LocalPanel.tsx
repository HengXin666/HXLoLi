import React from "react";
import { AnimatePresence, Reorder, motion } from "framer-motion";
import { HxCard, hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import { formatKyaraFilename } from "@site/src/utils/kyara/filename";
import type { KyaraImage } from "@site/src/utils/kyara/types";
import ImportDialog, { kyaraDev, type KyaraLocalApi } from "./ImportDialog";
import { SourceIcon, seqLabel } from "./Lightbox";
import styles from "./Kyara.module.css";

const IMAGE_EXT = /\.(png|jpe?g|webp|gif|avif)$/i;

const extOf = (img: KyaraImage, blob: Blob): string => {
    const m = img.file.match(IMAGE_EXT);
    if (m) return m[1].toLowerCase();
    const sub = blob.type.split("/")[1];
    return sub ? sub.replace("jpeg", "jpg").replace(/\+.*$/, "") : "png";
};

const imagesOnly = (files: FileList | File[] | null | undefined): File[] =>
    Array.from(files ?? []).filter((f) => f.type.startsWith("image/") || IMAGE_EXT.test(f.name));

interface Props {
    name: string;
    local: KyaraLocalApi;
    /** 仓库里最大序号 + 1: 写进仓库时本地条目从这里接着编号 */
    nextSeq: number;
}

/**
 * 本地模式面板: 拖拽 / 选择 / 粘贴图片进 IndexedDB, 从链接导入, 拖拽排序 (framer Reorder), 导出 zip,
 * dev 下多一个「写进仓库」(逐张 PUT /__hx/kyara/{name}/file/{NNN-标题@作者.ext}, 成功后从本地移除).
 */
export default function LocalPanel ({ name, local, nextSeq }: Props): React.ReactElement {
    const reduced = useHxReducedMotion();
    const { store, images, serverWritable } = local;
    const fileRef = React.useRef<HTMLInputElement>(null);
    const [over, setOver] = React.useState(false);
    const [busy, setBusy] = React.useState<string | null>(null);
    const [msg, setMsg] = React.useState<{ kind: "ok" | "error"; text: string } | null>(null);
    const [dialog, setDialog] = React.useState<{ text: string } | null>(null);

    // 排序在本地 state 里拖, 松手再落库, 拖动中不反复写 IndexedDB
    const [order, setOrder] = React.useState<KyaraImage[]>(images);
    React.useEffect(() => setOrder(images), [images]);

    const run = React.useCallback(async (label: string, job: () => Promise<string>): Promise<void> => {
        setBusy(label);
        setMsg(null);
        try {
            setMsg({ kind: "ok", text: await job() });
        } catch (e) {
            setMsg({ kind: "error", text: `${label}失败: ${(e as Error).message}` });
        } finally {
            setBusy(null);
        }
    }, []);

    const addFiles = React.useCallback((files: File[]): void => {
        if (files.length === 0) {
            setMsg({ kind: "error", text: "只收图片文件 (png / jpg / webp / gif / avif)." });
            return;
        }
        void run("添加", async () => `已加入 ${(await store.add(name, files)).length} 张到本地.`);
    }, [name, run, store]);

    // 粘贴: 图片直接入库; 文字里有链接就打开导入框并预填
    React.useEffect(() => {
        const onPaste = (e: ClipboardEvent): void => {
            const target = e.target as HTMLElement | null;
            if (target && (target.closest("input, textarea, [contenteditable='true']") || dialog)) return;
            const files = imagesOnly(e.clipboardData?.files);
            if (files.length > 0) {
                e.preventDefault();
                addFiles(files);
                return;
            }
            const text = e.clipboardData?.getData("text") ?? "";
            if (/https?:\/\//i.test(text)) {
                e.preventDefault();
                setDialog({ text });
            }
        };
        document.addEventListener("paste", onPaste);
        return () => document.removeEventListener("paste", onPaste);
    }, [addFiles, dialog]);

    const commitOrder = (): void => {
        const files = order.map((i: KyaraImage) => i.file);
        if (files.join("\n") === images.map((i: KyaraImage) => i.file).join("\n")) return;
        void run("排序", async () => {
            await store.reorder(name, files);
            return "顺序已保存.";
        });
    };

    const exportZip = (): void => {
        void run("导出", async () => {
            const blob = await store.exportZip(name);
            const a = document.createElement("a");
            a.href = URL.createObjectURL(blob);
            a.download = `kyara-${name}.zip`;
            a.click();
            setTimeout(() => URL.revokeObjectURL(a.href), 4000);
            return `已导出 ${images.length} 张, 解压到 static/img/kyara/${name}/ 即可.`;
        });
    };

    const writeRepo = (): void => {
        void run("写进仓库", async () => {
            let seq = nextSeq;
            for (const img of order) {
                const blob = await (await fetch(img.src)).blob();
                const file = formatKyaraFilename(seq, img.title, img.author, extOf(img, blob));
                await kyaraDev(name, `file/${encodeURIComponent(file)}`, {
                    method: "PUT",
                    headers: { "Content-Type": blob.type || "application/octet-stream" },
                    body: blob,
                });
                await store.remove(name, img.file);
                seq += 1;
            }
            return `已写进 static/img/kyara/${name}/, dev 热更新后出现在「仓库」里.`;
        });
    };

    const t = reduced ? { duration: 0 } : { duration: 0.32, ease: hxEase };

    return (
        <motion.section
            className={styles.localPanel}
            aria-label="本地模式"
            initial={reduced ? false : { opacity: 0, y: 12 }}
            animate={{ opacity: 1, y: 0 }}
            transition={t}
        >
            <HxCard accent="var(--kyara-accent)">
                <div className={styles.localGrid}>
                    <div
                        className={styles.dropzone}
                        data-over={over ? "true" : undefined}
                        onDragOver={(e) => {
                            e.preventDefault();
                            setOver(true);
                        }}
                        onDragLeave={() => setOver(false)}
                        onDrop={(e) => {
                            e.preventDefault();
                            setOver(false);
                            addFiles(imagesOnly(e.dataTransfer.files));
                        }}
                    >
                        <span className={styles.dropIcon} aria-hidden>＋</span>
                        <p><b>拖图片到这里</b>, 或直接 Ctrl/⌘ + V 粘贴</p>
                        <p className={styles.dim}>图片只存在这个浏览器里 (IndexedDB), 不会上传</p>
                        <div className={styles.dialogRow}>
                            <button type="button" className={styles.btn} onClick={() => fileRef.current?.click()} disabled={!!busy}>选择文件</button>
                            <button type="button" className={styles.btn} onClick={() => setDialog({ text: "" })} disabled={!!busy}>从链接导入</button>
                        </div>
                        <input
                            ref={fileRef}
                            type="file"
                            accept="image/*"
                            multiple
                            hidden
                            onChange={(e) => {
                                addFiles(imagesOnly(e.target.files));
                                e.target.value = "";
                            }}
                        />
                    </div>

                    <div className={styles.localList}>
                        <div className={styles.localHead}>
                            <span>本地 <b>{images.length}</b> 张 · 拖动调整顺序</span>
                            <div className={styles.dialogRow}>
                                <button type="button" className={styles.btnGhost} onClick={exportZip} disabled={!!busy || images.length === 0}>导出 zip</button>
                                {serverWritable && (
                                    <button type="button" className={styles.btnPrimary} onClick={writeRepo} disabled={!!busy || images.length === 0}>写进仓库</button>
                                )}
                            </div>
                        </div>
                        {order.length === 0 ? (
                            <p className={styles.dim}>还没有本地图片.</p>
                        ) : (
                            <Reorder.Group axis="y" values={order} onReorder={setOrder} className={styles.reorder}>
                                {order.map((img, i) => (
                                    <Reorder.Item
                                        key={img.file}
                                        value={img}
                                        className={styles.reorderItem}
                                        onDragEnd={commitOrder}
                                        dragListener={!busy}
                                        whileDrag={reduced ? undefined : { scale: 1.02, boxShadow: "var(--hx-shadow-md)" }}
                                    >
                                        <span className={styles.grip} aria-hidden>⋮⋮</span>
                                        <img src={img.src} alt="" loading="lazy" decoding="async" />
                                        <span className={styles.seq} data-static>{seqLabel(nextSeq + i)}</span>
                                        <span className={styles.reorderTitle}>{img.title}</span>
                                        {img.author && <span className={styles.capAuthor}>@{img.author}</span>}
                                        <SourceIcon image={img} />
                                        <button
                                            type="button"
                                            className={styles.iconBtn}
                                            aria-label={`删除 ${img.title}`}
                                            disabled={!!busy}
                                            onClick={() => void run("删除", async () => {
                                                await store.remove(name, img.file);
                                                return `已删除「${img.title}」.`;
                                            })}
                                        >
                                            ✕
                                        </button>
                                    </Reorder.Item>
                                ))}
                            </Reorder.Group>
                        )}
                    </div>
                </div>
                {(busy || msg) && (
                    <p className={styles.dialogNotice} role="status" data-kind={msg?.kind}>
                        {busy ? `${busy}中…` : msg?.text}
                    </p>
                )}
            </HxCard>

            <AnimatePresence>
                {dialog && (
                    <ImportDialog
                        key="import"
                        name={name}
                        local={local}
                        initialText={dialog.text}
                        onClose={() => setDialog(null)}
                        onRepoImported={(added) => setMsg({ kind: "ok", text: `已写进仓库 ${added.length} 张, dev 热更新后出现在「仓库」里.` })}
                    />
                )}
            </AnimatePresence>
        </motion.section>
    );
}
