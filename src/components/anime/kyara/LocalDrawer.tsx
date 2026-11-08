import React from "react";
import { AnimatePresence, Reorder, motion } from "framer-motion";
import { hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import { putLocalCharacter, removeLocalCharacter, type KyaraLocalApi } from "@site/src/utils/kyara/localStore";
import type { KyaraImage, KyaraMeta } from "@site/src/utils/kyara/types";
import ImportDialog from "./ImportDialog";
import type { KyaraDevModule } from "./devChunks";
import { SourceIcon, seqLabel, useDialogFocus } from "./Lightbox";
import styles from "./Kyara.module.css";

const IMAGE_EXT = /\.(png|jpe?g|webp|gif|avif)$/i;

const imagesOnly = (files: FileList | File[] | null | undefined): File[] =>
    Array.from(files ?? []).filter((f) => f.type.startsWith("image/") || IMAGE_EXT.test(f.name));

type Tab = "local" | "repo" | "meta";

interface Props {
    name: string;
    meta: KyaraMeta;
    local: KyaraLocalApi;
    dev: KyaraDevModule | null;
    /** 纯本地角色 (仓库里还没有目录) */
    localOnly: boolean;
    /** 本机 dev 下的仓库管理面板 (DevRepoTools), 其余情况为 null */
    repoTools: React.ReactNode;
    /** 写进仓库后通知页面刷新仓库数据 */
    onRepoChanged: () => void;
    /** 删除纯本地角色后 */
    onDeleted?: () => void;
}

/**
 * 本地模式的操作台: 吸底工具栏 (张数 / 添加 / 从链接导入 / 导出 zip / 写进仓库) + 侧抽屉 (本地图排序删除、本地角色资料、dev 仓库管理).
 * 全局粘贴: 图片直接进本地库, 链接打开导入框; 工具栏与抽屉都接受拖放.
 * 本地数据只在这个浏览器里 (IndexedDB), 线上与 dev 都能用; 「写进仓库」与「仓库」页签只在本机 dev 下出现.
 */
export default function LocalDrawer ({ name, meta, local, dev, localOnly, repoTools, onRepoChanged, onDeleted }: Props): React.ReactElement {
    const reduced = useHxReducedMotion();
    const { store, images } = local;
    const fileRef = React.useRef<HTMLInputElement>(null);
    const [open, setOpen] = React.useState(false);
    const [tab, setTab] = React.useState<Tab>("local");
    const [over, setOver] = React.useState(false);
    const [busy, setBusy] = React.useState<string | null>(null);
    const [msg, setMsg] = React.useState<{ kind: "ok" | "error"; text: string } | null>(null);
    const [dialog, setDialog] = React.useState<{ text: string } | null>(null);
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

    React.useEffect(() => {
        if (!msg || msg.kind === "error") return undefined;
        const t = window.setTimeout(() => setMsg(null), 3200);
        return () => window.clearTimeout(t);
    }, [msg]);

    const addFiles = React.useCallback((files: File[]): void => {
        if (files.length === 0) {
            setMsg({ kind: "error", text: "只收图片文件 (png / jpg / webp / gif / avif)." });
            return;
        }
        void run("添加", async () => {
            await store.add(name, files);
            return `已加入 ${files.length} 张.`;
        });
    }, [name, run, store]);

    // 全局粘贴: 图片直接入库; 文字里有链接就打开导入框并预填
    React.useEffect(() => {
        const onPaste = (e: ClipboardEvent): void => {
            const target = e.target as HTMLElement | null;
            if (dialog || (target && target.closest("input, textarea, [contenteditable='true']"))) return;
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

    const dropProps = {
        onDragOver: (e: React.DragEvent) => {
            if (!Array.from(e.dataTransfer.types).includes("Files")) return;
            e.preventDefault();
            setOver(true);
        },
        onDragLeave: () => setOver(false),
        onDrop: (e: React.DragEvent) => {
            if (!Array.from(e.dataTransfer.types).includes("Files")) return;
            e.preventDefault();
            setOver(false);
            addFiles(imagesOnly(e.dataTransfer.files));
        },
    };

    const commitOrder = (): void => {
        const files = order.map((i) => i.file);
        if (files.join("\n") === images.map((i) => i.file).join("\n")) return;
        void run("排序", async () => {
            await store.reorder(name, files);
            return "顺序已保存.";
        });
    };

    const exportZip = (): void => {
        void run("导出", async () => {
            const blob = await store.exportZip(name, localOnly ? { ...meta, cover: local.character?.cover } : undefined);
            const a = document.createElement("a");
            a.href = URL.createObjectURL(blob);
            a.download = `kyara-${name}.zip`;
            a.click();
            window.setTimeout(() => URL.revokeObjectURL(a.href), 4000);
            return `已导出 ${images.length} 张, 文件已按序号命名.`;
        });
    };

    const writeRepo = (): void => {
        if (!dev) return;
        void run("写进仓库", async () => {
            const done = await dev.commitLocalToRepo(name, store, order, localOnly ? { ...meta, cover: local.character?.cover } : undefined);
            if (localOnly) await removeLocalCharacter(name);
            onRepoChanged();
            return `已写进 static/img/kyara/${name}/ 共 ${done.length} 张.`;
        });
    };

    const t = reduced ? { duration: 0 } : { duration: 0.32, ease: hxEase };
    const tabs: { key: Tab; label: string }[] = [
        { key: "local", label: `本地 ${images.length}` },
        ...(localOnly ? [{ key: "meta" as Tab, label: "资料" }] : []),
        ...(repoTools ? [{ key: "repo" as Tab, label: "仓库" }] : []),
    ];

    return (
        <>
            <div className={styles.dock} data-over={over ? "true" : undefined} {...dropProps}>
                <div className={styles.dockInner} role="toolbar" aria-label="本地模式">
                    <span className={styles.dockLabel}>
                        <span className={styles.localTag}>本地</span>
                        <b>{images.length}</b> 张 · 拖放或粘贴图片即可加入
                    </span>
                    <div className={styles.dockActions}>
                        <button type="button" className={styles.btn} onClick={() => fileRef.current?.click()} disabled={!!busy}>添加图片</button>
                        <button type="button" className={styles.btn} onClick={() => setDialog({ text: "" })} disabled={!!busy}>从链接导入</button>
                        <button type="button" className={styles.btnGhost} onClick={exportZip} disabled={!!busy || images.length === 0}>导出 zip</button>
                        {dev && (
                            <button type="button" className={styles.btnGhost} onClick={writeRepo} disabled={!!busy || images.length === 0}>写进仓库</button>
                        )}
                        <button type="button" className={styles.btnPrimary} onClick={() => setOpen(true)} aria-haspopup="dialog">管理</button>
                    </div>
                </div>
                <AnimatePresence>
                    {(busy || msg) && (
                        <motion.p
                            key="notice"
                            className={styles.dockNotice}
                            role="status"
                            data-kind={msg?.kind}
                            initial={{ opacity: 0, y: 6 }}
                            animate={{ opacity: 1, y: 0 }}
                            exit={{ opacity: 0 }}
                            transition={t}
                        >
                            {busy ? `${busy}中…` : msg?.text}
                        </motion.p>
                    )}
                </AnimatePresence>
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

            <AnimatePresence>
                {open && (
                    <motion.div key="drawer" className={styles.drawerLayer} initial={{ opacity: 0 }} animate={{ opacity: 1 }} exit={{ opacity: 0 }} transition={t}>
                        <button type="button" className={styles.lbBackdrop} aria-label="关闭" tabIndex={-1} onClick={() => setOpen(false)} />
                        <DrawerShell onClose={() => setOpen(false)} reduced={reduced}>
                            <header className={styles.dialogHead}>
                                <h2>管理画集</h2>
                                <button type="button" className={styles.lbClose} onClick={() => setOpen(false)} aria-label="关闭 (Esc)" data-autofocus>✕</button>
                            </header>
                            {tabs.length > 1 && (
                                <div className={styles.segment} role="tablist" aria-label="分类">
                                    {tabs.map((x) => (
                                        <button
                                            key={x.key}
                                            type="button"
                                            role="tab"
                                            aria-selected={tab === x.key}
                                            data-on={tab === x.key ? "true" : undefined}
                                            onClick={() => setTab(x.key)}
                                        >
                                            {x.label}
                                        </button>
                                    ))}
                                </div>
                            )}

                            {tab === "local" && (
                                <div className={styles.drawerBody}>
                                    <div className={styles.dropzone} data-over={over ? "true" : undefined} {...dropProps}>
                                        <span className={styles.dropIcon} aria-hidden>＋</span>
                                        <p><b>拖图片到这里</b>, 或直接 Ctrl/⌘ + V 粘贴</p>
                                        <p className={styles.dim}>只保存在这个浏览器里, 不会上传</p>
                                    </div>
                                    {order.length === 0 ? (
                                        <p className={styles.dim}>本地还没有图片.</p>
                                    ) : (
                                        <Reorder.Group axis="y" values={order} onReorder={setOrder} className={styles.reorder}>
                                            {order.map((img) => (
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
                                                    <span className={styles.seq} data-static>{seqLabel(img.seq)}</span>
                                                    <span className={styles.reorderTitle}>{img.title}</span>
                                                    {img.author && <span className={styles.capAuthor}>@{img.author}</span>}
                                                    <SourceIcon image={img} />
                                                    {localOnly && (
                                                        <button
                                                            type="button"
                                                            className={styles.btnGhost}
                                                            data-small
                                                            disabled={!!busy || local.character?.cover === img.file}
                                                            onClick={() => void run("设封面", async () => {
                                                                if (local.character) await putLocalCharacter({ ...local.character, cover: img.file });
                                                                return `封面已设为「${img.title}」.`;
                                                            })}
                                                        >
                                                            {local.character?.cover === img.file ? "封面" : "设为封面"}
                                                        </button>
                                                    )}
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
                            )}

                            {tab === "meta" && local.character && (
                                <LocalMetaForm
                                    key={local.character.name}
                                    character={local.character}
                                    busy={!!busy}
                                    onSave={(next) => void run("保存", async () => {
                                        await putLocalCharacter(next);
                                        return "资料已保存.";
                                    })}
                                    onDelete={() => void run("删除角色", async () => {
                                        await removeLocalCharacter(name);
                                        setOpen(false);
                                        onDeleted?.();
                                        return "已删除这个本地角色.";
                                    })}
                                />
                            )}

                            {tab === "repo" && repoTools}

                            {(busy || msg) && (
                                <p className={styles.dialogNotice} role="status" data-kind={msg?.kind}>{busy ? `${busy}中…` : msg?.text}</p>
                            )}
                        </DrawerShell>
                    </motion.div>
                )}
            </AnimatePresence>

            <AnimatePresence>
                {dialog && (
                    <ImportDialog
                        key="import"
                        name={name}
                        store={store}
                        dev={dev}
                        initialText={dialog.text}
                        onClose={() => setDialog(null)}
                        onRepoImported={(added) => {
                            onRepoChanged();
                            setMsg({ kind: "ok", text: `已写进仓库 ${added.length} 张.` });
                        }}
                    />
                )}
            </AnimatePresence>
        </>
    );
}

/** 抽屉本体: 挂载即锁滚动、困住焦点, 卸载还焦点 (useDialogFocus 只在挂载时生效, 所以单独成组件) */
function DrawerShell ({ onClose, reduced, children }: { onClose: () => void; reduced: boolean; children: React.ReactNode }): React.ReactElement {
    const ref = React.useRef<HTMLDivElement>(null);
    useDialogFocus(ref, onClose);
    return (
        <motion.div
            ref={ref}
            className={styles.drawer}
            role="dialog"
            aria-modal="true"
            aria-label="管理画集"
            tabIndex={-1}
            initial={reduced ? false : { x: 40, opacity: 0 }}
            animate={{ x: 0, opacity: 1 }}
            exit={reduced ? { opacity: 0 } : { x: 40, opacity: 0 }}
            transition={reduced ? { duration: 0 } : { duration: 0.32, ease: hxEase }}
        >
            {children}
        </motion.div>
    );
}

function LocalMetaForm ({ character, busy, onSave, onDelete }: {
    character: NonNullable<KyaraLocalApi["character"]>;
    busy: boolean;
    onSave: (c: NonNullable<KyaraLocalApi["character"]>) => void;
    onDelete: () => void;
}): React.ReactElement {
    const [form, setForm] = React.useState({ displayName: character.displayName, work: character.work ?? "", accent: character.accent ?? "" });
    const [confirm, setConfirm] = React.useState(false);
    return (
        <form
            className={styles.metaForm}
            onSubmit={(e) => {
                e.preventDefault();
                onSave({
                    ...character,
                    displayName: form.displayName.trim() || character.name,
                    work: form.work.trim() || undefined,
                    accent: form.accent.trim() || undefined,
                });
            }}
        >
            <label>
                <span>显示名</span>
                <input value={form.displayName} onChange={(e) => setForm({ ...form, displayName: e.target.value })} />
            </label>
            <label>
                <span>作品</span>
                <input value={form.work} onChange={(e) => setForm({ ...form, work: e.target.value })} />
            </label>
            <label>
                <span>主色</span>
                <input value={form.accent} onChange={(e) => setForm({ ...form, accent: e.target.value })} placeholder="#e86a92" />
            </label>
            <label>
                <span>目录名</span>
                <input value={character.name} readOnly />
            </label>
            <div className={styles.metaWide}>
                <button type="submit" className={styles.btnPrimary} disabled={busy}>保存</button>
                <button
                    type="button"
                    className={styles.btnGhost}
                    data-danger="true"
                    disabled={busy}
                    onBlur={() => setConfirm(false)}
                    onClick={() => (confirm ? onDelete() : setConfirm(true))}
                >
                    {confirm ? "再点一次删除角色和全部本地图" : "删除这个本地角色"}
                </button>
            </div>
        </form>
    );
}
