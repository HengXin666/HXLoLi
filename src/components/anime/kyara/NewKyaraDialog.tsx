import React from "react";
import { useHistory } from "@docusaurus/router";
import { motion } from "framer-motion";
import { hxEase, useHxReducedMotion } from "@site/src/components/anime/ui";
import { isValidKyaraName, kyaraNameFrom } from "@site/src/utils/kyara/filename";
import { kyaraHref } from "@site/src/utils/kyara/link";
import { putLocalCharacter } from "@site/src/utils/kyara/localStore";
import type { KyaraIndexEntry } from "@site/src/utils/kyara/types";
import type { KyaraDevModule } from "./devChunks";
import { useDialogFocus } from "./Lightbox";
import styles from "./Kyara.module.css";

export interface NewKyaraSeed {
    displayName: string;
    work?: string;
    bangumiCharacterId?: number;
    bangumiSubjectIds?: number[];
}

interface Props {
    seed?: NewKyaraSeed;
    /** 当前可见的全部画集 (仓库 + 本地), 用来查重名 */
    entries: KyaraIndexEntry[];
    dev: KyaraDevModule | null;
    onClose: () => void;
}

/**
 * 新建画集: 显示名 → 目录名 (默认由显示名推出, 可改) → 作品. 默认建成本地角色 (IndexedDB, 线上可用);
 * 本机 dev 下可选直接建到仓库 (写 kyara.json). 目录名已被占用: 同一个 Bangumi 角色就直接打开那一页, 否则提示换名.
 * 右键菜单「以此新建」带着番剧角色的名字、角色 id、番剧 id 与番剧名进来.
 */
export default function NewKyaraDialog ({ seed, entries, dev, onClose }: Props): React.ReactElement {
    const reduced = useHxReducedMotion();
    const history = useHistory();
    const ref = React.useRef<HTMLFormElement>(null);
    const [displayName, setDisplayName] = React.useState(seed?.displayName ?? "");
    const [dirName, setDirName] = React.useState<string | null>(null);
    const [work, setWork] = React.useState(seed?.work ?? "");
    const [toRepo, setToRepo] = React.useState(false);
    const [busy, setBusy] = React.useState(false);
    const [error, setError] = React.useState<string | null>(null);

    useDialogFocus(ref, () => {
        if (!busy) onClose();
    });

    const name = (dirName ?? kyaraNameFrom(displayName)).trim();
    const taken = entries.find((e) => e.meta.name === name) ?? null;
    const sameCharacter = !!taken && !!seed?.bangumiCharacterId && taken.meta.bangumiCharacterId === seed.bangumiCharacterId;

    const submit = async (e: React.FormEvent): Promise<void> => {
        e.preventDefault();
        setError(null);
        if (!isValidKyaraName(name)) {
            setError("目录名不能为空, 不能含 / \\ 或以点开头, 也不能叫 local.");
            return;
        }
        if (taken) {
            if (sameCharacter) {
                onClose();
                history.push(kyaraHref(taken));
                return;
            }
            setError(`「${name}」已经被「${taken.meta.displayName}」用了, 换一个目录名.`);
            return;
        }
        setBusy(true);
        try {
            const meta = {
                displayName: displayName.trim() || name,
                ...(work.trim() ? { work: work.trim() } : {}),
                ...(seed?.bangumiCharacterId ? { bangumiCharacterId: seed.bangumiCharacterId } : {}),
                ...(seed?.bangumiSubjectIds?.length ? { bangumiSubjectIds: seed.bangumiSubjectIds } : {}),
            };
            if (toRepo && dev) {
                await dev.kyaraRepo.putMeta(name, meta);
            } else {
                await putLocalCharacter({ name, ...meta });
            }
            onClose();
            // 仓库新目录要等 dev 热重扫才有静态路由; 本地页会先用 dev 清单渲染, 路由出现后自动跳过去
            history.push(kyaraHref({ meta: { name }, local: true }));
        } catch (err) {
            setError((err as Error).message);
            setBusy(false);
        }
    };

    const t = reduced ? { duration: 0 } : { duration: 0.32, ease: hxEase };

    return (
        <motion.div className={styles.dialogLayer} initial={{ opacity: 0 }} animate={{ opacity: 1 }} exit={{ opacity: 0 }} transition={t}>
            <button type="button" className={styles.lbBackdrop} aria-label="关闭" tabIndex={-1} onClick={() => !busy && onClose()} />
            <motion.form
                ref={ref}
                className={styles.dialog}
                role="dialog"
                aria-modal="true"
                aria-labelledby="kyara-new-title"
                tabIndex={-1}
                onSubmit={submit}
                initial={reduced ? false : { opacity: 0, y: 16, scale: 0.98 }}
                animate={{ opacity: 1, y: 0, scale: 1 }}
                exit={reduced ? { opacity: 0 } : { opacity: 0, y: 12, scale: 0.98 }}
                transition={t}
            >
                <header className={styles.dialogHead}>
                    <h2 id="kyara-new-title">新建画集</h2>
                    <button type="button" className={styles.lbClose} onClick={onClose} disabled={busy} aria-label="关闭 (Esc)">✕</button>
                </header>

                <div className={styles.metaForm}>
                    <label className={styles.metaWide}>
                        <span>角色名</span>
                        <input data-autofocus value={displayName} onChange={(e) => setDisplayName(e.target.value)} placeholder="例: 春日野穹" autoComplete="off" />
                    </label>
                    <label>
                        <span>目录名 (也是页面地址)</span>
                        <input value={dirName ?? kyaraNameFrom(displayName)} onChange={(e) => setDirName(e.target.value)} autoComplete="off" spellCheck={false} />
                    </label>
                    <label>
                        <span>作品</span>
                        <input value={work} onChange={(e) => setWork(e.target.value)} placeholder="番剧名 (可空)" autoComplete="off" />
                    </label>
                </div>

                {dev && (
                    <fieldset className={styles.segment} aria-label="建在哪里">
                        {[false, true].map((repo) => (
                            <label key={String(repo)} data-on={toRepo === repo ? "true" : undefined}>
                                <input type="radio" name="kyara-new-target" checked={toRepo === repo} onChange={() => setToRepo(repo)} />
                                {repo ? "直接建到仓库" : "本地"}
                            </label>
                        ))}
                    </fieldset>
                )}

                {taken && sameCharacter && <p className={styles.dialogNotice} data-kind="ok">这个角色已经有画集了, 确认后直接打开.</p>}
                {error && <p className={styles.dialogNotice} role="alert" data-kind="error">{error}</p>}

                <footer className={styles.dialogFoot}>
                    <span className={styles.dim}>{toRepo && dev ? "写入 static/img/kyara/" + (name || "…") + "/kyara.json" : "保存在这个浏览器里, 之后可以导出 zip"}</span>
                    <button type="button" className={styles.btnGhost} onClick={onClose} disabled={busy}>取消</button>
                    <button type="submit" className={styles.btnPrimary} disabled={busy || !name}>{busy ? "创建中…" : sameCharacter ? "打开画集" : "创建"}</button>
                </footer>
            </motion.form>
        </motion.div>
    );
}
