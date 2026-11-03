import React, { useCallback, useContext, useEffect, useId, useMemo, useRef, useState } from "react";
import ReactDOM from "react-dom";
import { FloatingLayer } from "@site/src/components/common/Tooltip";
import { useDevService } from "@site/src/components/DevTools/useDevService";
import { isLocalDev } from "@site/src/utils/cdn/linkJsDelivr";
import {
    buildAllPrompts,
    buildCharacterPrompt,
    CharacterPromptInput,
    PROMPT_SEPARATOR,
} from "@site/src/utils/anime/characterPrompt";
import styles from "./CharacterContextMenu.module.css";

/** 番剧这一侧的信息, 由详情页的角色区提供一次 */
export interface CharacterMenuAnime {
    name: string;
    originalName?: string;
    coverUrl: string;
    coverFallbackUrl?: string;
}

/** 被右键的那个角色 */
export interface CharacterMenuTarget {
    id: number;
    name: string;
    originalName?: string;
    imageUrl: string;
    imageFallbackUrl?: string;
    relation?: string;
}

type MenuAction = "peek" | "standalone" | "download" | "copyUrl" | "openOriginal";

interface MenuItem {
    action: MenuAction;
    label: string;
    hint?: string;
    /** 开发者用途, 只在本地模式出现 */
    dev?: boolean;
}

/** 菜单项的唯一清单: 本地模式全显示, 网站模式过滤掉 dev 项 */
const MENU_ITEMS: readonly MenuItem[] = [
    { action: "peek", label: "复制探头立绘提示词", hint: "左右两份", dev: true },
    { action: "standalone", label: "复制角色独立立绘提示词", hint: "透明底" },
    { action: "download", label: "下载角色图" },
    { action: "copyUrl", label: "复制角色图片链接" },
    { action: "openOriginal", label: "打开 Bangumi 原图", hint: "新标签页" },
];

const LONG_PRESS_MS = 500;
const LONG_PRESS_SLOP = 10;
const TOAST_MS = 1800;

interface OpenState {
    target: CharacterMenuTarget;
    /** 指针位置 (视口坐标); 键盘触发时为 null, 菜单贴着触发元素下方 */
    point: { x: number; y: number } | null;
    trigger: HTMLElement;
}

interface MenuContextValue {
    open: (state: OpenState) => void;
    isOpenFor: (id: number) => boolean;
}

const MenuContext = React.createContext<MenuContextValue | null>(null);

async function copyText (text: string): Promise<boolean> {
    try {
        if (navigator.clipboard && window.isSecureContext) {
            await navigator.clipboard.writeText(text);
            return true;
        }
    } catch {
        // 权限被拒或非安全上下文, 退回 execCommand
    }
    const ta = document.createElement("textarea");
    ta.value = text;
    ta.setAttribute("readonly", "");
    ta.style.position = "fixed";
    ta.style.top = "0";
    ta.style.left = "0";
    ta.style.opacity = "0";
    document.body.appendChild(ta);
    ta.select();
    let ok = false;
    try {
        ok = document.execCommand("copy");
    } catch {
        ok = false;
    }
    ta.remove();
    return ok;
}

const safeFileName = (s: string): string => s.replace(/[\\/:*?"<>|\s]+/g, "_").slice(0, 60) || "character";

/** CDN 允许跨域, 先取 blob 再存, 文件名才可控; 取不到就新开标签页让用户自己存 */
async function downloadImage (url: string, name: string): Promise<"saved" | "opened"> {
    try {
        const res = await fetch(url, { mode: "cors" });
        if (!res.ok) throw new Error(String(res.status));
        const blob = await res.blob();
        const ext = blob.type.includes("png") ? "png" : blob.type.includes("webp") ? "webp" : "jpg";
        const href = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = href;
        a.download = `${safeFileName(name)}.${ext}`;
        document.body.appendChild(a);
        a.click();
        a.remove();
        window.setTimeout(() => URL.revokeObjectURL(href), 10_000);
        return "saved";
    } catch {
        window.open(url, "_blank", "noopener,noreferrer");
        return "opened";
    }
}

/** 本地模式 = isLocalDev() || dev-edit-server health 探测通过. 两套都是仓里已有的判定 */
function useLocalMode (): boolean {
    const { ready } = useDevService();
    const [localHost, setLocalHost] = useState(false);
    useEffect(() => {
        setLocalHost(isLocalDev());
    }, []);
    return localHost || ready;
}

/**
 * 角色立绘右键菜单的作用域: 一个详情页只有一个菜单实例与一次本地模式探测, 每张角色卡经 useCharacterMenuTrigger 接入.
 * 菜单定位复用 common/Tooltip 的 FloatingLayer (fixed、翻转、视口内夹取), 锚点是指针位置上一个 0x0 的 fixed 点.
 * 菜单按 WAI-ARIA menu 模式: 打开后焦点进第一项, 上下方向键 / Home / End 移动, Enter / 空格执行, Esc 与 Tab 关闭并把焦点还给角色图.
 * 点外部、滚动、窗口尺寸变化都会关闭. 结果用页内 toast (role=status) 告知, 不用原生 alert.
 * .agents/notes/implemented/feature/2026-11-03-anime-character-context-menu.md
 */
export function CharacterMenuScope ({ anime, children }: { anime: CharacterMenuAnime; children: React.ReactNode }): React.ReactElement {
    const local = useLocalMode();
    const [state, setState] = useState<OpenState | null>(null);
    const [toast, setToast] = useState<{ text: string; key: number } | null>(null);
    const [anchor, setAnchor] = useState<HTMLElement | null>(null);
    const cardRef = useRef<HTMLDivElement | null>(null);
    const itemRefs = useRef<(HTMLButtonElement | null)[]>([]);
    const toastTimer = useRef<number | undefined>(undefined);
    const uid = useId().replace(/:/g, "");
    const menuId = `hx-charmenu-${uid}`;

    // 网站模式去掉开发者项; 没有 Bangumi 原图地址的角色不显示"打开原图"
    const items = useMemo(
        () => MENU_ITEMS.filter((it) => (local || !it.dev) && (it.action !== "openOriginal" || !!state?.target.imageFallbackUrl)),
        [local, state],
    );

    const stateRef = useRef<OpenState | null>(null);
    stateRef.current = state;

    const close = useCallback((restoreFocus: boolean) => {
        const cur = stateRef.current;
        setState(null);
        if (cur && restoreFocus && cur.trigger.isConnected) cur.trigger.focus({ preventScroll: true });
    }, []);

    const showToast = useCallback((text: string) => {
        window.clearTimeout(toastTimer.current);
        setToast({ text, key: Date.now() });
        toastTimer.current = window.setTimeout(() => setToast(null), TOAST_MS);
    }, []);

    useEffect(() => () => window.clearTimeout(toastTimer.current), []);

    const ctx = useMemo<MenuContextValue>(() => ({
        open: (next) => setState(next),
        isOpenFor: (id) => state?.target.id === id,
    }), [state]);

    // 打开后焦点进第一项; 点外部 / 滚动 / resize 关闭
    useEffect(() => {
        if (!state) return undefined;
        const raf = window.requestAnimationFrame(() => itemRefs.current[0]?.focus({ preventScroll: true }));
        const inside = (t: EventTarget | null) => !!(t instanceof Node && cardRef.current?.contains(t));
        const onDown = (e: PointerEvent) => {
            if (!inside(e.target)) close(false);
        };
        const onScroll = (e: Event) => {
            if (!inside(e.target)) close(false);
        };
        const onResize = () => close(false);
        document.addEventListener("pointerdown", onDown, true);
        window.addEventListener("scroll", onScroll, true);
        window.addEventListener("resize", onResize);
        return () => {
            window.cancelAnimationFrame(raf);
            document.removeEventListener("pointerdown", onDown, true);
            window.removeEventListener("scroll", onScroll, true);
            window.removeEventListener("resize", onResize);
        };
    }, [state, close]);

    const run = useCallback(async (action: MenuAction) => {
        if (!state) return;
        const t = state.target;
        const input: CharacterPromptInput = {
            animeName: anime.name,
            animeOriginalName: anime.originalName,
            coverUrl: anime.coverUrl,
            coverFallbackUrl: anime.coverFallbackUrl,
            characterName: t.name,
            characterOriginalName: t.originalName,
            characterImageUrl: t.imageUrl,
            characterImageFallbackUrl: t.imageFallbackUrl,
            relation: t.relation,
        };
        close(true);
        if (action === "openOriginal") {
            window.open(t.imageFallbackUrl || t.imageUrl, "_blank", "noopener,noreferrer");
            return;
        }
        if (action === "download") {
            const how = await downloadImage(t.imageUrl, t.name);
            showToast(how === "saved" ? `已下载「${t.name}」的角色图` : "已在新标签页打开角色图, 可直接另存");
            return;
        }
        let text = t.imageUrl;
        if (action === "peek") {
            const all = buildAllPrompts(input);
            text = `${all.left}${PROMPT_SEPARATOR}${all.right}`;
        } else if (action === "standalone") {
            text = buildCharacterPrompt(input);
        }
        const ok = await copyText(text);
        const what = action === "peek" ? "探头立绘提示词 (左右两份)" : action === "standalone" ? "角色独立立绘提示词" : "角色图片链接";
        showToast(ok ? `已复制${what}` : "复制失败: 浏览器拒绝了剪贴板权限");
        if (state.trigger.isConnected) state.trigger.focus({ preventScroll: true });
    }, [state, anime, close, showToast]);

    const onMenuKeyDown = (e: React.KeyboardEvent<HTMLDivElement>) => {
        const list = itemRefs.current.slice(0, items.length).filter((x): x is HTMLButtonElement => !!x);
        const i = list.indexOf(document.activeElement as HTMLButtonElement);
        const move = (to: number) => {
            e.preventDefault();
            list[(to + list.length) % list.length]?.focus({ preventScroll: true });
        };
        switch (e.key) {
            case "ArrowDown": move(i + 1); break;
            case "ArrowUp": move(i < 0 ? list.length - 1 : i - 1); break;
            case "Home": move(0); break;
            case "End": move(list.length - 1); break;
            case "Escape":
                e.preventDefault();
                e.stopPropagation();
                close(true);
                break;
            case "Tab":
                e.preventDefault();
                close(true);
                break;
            default:
                break;
        }
    };

    const content = state && (
        <div
            role="menu"
            id={menuId}
            aria-label={`角色「${state.target.name}」`}
            className={styles.menu}
            onKeyDown={onMenuKeyDown}
            onContextMenu={(e) => e.preventDefault()}
        >
            <div className={styles.head} aria-hidden>
                <span className={styles.headName}>{state.target.name}</span>
                {local && <span className={styles.devBadge}>本地</span>}
            </div>
            {items.map((it, i) => (
                <React.Fragment key={it.action}>
                    {i > 0 && items[i - 1].dev && !it.dev && <div role="separator" className={styles.sep} />}
                    <button
                        type="button"
                        role="menuitem"
                        tabIndex={-1}
                        ref={(el) => { itemRefs.current[i] = el; }}
                        className={styles.item}
                        data-dev={it.dev ? "true" : undefined}
                        onClick={() => void run(it.action)}
                    >
                        <span className={styles.label}>{it.label}</span>
                        {it.hint && <span className={styles.hint}>{it.hint}</span>}
                    </button>
                </React.Fragment>
            ))}
        </div>
    );

    const pointAnchor = state?.point && typeof document !== "undefined"
        ? ReactDOM.createPortal(
            <span
                ref={setAnchor}
                aria-hidden
                className={styles.pointAnchor}
                style={{ left: state.point.x, top: state.point.y - 10 }}
            />,
            document.body,
        )
        : null;
    const layerAnchor = state ? (state.point ? anchor : state.trigger) : null;

    return (
        <MenuContext.Provider value={ctx}>
            {children}
            {pointAnchor}
            {state && layerAnchor && (
                <FloatingLayer
                    id={`${menuId}-layer`}
                    anchor={layerAnchor}
                    content={content}
                    className={styles.layer}
                    style={{ padding: 4 }}
                    width={256}
                    interactive
                    cardRef={cardRef}
                    role="presentation"
                    prefer="bottom"
                    align="start"
                    arrow={false}
                />
            )}
            {toast && typeof document !== "undefined" && ReactDOM.createPortal(
                <div key={toast.key} role="status" aria-live="polite" className={styles.toast}>{toast.text}</div>,
                document.body,
            )}
        </MenuContext.Provider>
    );
}

/** 给角色图挂上的事件与类名: 右键 / 键盘菜单键 / Shift+F10 / 触屏长按都打开同一个菜单 */
export function useCharacterMenuTrigger (target: CharacterMenuTarget): {
    className: string;
    "aria-haspopup": "menu";
    onContextMenu: React.MouseEventHandler<HTMLElement>;
    onPointerDown: React.PointerEventHandler<HTMLElement>;
    onPointerMove: React.PointerEventHandler<HTMLElement>;
    onPointerUp: React.PointerEventHandler<HTMLElement>;
    onPointerCancel: React.PointerEventHandler<HTMLElement>;
    onClickCapture: React.MouseEventHandler<HTMLElement>;
} {
    const ctx = useContext(MenuContext);
    const timer = useRef<number | undefined>(undefined);
    const start = useRef<{ x: number; y: number } | null>(null);
    const fired = useRef(false);
    const targetRef = useRef(target);
    targetRef.current = target;

    const clear = useCallback(() => {
        window.clearTimeout(timer.current);
        timer.current = undefined;
        start.current = null;
    }, []);

    useEffect(() => clear, [clear]);

    return {
        className: styles.trigger,
        "aria-haspopup": "menu",
        onContextMenu: (e) => {
            e.preventDefault();
            if (!ctx) return;
            if (fired.current && ctx.isOpenFor(targetRef.current.id)) return;
            // 键盘菜单键 / Shift+F10 触发时坐标是 0, 改为贴着元素
            const fromKeyboard = e.clientX === 0 && e.clientY === 0;
            ctx.open({
                target: targetRef.current,
                point: fromKeyboard ? null : { x: e.clientX, y: e.clientY },
                trigger: e.currentTarget,
            });
        },
        onPointerDown: (e) => {
            fired.current = false;
            if (e.pointerType === "mouse" || !ctx) return;
            const el = e.currentTarget;
            const x = e.clientX;
            const y = e.clientY;
            clear();
            start.current = { x, y };
            timer.current = window.setTimeout(() => {
                fired.current = true;
                start.current = null;
                ctx.open({ target: targetRef.current, point: { x, y }, trigger: el });
            }, LONG_PRESS_MS);
        },
        onPointerMove: (e) => {
            const s = start.current;
            if (s && Math.hypot(e.clientX - s.x, e.clientY - s.y) > LONG_PRESS_SLOP) clear();
        },
        onPointerUp: () => clear(),
        onPointerCancel: () => clear(),
        onClickCapture: (e) => {
            // 长按松手后的那次 click 不能把用户带去 bgm.tv
            if (fired.current) {
                e.preventDefault();
                e.stopPropagation();
                fired.current = false;
            }
        },
    };
}
