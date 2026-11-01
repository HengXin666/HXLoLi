import { useCallback, useMemo } from "react";
import { useHistory, useLocation } from "@docusaurus/router";

// 关系图的全部视图状态都放在 URL 上, 组件只读 URL 派生状态. 进出聚焦层用 push (浏览器后退可回到概览), 其余调整用 replace

export interface GraphUrlState {
    /** 聚焦层中心番剧; undefined = 概览层 */
    focusId?: number;
    hiddenRelations: string[];
    preferCn: boolean;
    showImages: boolean;
    chargeStrength: number;
    linkDistance: number;
    /** 概览层共享声优边的最小共享人数 */
    minShared: number;
}

export const DEFAULT_HIDDEN_RELATIONS = ["配角", "客串", "闲角", "旁白"];

const DEFAULTS: Omit<GraphUrlState, "focusId"> = {
    hiddenRelations: DEFAULT_HIDDEN_RELATIONS,
    preferCn: true,
    showImages: true,
    chargeStrength: -180,
    linkDistance: 100,
    minShared: 2,
};

function readNumber (params: URLSearchParams, key: string, fallback: number): number {
    const raw = params.get(key);
    if (raw === null || raw === "") return fallback;
    const n = Number(raw);
    return Number.isFinite(n) ? n : fallback;
}

function readBool (params: URLSearchParams, key: string, fallback: boolean): boolean {
    return params.has(key) ? params.get(key) !== "false" : fallback;
}

export function parseGraphState (search: string): GraphUrlState {
    const params = new URLSearchParams(search);
    const idRaw = params.get("id");
    const id = idRaw ? Number(idRaw) : NaN;
    const hide = params.get("hide");
    return {
        focusId: Number.isFinite(id) ? id : undefined,
        // 显式写了 hide= (空) 表示一个都不隐藏
        hiddenRelations: hide === null ? DEFAULTS.hiddenRelations : hide.split(",").filter(Boolean),
        preferCn: readBool(params, "cn", DEFAULTS.preferCn),
        showImages: readBool(params, "img", DEFAULTS.showImages),
        chargeStrength: readNumber(params, "charge", DEFAULTS.chargeStrength),
        linkDistance: readNumber(params, "dist", DEFAULTS.linkDistance),
        minShared: readNumber(params, "cv", DEFAULTS.minShared),
    };
}

/** 只写与默认值不同的参数, 默认视图就是干净的 /anime/graph */
export function serializeGraphState (state: GraphUrlState): string {
    const params = new URLSearchParams();
    if (state.focusId !== undefined) params.set("id", String(state.focusId));
    const sameHidden = state.hiddenRelations.length === DEFAULTS.hiddenRelations.length
        && DEFAULTS.hiddenRelations.every(r => state.hiddenRelations.includes(r));
    if (!sameHidden) params.set("hide", state.hiddenRelations.join(","));
    if (state.preferCn !== DEFAULTS.preferCn) params.set("cn", String(state.preferCn));
    if (state.showImages !== DEFAULTS.showImages) params.set("img", String(state.showImages));
    if (state.chargeStrength !== DEFAULTS.chargeStrength) params.set("charge", String(state.chargeStrength));
    if (state.linkDistance !== DEFAULTS.linkDistance) params.set("dist", String(state.linkDistance));
    if (state.minShared !== DEFAULTS.minShared) params.set("cv", String(state.minShared));
    return params.toString();
}

export function useGraphUrlState (): [GraphUrlState, (patch: Partial<GraphUrlState>) => void] {
    const history = useHistory();
    const location = useLocation();
    const state = useMemo(() => parseGraphState(location.search), [location.search]);

    const update = useCallback((patch: Partial<GraphUrlState>) => {
        const next = { ...parseGraphState(location.search), ...patch };
        const search = serializeGraphState(next);
        if (`?${search}` === location.search || (search === "" && location.search === "")) return;
        const target = { pathname: location.pathname, search, hash: location.hash };
        if ("focusId" in patch) history.push(target);
        else history.replace(target);
    }, [history, location.pathname, location.search, location.hash]);

    return [state, update];
}
