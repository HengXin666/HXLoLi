import React from "react";
import Layout from "@theme/Layout";
import Link from "@docusaurus/Link";
import BrowserOnly from "@docusaurus/BrowserOnly";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { Redirect, useHistory, useLocation } from "@docusaurus/router";
import { HxSkeleton, HxState } from "@site/src/components/anime/ui";
import { StagePaper } from "@site/src/components/anime/stage/Stage";
import { getLocalCharacter } from "@site/src/utils/kyara/localStore";
import { kyaraHref, useKyaraStatic } from "@site/src/utils/kyara/link";
import type { KyaraData, KyaraLocalCharacter } from "@site/src/utils/kyara/types";
import { KyaraView } from "./KyaraPage";
import { useKyaraDev } from "./devChunks";
import styles from "./Kyara.module.css";

type Load =
    | { status: "loading" }
    | { status: "missing" }
    | { status: "local"; character: KyaraLocalCharacter }
    | { status: "repo"; data: KyaraData };

function LocalBody (): React.ReactElement {
    const location = useLocation();
    const history = useHistory();
    const { siteConfig } = useDocusaurusContext();
    const dev = useKyaraDev();
    const statics = useKyaraStatic();
    const name = new URLSearchParams(location.search).get("name") ?? "";
    const [state, setState] = React.useState<Load>({ status: "loading" });

    const inRepo = statics.find((c) => c.meta.name === name);

    React.useEffect(() => {
        if (!name || inRepo) return undefined;
        let alive = true;
        let timer: number | undefined;
        const load = async (): Promise<void> => {
            const c = await getLocalCharacter(name).catch(() => null);
            if (!alive) return;
            if (c) {
                setState({ status: "local", character: c });
                return;
            }
            // 本机 dev 下刚建到仓库的角色: 先用 dev 清单渲染, 等热重扫注册路由后由上面的 inRepo 跳过去
            const d = dev ? await dev.kyaraRepo.read(siteConfig.baseUrl, name).catch(() => null) : null;
            if (!alive) return;
            if (d) {
                setState({ status: "repo", data: d });
                timer = window.setTimeout(() => void load(), 1500);
                return;
            }
            setState({ status: "missing" });
        };
        void load();
        return () => {
            alive = false;
            window.clearTimeout(timer);
        };
    }, [name, inRepo, dev, siteConfig.baseUrl]);

    if (inRepo) return <Redirect to={kyaraHref(inRepo)} />;

    if (state.status === "loading") {
        return (
            <StagePaper className={styles.page}>
                <div className={styles.product} aria-busy="true" aria-label="正在加载">
                    <HxSkeleton shape="card" count={2} />
                </div>
            </StagePaper>
        );
    }
    if (state.status === "missing") {
        return (
            <StagePaper className={styles.page}>
                <div className={styles.section}>
                    <HxState kind="empty" title="这个浏览器里没有这位角色" body="本地画集只保存在创建它的浏览器里. 回到角色列表看看吧." />
                    <p className={styles.emptyActions}><Link className={styles.btnPrimary} to="/anime/kyara">回到キャラ</Link></p>
                </div>
            </StagePaper>
        );
    }
    if (state.status === "repo") return <KyaraView kyara={state.data} />;
    const c = state.character;
    return (
        <KyaraView
            kyara={{ meta: { name: c.name, displayName: c.displayName }, images: [] }}
            localOnly
            onLocalDeleted={() => history.push("/anime/kyara")}
        />
    );
}

/**
 * /anime/kyara/local?name={name}: 纯本地角色 (只在这个浏览器的 IndexedDB 里) 的画集页.
 * 静态构建时没有这些角色, 所以主体只在浏览器里渲染 (BrowserOnly), 避免水合不一致; 同名仓库角色存在时直接跳到仓库路由.
 */
export default function KyaraLocalPage (): React.ReactElement {
    return (
        <Layout title="本地角色 · キャラ" description="保存在这个浏览器里的角色画集">
            <BrowserOnly fallback={<StagePaper className={styles.page}><div className={styles.product} /></StagePaper>}>
                {() => <LocalBody />}
            </BrowserOnly>
        </Layout>
    );
}
