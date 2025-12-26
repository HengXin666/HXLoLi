import React, { useEffect, useState } from "react";
import { useHistory, useLocation } from "@docusaurus/router";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { AnimeForceGraph, AnimeGraphController, THEME } from "@site/src/components/anime/AnimeGraphPage";
import { useAnimeRecords } from "@site/src/utils/anime/animeStore";


export default function AnimeGraphPage () {
    const { siteConfig } = useDocusaurusContext();
    const records = useAnimeRecords(siteConfig.baseUrl);

    const history = useHistory();
    const location = useLocation();

    // 状态初始化: 优先从 URL 读取, 如果没有则使用默认值
    const [targetAnimeId, setTargetAnimeId] = useState<number | undefined>(() => {
        const params = new URLSearchParams(location.search);
        const id = params.get('id');
        return id ? Number(id) : undefined;
    });

    const [hiddenRelations, setHiddenRelations] = useState<string[]>(() => {
        const params = new URLSearchParams(location.search);
        if (params.has('hide')) {
            const val = params.get('hide');
            // 处理空字符串的情况, 避免 split 出来 ['']
            return val ? val.split(',') : [];
        }
        // 默认值
        return ["配角", "客串", "闲角", "旁白"];
    });

    const [preferCn, setPreferCn] = useState(() => {
        const params = new URLSearchParams(location.search);
        // 如果 URL 里没有 'cn' 参数, 默认 true; 如果有, 判断是否不等于 'false'
        return params.has('cn') ? params.get('cn') !== 'false' : true;
    });

    const [showImages, setShowImages] = useState(() => {
        const params = new URLSearchParams(location.search);
        return params.has('img') ? params.get('img') !== 'false' : true;
    });

    // 从 URL 初始化 chargeStrength (默认为 -180)
    const [chargeStrength, setChargeStrength] = useState<number>(() => {
        const params = new URLSearchParams(location.search);
        const val = params.get('charge');
        return val ? Number(val) : -180; 
    });

    // 从 URL 初始化 linkDistance (默认为 100)
    const [linkDistance, setLinkDistance] = useState<number>(() => {
        const params = new URLSearchParams(location.search);
        const val = params.get('dist');
        return val ? Number(val) : 100;
    });

    // 监听状态变化并写入 URL
    useEffect(() => {
        const params = new URLSearchParams();
        
        // 原有参数
        if (targetAnimeId !== undefined) params.set('id', String(targetAnimeId));
        if (hiddenRelations.length > 0) params.set('hide', hiddenRelations.join(','));
        params.set('cn', String(preferCn));
        params.set('img', String(showImages));

        params.set('charge', String(chargeStrength));
        params.set('dist', String(linkDistance));

        const newSearch = params.toString();
        
        if (location.search !== `?${newSearch}`) {
            history.replace({
                pathname: location.pathname,
                search: newSearch,
                hash: location.hash
            });
        }
    }, [
        targetAnimeId, hiddenRelations, preferCn, showImages, 
        chargeStrength, linkDistance,
        history, location.pathname, location.hash
    ]);

    return (
        <div style={{
            position: 'relative',
            width: '100%',
            height: '100vh',
            background: THEME.background,
            overflow: 'hidden'
        }}>
            <AnimeForceGraph
                baseUrl={siteConfig.baseUrl}
                targetAnimeId={targetAnimeId}
                hiddenRelations={hiddenRelations}
                preferCn={preferCn}
                showImages={showImages}
                chargeStrength={chargeStrength}
                linkDistance={linkDistance}
            />
            <AnimeGraphController
                hiddenRelations={hiddenRelations}
                setHiddenRelations={setHiddenRelations}
                preferCn={preferCn}
                setPreferCn={setPreferCn}
                showImages={showImages}
                setShowImages={setShowImages}
                targetAnimeId={targetAnimeId}
                setTargetAnimeId={setTargetAnimeId}
                allRecords={records}
                chargeStrength={chargeStrength}
                setChargeStrength={setChargeStrength}
                linkDistance={linkDistance}
                setLinkDistance={setLinkDistance}
            />
        </div>
    );
}