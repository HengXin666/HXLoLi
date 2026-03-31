import { parse } from "@site/src/utils/anime/parser";
import { Actor, ANiMeRecord } from "@site/src/utils/anime/types";
import { useState, useEffect } from "react";
import { reqAnimeByCDN } from '@site/src/utils/cdn/linkJsDelivr'

let cachedRecords: readonly ANiMeRecord[] | null = null;
let loadingPromise: Promise<readonly ANiMeRecord[]> | null = null;

let actorMap: Map<number, Actor> | null = null;
let loadingActorPromise: Promise<Map<number, Actor>> | null = null;

function loadAnimeRecords (): Promise<readonly ANiMeRecord[]> {
    if (cachedRecords) {
        return Promise.resolve(cachedRecords);
    }

    if (loadingPromise) {
        return loadingPromise;
    }

    loadingPromise = (async () => {
        const result = await reqAnimeByCDN('/data/ANiMeRecord.json');
        const text = await result.blob.text();
        const data = parse<ANiMeRecord[]>(text);

        cachedRecords = Object.freeze(data);
        return cachedRecords;
    })();

    return loadingPromise;
}

function loadActorMap (): Promise<Map<number, Actor>> {
    if (actorMap) {
        return Promise.resolve(actorMap);
    }
    
    if (loadingActorPromise) {
        return loadingActorPromise;
    }
    
    loadingActorPromise = (async () => {
        const result = await reqAnimeByCDN('/data/Actor.json');
        const text = await result.blob.text();
        const data = parse<Actor[]>(text);
        
        const map = new Map<number, Actor>();
        data.forEach(actor => {
            map.set(actor.id, actor);
        });
        
        actorMap = map;
        return actorMap;
    })();
    
    return loadingActorPromise;
}

export function useAnimeRecords (baseUrl: string) {
    const [data, setData] = useState<readonly ANiMeRecord[]>([]);

    useEffect(() => {
        loadAnimeRecords().then(setData);
    }, [baseUrl]);

    return data;
}

export function useActorMap(baseUrl: string) {
    const [data, setData] = useState<Map<number, Actor>>(new Map());
    
    useEffect(() => {
        loadActorMap().then(setData);
    }, [baseUrl]);
    
    return data;
}

export function useAnimeRecordMap(baseUrl: string) {
    const records = useAnimeRecords(baseUrl);
    const [data, setData] = useState<Map<number, ANiMeRecord>>(new Map());

    useEffect(() => {
        const map = new Map<number, ANiMeRecord>();
        records.forEach(record => {
            map.set(record.anime_data.id, record);
        });
        setData(map);
    }, [records]);

    return data;
}