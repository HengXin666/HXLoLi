import { parse } from "@site/src/utils/anime/parser";
import { ANiMeRecord } from "@site/src/utils/anime/types";
import { useState, useEffect } from "react";

let cachedRecords: readonly ANiMeRecord[] | null = null;
let loadingPromise: Promise<readonly ANiMeRecord[]> | null = null;

export function loadAnimeRecords (
    baseUrl: string
): Promise<readonly ANiMeRecord[]> {
    if (cachedRecords) {
        return Promise.resolve(cachedRecords);
    }

    if (loadingPromise) {
        return loadingPromise;
    }

    loadingPromise = (async () => {
        const response = await fetch(`${baseUrl}anime/ANiMeRecord.json`);
        if (!response.ok) {
            throw new Error(response.statusText);
        }
        const data = parse<ANiMeRecord[]>(await response.text());

        cachedRecords = Object.freeze(data);
        return cachedRecords;
    })();

    return loadingPromise;
}

export function useAnimeRecords (baseUrl: string) {
    const [data, setData] = useState<readonly ANiMeRecord[]>([]);

    useEffect(() => {
        loadAnimeRecords(baseUrl).then(setData);
    }, [baseUrl]);

    return data;
}
