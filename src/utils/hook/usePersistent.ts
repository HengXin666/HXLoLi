import { useEffect, useState } from "react";

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export interface PersistCodec<T> {
    serialize: (value: T) => string;
    deserialize: (raw: string) => T;
}

export function usePersistent<T> (
    key: string,
    defaultValue: T,
    codec?: PersistCodec<T>
): [T, React.Dispatch<React.SetStateAction<T>>] {
    const { serialize, deserialize } = codec ?? {
        serialize: JSON.stringify,
        deserialize: JSON.parse
    };

    const [value, setValue] = useState<T>(() => {
        try {
            const raw = localStorage.getItem(key);
            return raw !== null ? deserialize(raw) : defaultValue;
        } catch {
            return defaultValue;
        }
    });

    useEffect(() => {
        try {
            localStorage.setItem(key, serialize(value));
        } catch {
            // 忽略写入失败(配额、隐私模式等)
        }
    }, [key, value, serialize]);

    return [value, setValue];
}
