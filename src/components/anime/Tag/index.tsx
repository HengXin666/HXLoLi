import React from "react";
import { ReactNode } from "react";

export default function Tag ({ 
    text,
    style = {}
}: { text: string, style?: React.CSSProperties }): ReactNode {
    return (
        <span
            style={{
                alignItems: "center",
                padding: "2px 8px",
                borderRadius: "999px",
                backgroundColor: "rgba(255, 255, 255, 0.1)",
                fontSize: "13px",
                color: "white",
                cursor: "default",
                border: "var(--ifm-color-primary) 1px solid",
                fontFamily: "var(--ifm-font-family-monospace)",
                ...style
            }}
        >
            {text}
        </span>
    );
}