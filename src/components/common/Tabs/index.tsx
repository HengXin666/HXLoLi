import React from "react";

export interface TabItem<T extends number | string> {
    value: T;
    label: React.ReactNode;
}

export interface TabsProps<T extends number | string> {
    value: T;
    items: readonly TabItem<T>[];
    onChange: (value: T) => void;
}

export function Tabs<T extends number | string> ({
    value,
    items,
    onChange
}: TabsProps<T>): React.ReactElement {
    return (
        <div
            style={{
                display: "flex",
                gap: "8px",
                padding: "6px",
                backgroundColor: "#1e1e1e",
                borderRadius: "12px",
                width: "fit-content",
                margin: "0 auto",
                marginTop: "10px",
                marginBlock: "10px"
            }}
        >
            {items.map(item => {
                const active = value === item.value;

                return (
                    <button
                        key={String(item.value)}
                        onClick={() => onChange(item.value)}
                        style={{
                            padding: "8px 24px",
                            borderRadius: "10px",
                            border: "none",
                            cursor: "pointer",
                            fontSize: "15px",
                            fontWeight: 750,
                            color: active
                                ? "var(--ifm-color-primary-dark)"
                                : "#b0b0b0",
                            backgroundColor: active
                                ? "#3a3a3a"
                                : "transparent",
                            transition: "all 0.2s ease",
                            boxShadow: active
                                ? "0 4px 12px rgba(0, 0, 0, 0.4)"
                                : "none"
                        }}
                    >
                        {item.label}
                    </button>
                );
            })}
        </div>
    );
}
