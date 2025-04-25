import React, { useEffect, useState } from "react";

/**
 * @copyright https://github.com/Casta-mere/My-Website/blob/master/src/components/ProgressBar/index.tsx
 */

const ProgressBar = () => {
    const [progress, setProgress] = useState(0);

    const handleScroll = () => {
        const scrollTop = window.scrollY;
        const scrollHeight =
            document.documentElement.scrollHeight -
            document.documentElement.clientHeight;
        const calculatedProgress = Math.ceil((scrollTop / scrollHeight) * 100);
        setProgress(calculatedProgress);
    };

    useEffect(() => {
        window.addEventListener("scroll", handleScroll);
        return () => {
            window.removeEventListener("scroll", handleScroll);
        };
    }, []);

    return (
        <div className="tailwind">
            <div className="text-fuchsia-400" style={{borderLeft: '1px solid var(--ifm-toc-border-color)'}}>
                <div 
                    className="ml-4 -mb-2" 
                    style={{borderBottom: '1.5px solid var(--ifm-toc-border-color)'}}
                >
                    {progress}%
                </div>
            </div>
        </div>
    );
};

export default ProgressBar;