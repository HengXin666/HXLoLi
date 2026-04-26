import BlogSidebar from '@theme/BlogSidebar';
import Layout from '@theme/Layout';
import React, { useEffect, useState, type ReactNode } from 'react';

import type { Props } from '@theme/BlogLayout';

export default function BlogLayout (props: Props): ReactNode {
    const { sidebar, toc, children, ...layoutProps } = props;
    const hasSidebar = sidebar && sidebar.items.length > 0;
    
    const [isMobile, setIsMobile] = useState<boolean>(() => {
        if (typeof window !== "undefined") {
            return window.innerWidth <= 996;
        }
        return false;
    });

    useEffect(() => {
        const handleResize = () => setIsMobile(window.innerWidth <= 996);
        window.addEventListener("resize", handleResize);
        return () => window.removeEventListener("resize", handleResize);
    }, []);

    return (
        <Layout {...layoutProps}>
            <div className="row" style={{ width: '100%', padding: 0, margin: 0 }}>
                <BlogSidebar sidebar={sidebar} />
                <main className={'col'} style={{ padding: 0 }}>
                    <div className="row" style={{ padding: 0, margin: 0 }}>
                        <div className="col" style={{ 
                            padding: 0,
                            maxWidth: isMobile ? '100%' : '78.25%',
                            width: '100%',
                        }}>
                            {children}
                        </div>
                        {!isMobile && (
                            toc
                                ? <div className="col" style={{
                                    padding: '0',
                                    maxWidth: '21.75%'
                                }}>{toc}</div>
                                : <div className="col" style={{
                                    padding: '0',
                                    maxWidth: '21.75%'
                                }} />
                        )}
                    </div>
                </main>
            </div>
        </Layout>
    );
}
