import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import BlogSidebar from '@theme/BlogSidebar';

import type { Props } from '@theme/BlogLayout';

export default function BlogLayout (props: Props): ReactNode {
    const { sidebar, toc, children, ...layoutProps } = props;
    const hasSidebar = sidebar && sidebar.items.length > 0; // 左侧边栏
    return (
        <Layout {...layoutProps}>
            <div className="row" style={{ width: '100%' }}>
                <BlogSidebar sidebar={sidebar} />
                <main
                    className={'col'}
                >
                    <div className="row" style={{width: '100%', padding: '0'}}>
                        <div className="col" style={{maxWidth: '78.23%', padding: '0'}}>
                            {children}
                        </div>
                        {
                            toc
                                ? <div className="col" style={{
                                    padding: '0',
                                    flex: '0 0 21.75%',
                                    maxWidth: '21.75%'
                                }}>{toc}</div>
                                : <div className="col" style={{
                                    padding: '0',
                                    flex: '0 0 21.75%',
                                    maxWidth: '21.75%'
                                }} />
                        }
                    </div>
                </main>
            </div>
        </Layout>
    );
}
