import React from 'react';
import { Slide } from './Slide';
import { Cover } from './blocks';
import { getDeck, listDecks } from './decks';
import './decks-builtin';

/**
 * 按名取演示页内容.
 *
 * 找不到时必须**明确报错**而不是渲染空白 —— 空白会让人以为是 CSS 问题,
 * 实际是注册表没配对.
 */
export function PptDeckFromCode({ title, name }: { title?: string; name?: string }): React.ReactElement {
    const entry = name ? getDeck(name) : undefined;

    if (!entry) {
        const known = listDecks();
        return (
            <Slide title={title ?? '未注册'} chapter="">
                <Cover
                    eyebrow="PPT"
                    title={title || (name ? `未注册的演示页: ${name}` : '未指定演示页')}
                    subtitle={known.length ? `已注册: ${known.join(' / ')}` : '注册表为空 —— 请先注册一个 deck'}
                />
            </Slide>
        );
    }
    return <>{entry.render()}</>;
}

export default PptDeckFromCode;