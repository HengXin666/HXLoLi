import React, { useEffect, useRef, useState } from 'react';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
interface TypewriterTextProps {
  text: string;
  speed?: number;
  className?: string;
  cursorClassName?: string;
  onComplete?: () => void;
}

/**
 * 打字机特效组件
 * 逐字显示文本, 带闪烁光标
 */
export default function TypewriterText({
  text,
  speed = 60,
  className,
  cursorClassName,
  onComplete,
}: TypewriterTextProps) {
  const [displayed, setDisplayed] = useState('');
  const [showCursor, setShowCursor] = useState(true);
  const idxRef = useRef(0);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    idxRef.current = 0;
    setDisplayed('');

    timerRef.current = setInterval(() => {
      idxRef.current++;
      if (idxRef.current <= text.length) {
        setDisplayed(text.slice(0, idxRef.current));
      } else {
        if (timerRef.current) clearInterval(timerRef.current);
        onComplete?.();
      }
    }, speed);

    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
    };
  }, [text, speed]);

  // 光标闪烁
  useEffect(() => {
    const blink = setInterval(() => setShowCursor((c) => !c), 530);
    return () => clearInterval(blink);
  }, []);

  return (
    <span className={className} style={{ display: 'inline' }}>
      {displayed}
      <span
        className={cursorClassName}
        style={{
          opacity: showCursor ? 1 : 0,
          transition: 'opacity 0.08s',
        }}
      >
        ▌
      </span>
    </span>
  );
}
