import { useLocation } from '@docusaurus/router';
import config from '@generated/docusaurus.config';
import React, { useEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom';
import { FaCompress, FaExpand, FaExternalLinkAlt, FaPlay, FaTimes } from 'react-icons/fa';

import styles from './styles.module.css';

interface PptHtmlViewerProps {
    title: string;
    src: string;
    width?: string;
}

function splitUrlSuffix (url: string): { pathPart: string; suffix: string } {
    const suffixIndex = url.search(/[?#]/);
    if (suffixIndex === -1) {
        return { pathPart: url, suffix: '' };
    }

    return {
        pathPart: url.slice(0, suffixIndex),
        suffix: url.slice(suffixIndex),
    };
}

function addBaseUrl (pathname: string): string {
    const baseUrl = config.baseUrl.replace(/\/$/, '');
    if (!baseUrl || baseUrl === '/') return pathname;
    if (pathname === baseUrl || pathname.startsWith(`${baseUrl}/`)) return pathname;

    return `${baseUrl}${pathname.startsWith('/') ? '' : '/'}${pathname}`;
}

function toCleanPptPath (pathPart: string): string {
    return pathPart.replace(/\.html?$/i, '');
}

function resolvePptSrc (src: string, currentPathname: string): string {
    if (!src || /^(?:[a-z][a-z\d+.-]*:|\/\/|#)/i.test(src)) {
        return src;
    }

    const { pathPart, suffix } = splitUrlSuffix(src);
    const cleanPathPart = toCleanPptPath(pathPart);

    if (cleanPathPart.startsWith('/')) {
        return `${addBaseUrl(cleanPathPart)}${suffix}`;
    }

    const currentDirectoryPath = addBaseUrl(currentPathname).endsWith('/')
        ? addBaseUrl(currentPathname)
        : `${addBaseUrl(currentPathname)}/`;
    const resolvedPath = new URL(cleanPathPart, `https://hxloli.local${currentDirectoryPath}`).pathname;
    return `${resolvedPath}${suffix}`;
}

export default function PptHtmlViewer ({
    title,
    src,
    width = '80%',
}: PptHtmlViewerProps): React.ReactNode {
    const location = useLocation();
    const [isOpen, setIsOpen] = useState(false);
    const [isFullscreen, setIsFullscreen] = useState(false);
    const modalShellRef = useRef<HTMLDivElement | null>(null);
    const displayTitle = title || 'PPT HTML';
    const resolvedSrc = resolvePptSrc(src, location.pathname);

    useEffect(() => {
        if (!isOpen) return;

        const previousOverflow = document.body.style.overflow;
        const handleKeyDown = (event: KeyboardEvent) => {
            if (event.key === 'Escape' && !document.fullscreenElement) {
                setIsOpen(false);
            }
        };
        const handleFullscreenChange = () => {
            setIsFullscreen(document.fullscreenElement === modalShellRef.current);
        };

        document.body.style.overflow = 'hidden';
        document.addEventListener('keydown', handleKeyDown);
        document.addEventListener('fullscreenchange', handleFullscreenChange);

        return () => {
            document.body.style.overflow = previousOverflow;
            document.removeEventListener('keydown', handleKeyDown);
            document.removeEventListener('fullscreenchange', handleFullscreenChange);
        };
    }, [isOpen]);

    const closeModal = () => {
        if (document.fullscreenElement === modalShellRef.current) {
            void document.exitFullscreen();
        }
        setIsOpen(false);
    };

    const toggleFullscreen = () => {
        const shell = modalShellRef.current;
        if (!shell) return;

        if (document.fullscreenElement === shell) {
            void document.exitFullscreen();
            return;
        }

        void shell.requestFullscreen();
    };

    const iframe = (
        <iframe
            className={styles.iframe}
            src={resolvedSrc}
            title={displayTitle}
            loading="lazy"
            allow="fullscreen"
            allowFullScreen
        />
    );

    return (
        <>
            <span
                className={styles.viewer}
                style={{ width }}
            >
                <span className={styles.toolbar}>
                    <span className={styles.title} title={displayTitle}>
                        {displayTitle}
                    </span>
                    <span className={styles.actions}>
                        <button
                            type="button"
                            className={styles.actionButton}
                            onClick={() => setIsOpen(true)}
                            title="打开"
                            aria-label={`打开 ${displayTitle}`}
                        >
                            <FaPlay aria-hidden="true" />
                        </button>
                        <a
                            className={styles.actionButton}
                            href={resolvedSrc}
                            target="_blank"
                            rel="noopener noreferrer"
                            title="新标签页打开"
                            aria-label={`新标签页打开 ${displayTitle}`}
                        >
                            <FaExternalLinkAlt aria-hidden="true" />
                        </a>
                    </span>
                </span>

                <span
                    className={styles.preview}
                    onClick={() => setIsOpen(true)}
                    role="button"
                    tabIndex={0}
                    onKeyDown={(event) => {
                        if (event.key === 'Enter' || event.key === ' ') {
                            event.preventDefault();
                            setIsOpen(true);
                        }
                    }}
                    aria-label={`打开 ${displayTitle}`}
                >
                    {iframe}
                    <span className={styles.previewOverlay}>
                        <FaPlay aria-hidden="true" />
                        <span>打开</span>
                    </span>
                </span>
            </span>

            {isOpen &&
                ReactDOM.createPortal(
                    <div className={styles.modalOverlay} onMouseDown={closeModal}>
                        <div
                            ref={modalShellRef}
                            className={styles.modalShell}
                            onMouseDown={(event) => event.stopPropagation()}
                        >
                            <div className={styles.modalToolbar}>
                                <span className={styles.modalTitle} title={displayTitle}>
                                    {displayTitle}
                                </span>
                                <div className={styles.modalActions}>
                                    <a
                                        className={styles.modalButton}
                                        href={resolvedSrc}
                                        target="_blank"
                                        rel="noopener noreferrer"
                                        title="新标签页打开"
                                        aria-label={`新标签页打开 ${displayTitle}`}
                                    >
                                        <FaExternalLinkAlt aria-hidden="true" />
                                    </a>
                                    <button
                                        type="button"
                                        className={styles.modalButton}
                                        onClick={toggleFullscreen}
                                        title={isFullscreen ? '退出全屏' : '全屏'}
                                        aria-label={isFullscreen ? '退出全屏' : '全屏'}
                                    >
                                        {isFullscreen ? (
                                            <FaCompress aria-hidden="true" />
                                        ) : (
                                            <FaExpand aria-hidden="true" />
                                        )}
                                    </button>
                                    <button
                                        type="button"
                                        className={styles.modalButton}
                                        onClick={closeModal}
                                        title="关闭"
                                        aria-label="关闭"
                                    >
                                        <FaTimes aria-hidden="true" />
                                    </button>
                                </div>
                            </div>
                            <div className={styles.modalFrame}>
                                <iframe
                                    className={styles.iframe}
                                    src={resolvedSrc}
                                    title={displayTitle}
                                    allow="fullscreen"
                                    allowFullScreen
                                />
                            </div>
                        </div>
                    </div>,
                    document.body
                )}
        </>
    );
}
