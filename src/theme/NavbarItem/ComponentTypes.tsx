/**
 * Swizzle NavbarItem/ComponentTypes 注册自定义 Navbar Item
 */
import { MusicNavbarButton } from '@site/src/components/MusicPlayer/MusicPlayerBar';
import { CDNNavbarButton } from '@site/src/components/CDNNodeSelector';
import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';

/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
const CustomComponentTypes = {
    ...ComponentTypes,
    'custom-musicPlayer': MusicNavbarButton,
    'custom-cdnSelector': CDNNavbarButton,
};

export default CustomComponentTypes;
