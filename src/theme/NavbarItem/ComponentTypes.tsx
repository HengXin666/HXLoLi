/**
 * Swizzle NavbarItem/ComponentTypes 注册自定义 Navbar Item
 */
import { MusicNavbarButton } from '@site/src/components/MusicPlayer/MusicPlayerBar';
import { CDNNavbarButton } from '@site/src/components/CDNNodeSelector';
import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';

const CustomComponentTypes = {
    ...ComponentTypes,
    'custom-musicPlayer': MusicNavbarButton,
    'custom-cdnSelector': CDNNavbarButton,
};

export default CustomComponentTypes;
