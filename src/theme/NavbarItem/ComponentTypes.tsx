/**
 * Swizzle NavbarItem/ComponentTypes 注册自定义 Navbar Item
 */
import { MusicNavbarButton } from '@site/src/components/MusicPlayer/MusicPlayerBar';
import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';

const CustomComponentTypes = {
    ...ComponentTypes,
    'custom-musicPlayer': MusicNavbarButton,
};

export default CustomComponentTypes;
