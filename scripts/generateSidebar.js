const fs = require('fs-extra');
const path = require('path');

const docsDir = path.join(__dirname, '../docs');
const sidebar = {
    tutorialSidebar: []
};

const baseUrl = "/HXLoLiHub";

// 默认图标路径，位于 static/icons 下
const defaultFolderIcon = `${baseUrl}/default-icons/default_folder.svg`;  // 默认文件夹图标
const defaultDocIcon = `${baseUrl}/default-icons/file_type_markdown.svg`;  // 默认文档图标

function stripPrefix(name) {
    return name.replace(/^\d+[-_]/, ''); // 去掉类似 01- 前缀
}

// 获取json配置, 如果有 tag.json 就使用该配置
function getJsonTagConfig(folderPath) {
    const tagJsonPath = path.join(folderPath, 'tag.json');
    if (fs.existsSync(tagJsonPath)) {
        try {
            const data = fs.readFileSync(tagJsonPath, 'utf-8');
            const config = JSON.parse(data);
            return {
                icon: `${baseUrl}/icons/${config.icon}`,
                tags: config.tags
            };
        } catch (err) {
            console.error('Error reading or parsing tag.json:', err);
        }
    }
    return {
        icon: undefined,
        tags: undefined
    };  // 如果没有图标, 返回 undefined
}

function scanDocs(dir, relativePath = '') {
    const entries = fs.readdirSync(dir);
    const items = [];
    let hasIndex = false;

    for (const entry of entries) {
        const fullPath = path.join(dir, entry);
        const stat = fs.statSync(fullPath);

        if (stat.isDirectory()) {
            const cleanLabel = stripPrefix(entry);
            const folderRelative = path.join(relativePath, entry);

            // 递归一下
            const result = scanDocs(fullPath, folderRelative);

            const {
                icon = result.items.length ? defaultFolderIcon : defaultDocIcon,
                tags = []
            } = getJsonTagConfig(fullPath);

            const subCategory = {
                type: 'category',
                label: cleanLabel,
                collapsible: true,
                items: result.items,
                customProps: {
                    icon: icon,
                    tags: tags
                },
            };

            if (result.hasIndex) {
                const id = path.posix.join(
                    ...folderRelative.split(path.sep).map(stripPrefix),
                    'index'
                );
                subCategory.link = {
                    type: 'doc',
                    id: id.replace(/\\/g, '/')
                };
            }

            items.push(subCategory);
        } else if (entry === 'index.md') {
            hasIndex = true;
        } else if (entry.endsWith('.md')) {
            const id = path.posix.join(
                ...relativePath.split(path.sep).map(stripPrefix),
                stripPrefix(entry.replace(/\.md$/, ''))
            );
            // 普通文档没有图标
            items.push({
                type: 'doc',
                id: id,
            });
        }
    }

    return { items, hasIndex };
}

sidebar.tutorialSidebar = scanDocs(docsDir).items;

const sidebarContent = `module.exports = ${JSON.stringify(sidebar, null, 2)};\n`;
fs.outputFileSync(path.join(__dirname, '../sidebars.ts'), sidebarContent);

console.log('[OK]: sidebars.ts 已自动生成');
