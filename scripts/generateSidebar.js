const fs = require('fs-extra');
const path = require('path');

const docsDir = path.join(__dirname, '../docs');
const sidebar = {
    tutorialSidebar: []
};

const baseUrl = "/HXLoLi";

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
            if (config.icon) {
                console.log(`找到图标: ${baseUrl}/icons/${config.icon}`);
            }
            return {
                icon: config.icon ? `${baseUrl}/icons/${config.icon}` : undefined,
                tags: Array.isArray(config.tags) ? config.tags : []
            };
        } catch (err) {
            console.error('[tag.json] 无法解析:', err);
        }
    }
    return {
        icon: undefined,
        tags: []
    };
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
                icon: tagIcon,
                tags = []
            } = getJsonTagConfig(fullPath);

            // 合理优先级: tag.json > 是否有子项
            const icon = tagIcon ?? (result.items.length > 0 ? defaultFolderIcon : defaultDocIcon);

            const subCategory = {
                type: 'category',
                label: cleanLabel,
                collapsible: true,
                items: result.items,
                customProps: {
                    icon,
                    tags
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
