/**
 * AI 知识库侧边栏自动生成脚本
 *
 * 扫描 ai-docs/ 目录, 自动生成 sidebarsAiDocs.ts
 * 用法: node scripts/generateAiDocsSidebar.js
 *
 * 类似 scripts/generateSidebar.js, 但针对 AI 知识库的目录结构
 */

const fs = require('fs-extra');
const path = require('path');

const docsDir = path.join(__dirname, '../ai-docs');

// 图标路径不带 baseUrl 前缀, 运行时由组件根据实际 baseUrl 拼接
const defaultFolderIcon = 'default-icons/ai-folder.svg';      // AI 主题文件夹图标
const defaultDocIcon = 'default-icons/ai-doc.svg';            // AI 主题文档图标

function stripPrefix(name) {
    return name.replace(/^\d+[-_]/, '');
}

function getJsonTagConfig(folderPath) {
    const tagJsonPath = path.join(folderPath, 'tag.json');
    if (fs.existsSync(tagJsonPath)) {
        try {
            const data = fs.readFileSync(tagJsonPath, 'utf-8');
            const config = JSON.parse(data);
            return {
                icon: config.icon ? `icons/${config.icon}` : undefined,
                tags: Array.isArray(config.tags) ? config.tags : []
            };
        } catch (err) {
            console.error('[tag.json] 无法解析:', err);
        }
    }
    return { icon: undefined, tags: [] };
}

function scanDocs(dir, relativePath = '') {
    if (!fs.existsSync(dir)) return { items: [], hasIndex: false };

    const entries = fs.readdirSync(dir);
    const items = [];
    let hasIndex = false;

    for (const entry of entries) {
        const fullPath = path.join(dir, entry);
        const stat = fs.statSync(fullPath);

        if (entry.startsWith('.')) continue;

        if (stat.isDirectory()) {
            const cleanLabel = stripPrefix(entry);
            const folderRelative = path.join(relativePath, entry);
            const result = scanDocs(fullPath, folderRelative);

            const { icon: tagIcon, tags = [] } = getJsonTagConfig(fullPath);
            const icon = tagIcon ?? (result.items.length > 0 ? defaultFolderIcon : defaultDocIcon);

            const subCategory = {
                type: 'category',
                label: cleanLabel,
                collapsible: true,
                collapsed: false,
                items: result.items,
                customProps: { icon, tags },
            };

            if (result.hasIndex) {
                const id = path.posix.join(
                    ...folderRelative.split(path.sep).map(stripPrefix),
                    'index'
                );
                subCategory.link = { type: 'doc', id: id.replace(/\\/g, '/') };
            }

            if (result.items.length > 0 || result.hasIndex) {
                items.push(subCategory);
            }
        } else if (entry === 'index.md' || entry === 'index.mdx') {
            hasIndex = true;
        } else if (entry.endsWith('.md') || entry.endsWith('.mdx')) {
            const id = path.posix.join(
                ...relativePath.split(path.sep).map(stripPrefix),
                stripPrefix(entry.replace(/\.mdx?$/, ''))
            );
            items.push({ type: 'doc', id: id });
        }
    }

    return { items, hasIndex };
}

const sidebar = { aiDocsSidebar: scanDocs(docsDir).items };

const sidebarContent = `// 由 scripts/generateAiDocsSidebar.js 自动生成\n// 请勿手动编辑 — 运行 node scripts/generateAiDocsSidebar.js 以更新\n\nconst aiDocsSidebar = ${JSON.stringify(sidebar.aiDocsSidebar, null, 2)};\n\nexport default { aiDocsSidebar };\n`;

fs.outputFileSync(path.join(__dirname, '../sidebarsAiDocs.ts'), sidebarContent);

console.log('[OK]: sidebarsAiDocs.ts 已自动生成 (ai-docs/)');
