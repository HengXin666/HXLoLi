#!/usr/bin/env node
// 角色立绘提示词的机械校验: 用仓库自带的 typescript 把 src/utils/anime/characterPrompt.ts 转成 JS 再执行, 逐条断言.
// 运行 (在 HXLoLi 目录): node src/components/anime/CharacterMenu/characterPrompt.check.mjs
// 退出码 0 通过, 1 有断言失败. 不启动站点, 不发网络请求.
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const src = resolve(here, "../../../utils/anime/characterPrompt.ts");
const require = createRequire(import.meta.url);
const ts = require("typescript");

/** 转译并加载 characterPrompt.ts (它不 import 任何模块, 所以可以直接 eval); 决策见同目录 CharacterContextMenu.tsx 的 CharacterMenuScope 注释 */
function loadPromptModule () {
    const js = ts.transpileModule(readFileSync(src, "utf8"), {
        compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 },
    }).outputText;
    const mod = { exports: {} };
    new Function("module", "exports", js)(mod, mod.exports);
    return mod.exports;
}

const { buildPeekPrompt, buildCharacterPrompt, buildAllPrompts, PEEK_POSE, PEEK_NEGATIVE } = loadPromptModule();

// 真实数据: ANiMeRecord.json (CDN tag HX-20261102180459-V1HA7J) 里的 Fate/strange Fake (bgm 443831) · 吉尔伽美什 (bgm 1227)
const input = {
    animeName: "Fate/strange Fake",
    coverUrl: "https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/anime/443831.jpg",
    coverFallbackUrl: "https://lain.bgm.tv/pic/cover/l/f0/51/443831_CNG0F.jpg",
    characterName: "吉尔伽美什",
    characterOriginalName: "ギルガメッシュ",
    characterImageUrl: "https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/kyara/1227.jpg",
    characterImageFallbackUrl: "https://lain.bgm.tv/pic/crt/l/db/ca/1227_crt_Dq9kZ.jpg?r=1498989532",
    relation: "主角",
};

let failed = 0;
const check = (name, cond) => {
    if (cond) console.log(`ok   ${name}`);
    else { failed++; console.log(`FAIL ${name}`); }
};

for (const side of ["left", "right"]) {
    const p = buildPeekPrompt(input, side);
    check(`${side}: 含番剧封面 URL`, p.includes(input.coverUrl));
    check(`${side}: 含番剧名`, p.includes(input.animeName));
    check(`${side}: 含角色名`, p.includes(input.characterName));
    check(`${side}: 含角色图片 URL`, p.includes(input.characterImageUrl));
    check(`${side}: 含姿势描述`, p.includes(PEEK_POSE));
    check(`${side}: 构图约束 俯探 / 半身出画 / 朝内 / 竖直裁切线在内侧`,
        ["俯探身子", "半身出画", "朝向页面内侧", "竖直裁切线", "(内侧)"].every((k) => p.includes(k)));
    check(`${side}: 透明背景`, p.includes("透明背景"));
}
const left = buildPeekPrompt(input, "left");
const right = buildPeekPrompt(input, "right");
check("左: 身体朝右, 裁切线在右边缘", left.includes("身体与脸朝右") && left.includes("裁切线在画面右边缘"));
check("右: 身体朝左, 裁切线在左边缘", right.includes("身体与脸朝左") && right.includes("裁切线在画面左边缘"));

const all = buildAllPrompts(input);
check("buildAllPrompts 返回 left / right / character", all.left === left && all.right === right && all.character === buildCharacterPrompt(input));
check("探头两份都带完整负面清单", PEEK_NEGATIVE.length === 5 && PEEK_NEGATIVE.every((x) => left.includes(x) && right.includes(x)));

const solo = all.character;
check("独立立绘: 含角色名与角色图片 URL", solo.includes(input.characterName) && solo.includes(input.characterImageUrl));
check("独立立绘: 透明底 / 无背景", solo.includes("透明背景") && solo.includes("背景、文字"));
check("独立立绘: 不涉及探头构图", !solo.includes("俯探") && !solo.includes("裁切线") && !solo.includes("半身出画"));

const noOrig = buildCharacterPrompt({ ...input, characterOriginalName: input.characterName, characterImageFallbackUrl: undefined });
check("原名与译名相同时不重复, 没有备用地址时不输出备用行", !noOrig.includes(`(${input.characterName})`) && !noOrig.includes("备用"));

if (process.argv.includes("--print")) {
    console.log(`\n${JSON.stringify(input, null, 2)}\n\n${all.left}\n\n=====\n\n${all.right}\n\n=====\n\n${all.character}`);
}
console.log(failed ? `\n${failed} 项失败` : "\n全部通过");
process.exit(failed ? 1 : 0);
