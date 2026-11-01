// 由 HXLoLi-ANiMe 仓库 GitHub Actions 自动发 PR 更新
// Tag 格式: HX-{随机6位}-{时间戳}
// 2026-11-01: 原值 HX-20260906113630-E1EgjV 在远端已被删 (HXLoLi-ANiMe 的 Tag 清理
// 只保留最新 2 个), jsdmirror 与 raw.githubusercontent 双双 404, 线上番剧数据一直是
// 断的. 改用远端现存的 HX-20261101141415-poqRvw (git 内 13,486,403 字节, 177 部).
// 清理策略已同时改为保留最新 5 个, 见 HXLoLi-ANiMe .github/workflows/anime-pipeline.yaml.
export const ANIME_CDN_TAG = 'HX-20261101141415-poqRvw';
