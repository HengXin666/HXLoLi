/**
 * 侧边栏重建后的「仅高亮变化项」动画协调器.
 *
 * 流程:
 *   - 工具栏点击重建成功 -> diff (added/changed/removed) 存 sessionStorage (hx-sidebar-flash)
 *   - 页面 reload (新增文档需 Docusaurus 重新生成路由/元数据)
 *   - reload 后组件挂载时调用 flashSidebarChanges():
 *       轮询等待侧边栏渲染稳定, 给每个 added/changed 命中项加动画类;
 *       每个项加类时立即安排独立的类移除定时器 (与轮询状态无关, 不会漏清).
 *   - 折叠分类里的新增项会先被展开 (点击 menu__link), 让 lazy 子项挂载后再高亮.
 */

const FLASH_KEY = 'hx-sidebar-flash';
const SCAN_MS = 6000;      // 最长等待侧边栏渲染
const ANIM_MS = 2600;      // 类保留时长 (略大于 CSS 动画)

interface FlashState {
  at: number;
  added: string[];
  changed: string[];
  removed: string[];
}

interface RunningWatch {
  timer: number;
  timeout: number;
}

let runningWatch: RunningWatch | null = null;

export function rememberFlash (state: FlashState): void {
  try {
    sessionStorage.setItem(FLASH_KEY, JSON.stringify(state));
  } catch { /* ignore */ }
}

/** 读回并消费待播 flash (超过 60s 视为过期) */
function readFlash (): FlashState | null {
  try {
    const raw = sessionStorage.getItem(FLASH_KEY);
    if (!raw) return null;
    const state = JSON.parse(raw) as FlashState;
    if (!state || typeof state.at !== 'number') return null;
    if (Date.now() - state.at > 60_000) {
      sessionStorage.removeItem(FLASH_KEY);
      return null;
    }
    return state;
  } catch {
    return null;
  }
}

function clearFlash (): void {
  try { sessionStorage.removeItem(FLASH_KEY); } catch { /* ignore */ }
}

/** 规范化: 去掉首尾斜杠/空白 */
function normalize (p: string): string {
  return p.replace(/^\/+|\/+$/g, '').trim();
}

/**
 * 给单个元素加动画类, 并立即为该元素安排独立的类移除.
 * 加类返回 true (本轮有新增), 已加过返回 false.
 */
function applyFlashClass (el: HTMLElement, cls: string): boolean {
  if (el.classList.contains(cls)) return false;
  el.classList.add(cls);
  // 独立定时器: 无论轮询/后续状态如何, 这个元素到时必定清理
  window.setTimeout(() => {
    try {
      el.classList.remove('hx-sidebar-insert', 'hx-sidebar-change');
    } catch { /* ignore */ }
  }, ANIM_MS);
  return true;
}

/** 展开 key 路径上所有「当前折叠」的分类祖先, 让 lazy 子项挂载 */
function expandCollapsedAncestors (key: string): boolean {
  const parts = normalize(key).split('/');
  let acc = '';
  let expandedAny = false;
  for (let i = 0; i < parts.length - 1; i++) {
    acc = acc ? acc + '/' + parts[i] : parts[i];
    const el = Array.from(document.querySelectorAll<HTMLElement>('[data-sidebar-path]'))
      .find((e) => normalize(e.getAttribute('data-sidebar-path') || '') === acc);
    if (!el) continue;
    const cls = el.className || '';
    if (cls.includes('menu__list-item--collapsed')) {
      const link = el.querySelector<HTMLElement>('a.menu__link');
      if (link) {
        link.click();
        expandedAny = true;
      }
    }
  }
  return expandedAny;
}

/** key 的祖先链是否已在 DOM (即只差展开折叠分类这一步) */
function ancestorsInDom (key: string): boolean {
  const parts = normalize(key).split('/');
  let acc = '';
  for (let i = 0; i < parts.length - 1; i++) {
    acc = acc ? acc + '/' + parts[i] : parts[i];
    const hit = Array.from(document.querySelectorAll<HTMLElement>('[data-sidebar-path]'))
      .some((el) => normalize(el.getAttribute('data-sidebar-path') || '') === acc);
    if (!hit) return false;
  }
  return true;
}

/**
 * 扫描一轮: 直接命中 -> 播放; 未命中但祖先链在 -> 展开等待挂载.
 * 返回本轮新增播放数.
 */
function playOnExisting (state: FlashState): number {
  let count = 0;
  const all = Array.from(document.querySelectorAll<HTMLElement>('[data-sidebar-path]'));
  const keys = state.added.map((k) => ({ kind: 'insert', key: k }))
    .concat(state.changed.map((k) => ({ kind: 'change', key: k })));
  for (const { kind, key } of keys) {
    const k = normalize(key);
    const direct = all.find((el) => normalize(el.getAttribute('data-sidebar-path') || '') === k);
    if (direct) {
      const cls = kind === 'insert' ? 'hx-sidebar-insert' : 'hx-sidebar-change';
      if (applyFlashClass(direct, cls)) count++;
      continue;
    }
    // 直接元素不在 DOM: 若祖先链存在说明只是折叠 lazy 未挂载 -> 展开
    if (ancestorsInDom(k)) {
      expandCollapsedAncestors(k);
    }
  }
  return count;
}

function stopWatch (): void {
  if (runningWatch) {
    window.clearInterval(runningWatch.timer);
    window.clearTimeout(runningWatch.timeout);
    runningWatch = null;
  }
}

export function flashSidebarChanges (): void {
  const state = readFlash();
  if (!state) return;
  stopWatch();

  const startedAt = Date.now();
  let pending = true;

  // 每 150ms 扫一轮, 直到: 全部命中过 (播放完) 或超时
  const timer = window.setInterval(() => {
    const hits = playOnExisting(state);
    const now = Date.now();
    const anyFlashLeft = !!document.querySelector('.hx-sidebar-insert, .hx-sidebar-change');
    if (hits === 0 && !anyFlashLeft) {
      // 没有新增也没有在播的 -> 要么全播完(等清理), 要么没可播的
      pending = false;
      stopWatch();
      clearFlash();
      return;
    }
    if (now - startedAt > SCAN_MS) {
      pending = false;
      stopWatch();
      return;
    }
  }, 150);

  // 兜底: 最迟 SCAN_MS 后停止轮询, 类由各自的独立定时器清理
  const timeout = window.setTimeout(() => {
    stopWatch();
  }, SCAN_MS + 800);

  runningWatch = { timer, timeout };
}

/** 页面离开 / 组件卸载时兜底清理 */
export function cancelSidebarFlash (): void {
  stopWatch();
}
