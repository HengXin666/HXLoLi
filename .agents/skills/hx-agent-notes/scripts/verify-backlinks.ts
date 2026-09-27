/**
 * Gate 4 — anchors: source that cites a note path must cite one that still resolves, and (when
 * required) every shipped note must be cited somewhere. Whole-repo sweep, cheap on every change.
 *
 * A "backlink" is any source reference to a note path — repo-relative
 * (.agents/notes/implemented/<class>/<file>.md) or lifecycle-relative
 * (implemented/<class>/<file>.md). There is no magic marker token: a token like the one this gate
 * used to require also appears in ordinary prose, so a convention that fires on English sentences
 * reports phantom violations instead of gaps.
 *
 * Usage: npx tsx scripts/verify-backlinks.ts [--repo <dir>]
 */
import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs'
import { join, relative, resolve } from 'node:path'
import { describe, isNestedWorkTree, loadNotes, matchesAny, parseArgv, walkNotes } from './notes-lib.ts'

const parsed = parseArgv(process.argv.slice(2), ['--repo'])
const cwd = parsed.values['--repo'] !== undefined && parsed.values['--repo'] !== '' ? resolve(parsed.values['--repo'] as string) : process.cwd()
const loaded = loadNotes(cwd)
const { config, notesRoot, repoRoot } = loaded

if (!config.backlinks.enabled) {
  console.log('verify-agent-notes:backlinks: disabled in config')
  process.exit(0)
}

const SKIP_DIRS = new Set(['node_modules', '.git', 'dist', 'build', 'out', 'coverage', '.next', 'vendor', '__pycache__', '.venv', 'venv', 'target'])
const errors: string[] = []

function readdirSafe(dir: string): { name: string; isDirectory(): boolean; isFile(): boolean }[] {
  try {
    return readdirSync(dir, { withFileTypes: true }) as unknown as { name: string; isDirectory(): boolean; isFile(): boolean }[]
  } catch {
    return []
  }
}

function* implementationFiles(): Generator<string> {
  const stack = config.backlinks.roots.map((root) => join(repoRoot, root)).filter((path) => existsSync(path))
  while (stack.length > 0) {
    const dir = stack.pop() as string
    for (const entry of readdirSafe(dir)) {
      const full = join(dir, entry.name)
      if (entry.isDirectory()) {
        // A nested work tree keeps its own notes; its citations are not ours to grade.
        if (!SKIP_DIRS.has(entry.name) && !isNestedWorkTree(full, loaded.gitRoot)) stack.push(full)
        continue
      }
      if (!entry.isFile()) continue
      const rel = relative(repoRoot, full).split('\\').join('/')
      if (matchesAny(rel, config.backlinks.exclude)) continue
      if (!config.backlinks.extensions.some((ext) => entry.name.endsWith(ext))) continue
      yield full
    }
  }
}

/** 读取某文件的第 n 行 (1-based); 越界返回空串。 */
function lineAt(file: string, n: number): string {
  if (n < 1) return ''
  const lines = readFileSync(file, 'utf8').split('\n')
  return lines[n - 1] ?? ''
}

/** 取某行往上的若干行, 用于判断引用点上方 (或同行) 有没有真实声明。 */
function linesAbove(file: string, n: number, count: number): string {
  const lines = readFileSync(file, 'utf8').split('\n')
  return lines.slice(Math.max(0, n - 1 - count), n).join('\n')
}

/** 相对 repoRoot 的路径, 用于报错信息。 */
function relFileOf(abs: string, root: string): string {
  return relative(root, abs).split('\\').join('/')
}

const rootPattern = config.root.replace(/[.*+?^$()|[\]\\]/g, '\\$&')
const lifecycleAlt = [...config.lifecycles, config.archive].join('|')
const BTN = '`'
const STOP = '\\s)' + "'" + '"' + BTN + ' ,;'
const refPatterns = [
  // Repo-relative, including the path inside a relative markdown link.
  new RegExp(rootPattern + '\\/((?:' + lifecycleAlt + ')\\/[^' + STOP + ']+\\.md)', 'g'),
  // Lifecycle-relative, which is unambiguous because of the dated filename.
  new RegExp('(?:^|[\\s(' + "'" + '"' + BTN + '])((?:' + lifecycleAlt + ')\\/[a-z-]+\\/\\d{4}-\\d{2}-\\d{2}-[^' + STOP + ']+\\.md)', 'g'),
]

const anchored = new Set<string>()
/** 每个引用点的位置: 绝对路径 -> 行号列表。用于"引用要落到具体声明旁"的校验。 */
const citationSites = new Map<string, number[]>()
let referenceCount = 0

for (const file of implementationFiles()) {
  const text = readFileSync(file, 'utf8')
  if (!text.includes('.md')) continue
  const relFile = relative(repoRoot, file).split('\\').join('/')
  text.split('\n').forEach((line, index) => {
    for (const pattern of refPatterns) {
      pattern.lastIndex = 0
      let match: RegExpExecArray | null
      while ((match = pattern.exec(line)) !== null) {
        referenceCount += 1
        const ref = match[1] as string
        const target = ref.startsWith(describe(loaded)) ? join(repoRoot, ref) : join(notesRoot, ref)
        if (!existsSync(target) || !statSync(target).isFile()) {
          errors.push('backlink: ' + relFile + ':' + (index + 1) + ' — ' + ref + ' does not resolve; the note moved or was archived')
          continue
        }
        anchored.add(resolve(target))
        const sites = citationSites.get(file) ?? []
        sites.push(index + 1)
        citationSites.set(file, sites)
      }
    }
  })
}

if (config.backlinks.required) {
  const { notes } = walkNotes(loaded)
  for (const note of notes) {
    if (note.lifecycle !== 'implemented') continue
    const noteAbs = resolve(notesRoot, note.rel)
    const noteText = readFileSync(noteAbs, 'utf8')
    // 内容类 note (约束的是 ai-docs 内容组织而非代码声明) 可以显式声明没有源码落点。
    // 这不是绕过门禁: 它要求作者**写下一个断言**, 而不是让引用悄悄缺席。
    const contentOnly = /^- .*\*\*引用落点\*\*:\s*无源码引用/m.test(noteText)
    if (!anchored.has(noteAbs)) {
      if (contentOnly) continue
      errors.push(
        'backlink: ' + note.rel + ' — no source file cites this shipped decision. ' +
          '在**声明旁边**引它, 不要写在文件头: ' +
          "在函数/JSDoc/类定义的正上方或同行, 用 (see <note 路径>); " +
          '若这条 note 确实不约束任何代码声明 (如只约束 ai-docs 内容组织), ' +
          '在 note 里加一行 "- **引用落点**: 无源码引用 (<理由>)"',
      )
      continue
    }
    // 已落地 note 必须记录它由哪次提交引入 —— 否则读者无法把决策与代码版本对上
    const text = noteText
    // 接受三种写法: "- **引入于**: <sha>" / "- 引入于: <sha>" / "Commit: <sha>"
    const INTRO_RE = /^\s*-?\s*(?:\*\*)?引入于(?:\*\*)?\s*[:：]\s*(?:\*\*)?[@`]?([0-9a-f]{7,40})`?/m
    const COMMIT_RE = /^\s*(?:\*\*)?Commit(?:\*\*)?\s*[:：]\s*[@`]?([0-9a-f]{7,40})`?/m
    if (!INTRO_RE.test(text) && !COMMIT_RE.test(text)) {
      errors.push(
        'backlink: ' + note.rel + ' — 缺少「引入于: <short-sha>」. ' +
          '首次落地时记下这次提交的 sha, 之后不再改',
      )
    }
  }

  // 位置检查: 引用点必须在**具体的声明旁**, 不允许出现在文件头部注释块
  for (const [abs, lines] of citationSites) {
    const noteRel = relative(notesRoot, abs).split('\\').join('/')
    for (const ln of lines) {
      const ctx = lineAt(abs, ln)
      const above = linesAbove(abs, ln, 4)
      // 头部: 前 12 行以内, 且上方没有任何声明 (function/class/const/接口/导出)
      const isTop = ln <= 12
      const hasDeclaration = /\b(function|class|interface|const|let|var|def|export|impl|struct)\b|=>/.test(above + ctx)
      if (isTop && !hasDeclaration) {
        errors.push(
          'backlink: ' + relFileOf(abs, repoRoot) + ':' + ln + ' — 引用落在文件头部, 没有指向具体位置. ' +
            '把它移到它约束的那个声明旁边 (函数/类/JSDoc 的正上方或同行)',
        )
      }
    }
  }
}

if (errors.length > 0) {
  console.error('verify-agent-notes:backlinks: ' + errors.length + ' violation(s)')
  for (const error of errors.slice(0, 25)) console.error('  ' + error)
  if (errors.length > 25) console.error('  … and ' + (errors.length - 25) + ' more')
  console.error('  Cite the note next to the declaration the decision governs:')
  console.error('    ' + "/**" + '… (see the [<topic> note](../../../../' + describe(loaded) + '/implemented/<class>/<yyyy-mm-dd-topic>.md)). ' + "*/")
  process.exit(1)
}

const suffix = config.backlinks.required
  ? 'every implemented note is anchored in source'
  : 'anchors resolve (backlinks.required is off, so an unanchored note is allowed)'
console.log('verify-agent-notes:backlinks: ' + referenceCount + ' reference(s) to ' + anchored.size + ' note(s); ' + suffix)