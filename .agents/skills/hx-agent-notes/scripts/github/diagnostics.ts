const fs = require('node:fs')
const {createHash} = require('node:crypto')

const ROOT = '<!-- agent-notes:'
const safePath = (value: unknown): value is string => typeof value === 'string' && !!value &&
  !value.startsWith('-') && !/[\\\n\r\t`#?*{}\[\]]/.test(value) &&
  value.split('/').every(part => !['', '.', '..'].includes(part))
const escape = (value: unknown) => String(value).replace(/[\r\n]/g, ' ').replace(/@/g, '@\u200b')
  .replace(/[\[\]<>`*_]/g, '\\$&').slice(0, 800)

function readReport(head: string) {
  const path = '.agent-notes-report/agent-notes-report.json'
  if (fs.statSync(path).size > 2_000_000) throw new Error('Report too large')
  const data = JSON.parse(fs.readFileSync(path, 'utf8'))
  if (!/^[a-f0-9]{40}$/.test(head) || data.head !== head) throw new Error('Report belongs to another commit')
  if (data.version !== 2 || typeof data.ok !== 'boolean' || !Array.isArray(data.issues) ||
      data.ok !== (data.issues.length === 0)) throw new Error('Invalid report')
  if (data.base_commit && !/^[a-f0-9]{40}$/.test(data.base_commit)) throw new Error('Invalid base commit')
  if (data.deleted && (!Array.isArray(data.deleted) || !data.deleted.every(safePath))) throw new Error('Invalid deleted paths')
  for (const issue of data.issues) {
    if (!safePath(issue.path) || (issue.related && (!Array.isArray(issue.related) || !issue.related.every(safePath)))) {
      throw new Error('Invalid path')
    }
    if (!Number.isSafeInteger(issue.line) || issue.line < 1) throw new Error('Invalid line')
    if (typeof issue.rule !== 'string' || typeof issue.message !== 'string') throw new Error('Invalid diagnostic')
  }
  return data
}

function marker(key: string) {
  return `${ROOT}${createHash('sha256').update(key).digest('hex').slice(0, 24)} -->`
}

function permalink(context: any, head: string, path: string, line: number) {
  const encoded = path.split('/').map(encodeURIComponent).join('/')
  return `${context.serverUrl}/${context.repo.owner}/${context.repo.repo}/blob/${head}/${encoded}#L${line}`
}

function body(context: any, run: any, issues: any[], key: string, links: any = {}) {
  const reference = (path: string, line: number) => {
    const file = (links.files || []).find((file: any) => file.filename === path || file.previous_filename === path)
    const old = (links.deleted || []).includes(path) || file?.status === 'removed' ||
      (file?.previous_filename === path && file.filename !== path)
    const url = old && !links.base
      ? `${context.serverUrl}/${context.repo.owner}/${context.repo.repo}/commit/${run.head_sha}`
      : permalink(context, old ? links.base : run.head_sha, path, line)
    return `[${escape(path)}:${line}${old ? ' (删除前)' : ''}](${url})`
  }
  const lines = issues.map(issue => {
    const related = (issue.related || []).map((path: string) => reference(path, 1))
    return `- ${reference(issue.path, issue.line)} | ${escape(issue.rule)}: ${escape(issue.message)}` +
      (related.length ? `\n  关联代码或决策: ${related.join(', ')}` : '')
  })
  return `${marker(key)}\nAgent Notes: 请核对双向引用或决策与代码的配对\n\n${lines.join('\n')}\n\n[完整诊断与运行日志](${run.html_url})`
}

function patchLines(patch: string = '') {
  let left = 0, right = 0, position = 0, started = false
  const lines: {line: number, side: string, position: number}[] = []
  for (const text of patch.split('\n')) {
    const hunk = /^@@ -(\d+)(?:,\d+)? \+(\d+)(?:,\d+)? @@/.exec(text)
    if (hunk) {
      if (started) position++
      started = true
      left = Number(hunk[1]); right = Number(hunk[2])
      continue
    }
    if (!started) continue
    position++
    if (text.startsWith('+')) lines.push({line: right++, side: 'RIGHT', position})
    else if (text.startsWith('-')) lines.push({line: left++, side: 'LEFT', position})
    else if (text.startsWith(' ')) {
      lines.push({line: right++, side: 'RIGHT', position})
      left++
    }
  }
  return lines
}

function locate(issue: any, files: any[]) {
  const candidates = [issue.path, ...(issue.related || [])]
  const source = (path: string) => /\.(?:cpp|cc|cxx|h|hpp|ts|tsx|js|mjs|go|py|rs)$/.test(path)
  candidates.sort((a, b) => Number(source(b)) - Number(source(a)))
  for (const path of candidates) {
    const file = files.find(file => file.filename === path || file.previous_filename === path)
    if (!file) continue
    const side = file.status === 'removed' || (file.previous_filename === path && file.filename !== path) ? 'LEFT' : 'RIGHT'
    const rows = patchLines(file.patch).filter(row => row.side === side)
    const exact = path === issue.path ? rows.find(row => row.line === issue.line) : undefined
    const row = exact || (path === issue.path
      ? rows.reduce((nearest, row) => Math.abs(row.line - issue.line) < Math.abs(nearest.line - issue.line) ? row : nearest, rows[0])
      : rows[0])
    return {path: file.filename, ...(row || {}), subject_type: row ? 'line' : 'file'}
  }
  return null
}

module.exports = {readReport, marker, body, locate, patchLines, ROOT}
