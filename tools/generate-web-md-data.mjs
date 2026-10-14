#!/usr/bin/env node
/**
 * 生成 docs/web/md-data.js —— 把仓库内所有 Markdown 快照进网页文档站点。
 *
 * 背景：浏览器在 file:// 下无法 fetch 本地 .md（CORS 限制），
 * 因此把 Markdown 文本内联成 JS 数据，配合 md-viewer.js 纯前端解析渲染。
 *
 * 重新生成：node tools/generate-web-md-data.mjs
 */
import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const EXCLUDE_DIRS = new Set([
  '.git', '.cache', '.vscode', 'node_modules', '__pycache__',
  'third_party', 'artifacts', 'build', 'install', 'log', 'output', 'deploy',
]);

const files = [];
function walk(dir, rel) {
  let entries;
  try { entries = fs.readdirSync(dir, { withFileTypes: true }); } catch { return; }
  for (const e of entries) {
    if (e.isDirectory()) {
      if (EXCLUDE_DIRS.has(e.name) || e.name.startsWith('.')) continue;
      walk(path.join(dir, e.name), rel ? rel + '/' + e.name : e.name);
    } else if (e.isFile() && e.name.toLowerCase().endsWith('.md')) {
      files.push(rel ? rel + '/' + e.name : e.name);
    }
  }
}
walk(root, '');
files.sort();

const data = {};
let total = 0;
for (const f of files) {
  const text = fs.readFileSync(path.join(root, f), 'utf8');
  data[f] = text;
  total += text.length;
}

const json = JSON.stringify(data)
  .replace(/<\//g, '<\\/')          // 防止 </script> 提前闭合脚本标签
  .replace(/\u2028/g, '\\u2028')
  .replace(/\u2029/g, '\\u2029');

const banner = '/* 由 tools/generate-web-md-data.mjs 自动生成，请勿手工编辑。 */\n';
const outPath = path.join(root, 'docs/web/md-data.js');
fs.writeFileSync(outPath, banner + 'window.__MD_DOCS__ = ' + json + ';\n', 'utf8');

console.log('generated ' + files.length + ' markdown docs, ' + total + ' chars -> docs/web/md-data.js (' + fs.statSync(outPath).size + ' bytes)');
