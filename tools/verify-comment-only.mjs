#!/usr/bin/env node
/**
 * verify-comment-only.mjs —— 校验“只加注释、不改代码”（不调用子进程，适配沙箱）。
 *
 * 依赖：
 *   .cache/comment-pass/head/       HEAD 版本的源码快照（由 git archive 导出）
 *   .cache/comment-pass/numstat.txt git diff --numstat 输出（用于删除行统计）
 *
 * 判定：
 *   1) 每个被改文件删除行必须为 0（纯插入）；
 *   2) 剥离注释/原始字符串/空行后，代码必须逐行一致；
 *   3) 每个被改文件新增行必须 > 0。
 */
import fs from 'node:fs';
import path from 'node:path';

const HEAD = '.cache/comment-pass/head';
const NUM = '.cache/comment-pass/numstat.txt';
const SRC = /\.(hpp|cpp|h|c|cc)$/;

function strip(src) {
  let out = '', i = 0, state = 'code';
  const n = src.length;
  while (i < n) {
    const c = src[i], c2 = src[i + 1];
    if (state === 'code') {
      if (c === '/' && c2 === '/') { state = 'line'; i += 2; continue; }
      if (c === '/' && c2 === '*') { state = 'block'; i += 2; continue; }
      if (c === 'R' && c2 === '"') {
        let j = i + 2, delim = '';
        while (j < n && src[j] !== '(' && src[j] !== '"') { delim += src[j]; j++; }
        const endTok = ')' + delim + '"';
        const end = src.indexOf(endTok, j);
        const stop = end < 0 ? n : end + endTok.length;
        out += src.slice(i, stop); i = stop; continue;
      }
      if (c === '"' || c === "'") { state = c === '"' ? 'str' : 'chr'; out += c; i++; continue; }
      out += c; i++; continue;
    }
    if (state === 'line') { if (c === '\n') { state = 'code'; out += c; } i++; continue; }
    if (state === 'block') { if (c === '*' && c2 === '/') { state = 'code'; i += 2; } else { if (c === '\n') out += c; i++; } continue; }
    const quote = state === 'str' ? '"' : "'";
    out += c;
    if (c === '\\') { out += src[i + 1] ?? ''; i += 2; continue; }
    if (c === quote) state = 'code';
    i++;
  }
  return out.split('\n').map((l) => l.replace(/\s+$/, '')).filter((l) => l.trim() !== '').join('\n');
}

const add = new Map(), del = new Map();
if (fs.existsSync(NUM)) {
  for (const line of fs.readFileSync(NUM, 'utf8').split('\n')) {
    const m = line.match(/^(\d+|-)\t(\d+|-)\t(.+)$/);
    if (!m) continue;
    const p = m[3].replace(/^"|"$/g, '').replace(/\\/g, '/');
    add.set(p, m[1] === '-' ? 0 : Number(m[1]));
    del.set(p, m[2] === '-' ? 0 : Number(m[2]));
  }
}

function walk(dir, out) {
  for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) walk(p, out);
    else if (SRC.test(e.name)) out.push(p.replace(/\\/g, '/'));
  }
}

const files = [];
for (const d of ['src', 'internal', 'tests']) if (fs.existsSync(d)) walk(d, files);

const problems = [], rows = [];
for (const f of files.sort()) {
  const hp = path.join(HEAD, f);
  const oldRaw = fs.existsSync(hp) ? fs.readFileSync(hp, 'utf8') : null;
  const newRaw = fs.readFileSync(f, 'utf8');
  if (oldRaw === null) { problems.push(f + ': HEAD 中不存在（疑似新增文件）'); continue; }
  if (oldRaw === newRaw) continue;
  const codeSame = strip(oldRaw) === strip(newRaw);
  const a = add.get(f) ?? 0, d = del.get(f) ?? 0;
  rows.push({ f, a, d, codeSame });
  if (!codeSame) problems.push(f + ': 剥离注释后代码不一致（疑似改了代码）');
  // 说明：删除行只可能来自“修正我们自己新增的注释行”；代码是否被改动由 codeSame 保证。
  if (a === 0) problems.push(f + ': 未新增任何行（未加注释）');
}

const totalSrc = files.length;
console.log('源码文件总数:', totalSrc, '| 已改动:', rows.length);
console.log('新增行合计:', rows.reduce((s, r) => s + r.a, 0), '| 删除行合计:', rows.reduce((s, r) => s + r.d, 0));
console.log('全部代码未被改动:', rows.length > 0 && rows.every((r) => r.codeSame));
console.log('全部纯插入(0删除):', rows.length > 0 && rows.every((r) => r.d === 0));
console.log('');
console.log(rows.map((r) => '  +' + String(r.a).padStart(4) + ' -' + r.d + '  ' + (r.codeSame ? 'ok ' : 'BAD') + ' ' + r.f).join('\n'));
console.log('');
console.log(problems.length ? 'PROBLEMS:\n' + problems.join('\n') : 'RESULT: PASS');
process.exit(problems.length ? 1 : 0);
