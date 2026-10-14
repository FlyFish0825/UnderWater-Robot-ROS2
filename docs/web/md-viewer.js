/* md-viewer.js —— 纯前端 Markdown 渲染器（零依赖、可离线 file:// 运行）
 * 读取 window.__MD_DOCS__（由 tools/generate-web-md-data.mjs 生成），
 * 依据 URL 的 ?doc=<仓库相对路径> 渲染，并把文档内的 .md 链接改写为本站查看器链接。
 */
(function () {
  'use strict';

  var DOCS = window.__MD_DOCS__ || {};
  var currentDir = '';

  function escapeHtml(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  }
  function attr(s) { return String(s).replace(/"/g, '&quot;'); }

  function resolvePath(baseDir, p) {
    var parts = (baseDir ? baseDir.split('/') : []).concat(String(p).split('/'));
    var out = [];
    for (var i = 0; i < parts.length; i++) {
      var seg = parts[i];
      if (seg === '' || seg === '.') continue;
      if (seg === '..') { out.pop(); continue; }
      out.push(seg);
    }
    return out.join('/');
  }

  function isExternal(u) { return /^(https?:|mailto:|tel:|data:|#)/i.test(u); }

  function assetUrl(src) {
    if (isExternal(src)) return src;
    return '../../' + resolvePath(currentDir, src);
  }

  function linkUrl(href) {
    if (isExternal(href)) return href;
    var hi = href.indexOf('#');
    var pathPart = hi >= 0 ? href.slice(0, hi) : href;
    var frag = hi >= 0 ? href.slice(hi) : '';
    if (pathPart === '') return href;
    var resolved = resolvePath(currentDir, pathPart);
    if (/\.md$/i.test(resolved)) {
      return 'viewer.html?doc=' + encodeURIComponent(resolved).replace(/%2F/g, '/') + frag;
    }
    if (/^[a-z][a-z0-9+.-]*:/i.test(pathPart)) return href;
    return '../../' + resolved + frag;
  }

  function slugify(s) {
    return String(s)
      .replace(/<[^>]+>/g, '')
      .toLowerCase()
      .replace(/[^\w\u4e00-\u9fff\- ]/g, '')
      .trim()
      .replace(/\s+/g, '-');
  }
  var slugCount = {};
  function uniqueSlug(base) {
    var s = base || 'section';
    if (Object.prototype.hasOwnProperty.call(slugCount, s)) {
      slugCount[s] += 1;
      s = s + '-' + slugCount[s];
    } else {
      slugCount[s] = 0;
    }
    return s;
  }

  function inline(text) {
    var codes = [];
    text = String(text).replace(/`([^`]+)`/g, function (m, c) {
      codes.push(c);
      return '\u0000' + (codes.length - 1) + '\u0000';
    });
    text = escapeHtml(text);
    text = text.replace(/!\[([^\]]*)\]\(([^)\s]+)(?:\s+"[^"]*")?\)/g, function (m, alt, src) {
      return '<img src="' + attr(assetUrl(src)) + '" alt="' + attr(alt) + '" loading="lazy">';
    });
    text = text.replace(/\[([^\]]+)\]\(([^)\s]+)(?:\s+"[^"]*")?\)/g, function (m, label, href) {
      var ext = /^https?:/i.test(href) ? ' target="_blank" rel="noopener"' : '';
      return '<a href="' + attr(linkUrl(href)) + '"' + ext + '>' + label + '</a>';
    });
    text = text.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
    text = text.replace(/__([^_]+)__/g, '<strong>$1</strong>');
    text = text.replace(/\*([^*\n]+)\*/g, '<em>$1</em>');
    text = text.replace(/~~([^~]+)~~/g, '<del>$1</del>');
    text = text.replace(/\u0000(\d+)\u0000/g, function (m, n) {
      return '<code>' + escapeHtml(codes[+n]) + '</code>';
    });
    return text;
  }

  function splitRow(row) {
    var r = String(row).trim();
    if (r.charAt(0) === '|') r = r.slice(1);
    if (r.charAt(r.length - 1) === '|') r = r.slice(0, -1);
    return r.split('|').map(function (c) { return c.trim(); });
  }
  function isTableSep(s) {
    var t = String(s).trim();
    if (t.indexOf('|') < 0 || t.indexOf('-') < 0) return false;
    return /^\|?\s*:?-+:?\s*(\|\s*:?-+:?\s*)*\|?$/.test(t);
  }
  function parseTable(lines, i) {
    var headers = splitRow(lines[i]);
    var aligns = splitRow(lines[i + 1]).map(function (s) {
      var l = s.charAt(0) === ':';
      var r = s.charAt(s.length - 1) === ':';
      return l && r ? 'center' : r ? 'right' : l ? 'left' : '';
    });
    var j = i + 2, rows = [];
    while (j < lines.length && lines[j].trim() !== '' && lines[j].indexOf('|') >= 0) {
      rows.push(splitRow(lines[j]));
      j++;
    }
    var html = '<div class="table-container"><table><thead><tr>';
    headers.forEach(function (h, k) {
      html += '<th' + (aligns[k] ? ' style="text-align:' + aligns[k] + '"' : '') + '>' + inline(h) + '</th>';
    });
    html += '</tr></thead><tbody>';
    for (var r = 0; r < rows.length; r++) {
      html += '<tr>';
      headers.forEach(function (_, k) {
        html += '<td' + (aligns[k] ? ' style="text-align:' + aligns[k] + '"' : '') + '>' + inline(rows[r][k] || '') + '</td>';
      });
      html += '</tr>';
    }
    html += '</tbody></table></div>';
    return { html: html, next: j };
  }

  function collectList(lines, i) {
    var entries = [];
    while (i < lines.length) {
      var line = lines[i];
      if (/^\s*$/.test(line)) {
        var nxt = lines[i + 1] || '';
        if (/^\s*([-*+]|\d+\.)\s+/.test(nxt)) { i++; continue; }
        break;
      }
      var m = line.match(/^(\s*)([-*+]|\d+\.)\s+(.*)$/);
      if (m) {
        entries.push({ indent: m[1].length, ordered: /^\d/.test(m[2]), text: m[3] });
        i++;
      } else if (entries.length && /^\s+\S/.test(line)) {
        entries[entries.length - 1].text += ' ' + line.trim();
        i++;
      } else {
        break;
      }
    }
    return { entries: entries, next: i };
  }
  function buildList(entries) {
    var idx = 0;
    function parse(indent) {
      var ordered = entries[idx].ordered;
      var html = ordered ? '<ol>' : '<ul>';
      while (idx < entries.length && entries[idx].indent >= indent) {
        if (entries[idx].indent > indent) { html += parse(entries[idx].indent); continue; }
        var e = entries[idx]; idx++;
        var item = '<li>' + inline(e.text);
        if (idx < entries.length && entries[idx].indent > indent) item += parse(entries[idx].indent);
        html += item + '</li>';
      }
      return html + (ordered ? '</ol>' : '</ul>');
    }
    return parse(entries[0].indent);
  }

  var HR = /^\s*[-*_](\s*[-*_]){2,}\s*$/;
  function startsBlock(lines, i) {
    var line = lines[i];
    if (/^\s*$/.test(line)) return true;
    if (/^\s*`{3,}/.test(line)) return true;
    if (/^#{1,6}\s+/.test(line)) return true;
    if (/^\s*>/.test(line)) return true;
    if (/^\s*([-*+]|\d+\.)\s+/.test(line)) return true;
    if (HR.test(line)) return true;
    if (line.indexOf('|') >= 0 && i + 1 < lines.length && isTableSep(lines[i + 1])) return true;
    return false;
  }

  function render(md) {
    md = String(md).replace(/\r\n?/g, '\n');
    var lines = md.split('\n');
    var out = [];
    var i = 0;
    while (i < lines.length) {
      var line = lines[i];
      if (/^\s*$/.test(line)) { i++; continue; }

      if (/^\s*<!--/.test(line)) {
        while (i < lines.length && lines[i].indexOf('-->') < 0) i++;
        i++;
        continue;
      }

      var m = line.match(/^\s*`{3,}(\S*)\s*$/);
      if (m) {
        var lang = m[1];
        i++;
        var buf = [];
        while (i < lines.length && !/^\s*`{3,}\s*$/.test(lines[i])) { buf.push(lines[i]); i++; }
        i++;
        out.push('<pre><code' + (lang ? ' class="language-' + attr(lang) + '"' : '') + '>' + escapeHtml(buf.join('\n')) + '</code></pre>');
        continue;
      }

      m = line.match(/^(#{1,6})\s+(.*?)\s*#*\s*$/);
      if (m) {
        var lvl = m[1].length;
        var txt = m[2];
        var id = uniqueSlug(slugify(txt));
        out.push('<h' + lvl + ' id="' + attr(id) + '">' + inline(txt) + '</h' + lvl + '>');
        i++;
        continue;
      }

      if (HR.test(line)) { out.push('<hr>'); i++; continue; }

      if (line.indexOf('|') >= 0 && i + 1 < lines.length && isTableSep(lines[i + 1])) {
        var t = parseTable(lines, i);
        out.push(t.html);
        i = t.next;
        continue;
      }

      if (/^\s*>/.test(line)) {
        var q = [];
        while (i < lines.length && /^\s*>/.test(lines[i])) {
          q.push(lines[i].replace(/^\s*>\s?/, ''));
          i++;
        }
        out.push('<blockquote>' + render(q.join('\n')) + '</blockquote>');
        continue;
      }

      if (/^\s*([-*+]|\d+\.)\s+/.test(line)) {
        var c = collectList(lines, i);
        out.push(buildList(c.entries));
        i = c.next;
        continue;
      }

      var p = [];
      while (i < lines.length && !startsBlock(lines, i)) { p.push(lines[i]); i++; }
      if (p.length === 0) { p.push(lines[i]); i++; }
      out.push('<p>' + inline(p.join(' ')) + '</p>');
    }
    return out.join('\n');
  }

  function buildToc(content) {
    var toc = document.getElementById('tocPanel');
    if (!toc) return;
    var heads = content.querySelectorAll('h2, h3');
    if (!heads.length) {
      toc.innerHTML = '<div class="toc-title">本页内容</div><div class="toc-empty">无小节标题</div>';
      return;
    }
    var html = '<div class="toc-title">本页内容</div>';
    for (var i = 0; i < heads.length; i++) {
      var h = heads[i];
      html += '<a href="#' + attr(h.id) + '" class="toc-link' + (h.tagName === 'H3' ? ' toc-sub' : '') + '">'
        + escapeHtml(h.textContent) + '</a>';
    }
    toc.innerHTML = html;
  }

  function boot() {
    var content = document.getElementById('md-content');
    if (!content) return;

    var params = new URLSearchParams(window.location.search);
    var doc = params.get('doc') || 'README.md';
    doc = doc.replace(/^\.?\//, '');

    var text = DOCS[doc];
    if (typeof text !== 'string') {
      content.innerHTML =
        '<div class="callout danger"><div class="callout-icon">❌</div><div class="callout-content">' +
        '<div class="callout-title">找不到文档</div><p>内置文档数据中没有 <code>' + escapeHtml(doc) +
        '</code>。请确认路径为仓库相对路径，并重新运行 <code>node tools/generate-web-md-data.mjs</code>。</p>' +
        '</div></div>';
      buildToc(content);
      return;
    }

    currentDir = doc.indexOf('/') >= 0 ? doc.slice(0, doc.lastIndexOf('/')) : '';
    content.innerHTML = render(text);

    var h1 = content.querySelector('h1');
    if (h1 && h1.textContent.trim()) {
      document.title = h1.textContent.trim() + ' | UnderWater-Robot-ROS2 文档';
    }
    var pathTag = document.getElementById('mdPath');
    if (pathTag) pathTag.textContent = doc;

    buildToc(content);

    if (window.location.hash) {
      var target = document.getElementById(decodeURIComponent(window.location.hash).slice(1));
      if (target) target.scrollIntoView();
    }
  }

  /* 供离线测试与二次开发使用 */
  window.__MD_RENDER__ = render;
  window.__MD_INTERNALS__ = { render: render, resolvePath: resolvePath, linkUrl: linkUrl, setDir: function (d) { currentDir = d; } };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }
})();
