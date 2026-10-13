document.addEventListener('DOMContentLoaded', () => {
  // 1. 主题切换
  const themeToggle = document.getElementById('themeToggle');
  const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
  const savedTheme = localStorage.getItem('theme') || (prefersDark ? 'dark' : 'light');
  
  document.documentElement.setAttribute('data-theme', savedTheme);
  updateThemeIcon(savedTheme);

  if (themeToggle) {
    themeToggle.addEventListener('click', () => {
      const current = document.documentElement.getAttribute('data-theme');
      const next = current === 'dark' ? 'light' : 'dark';
      document.documentElement.setAttribute('data-theme', next);
      localStorage.setItem('theme', next);
      updateThemeIcon(next);
    });
  }

  function updateThemeIcon(theme) {
    if (!themeToggle) return;
    themeToggle.innerHTML = theme === 'dark' ? '☀️' : '🌙';
    themeToggle.setAttribute('title', theme === 'dark' ? '切换为亮色模式' : '切换为暗色模式');
  }

  // 2. 移动端侧边栏切换
  const menuToggle = document.getElementById('menuToggle');
  const sidebar = document.getElementById('sidebar');

  if (menuToggle && sidebar) {
    menuToggle.addEventListener('click', () => {
      sidebar.classList.toggle('open');
    });

    document.addEventListener('click', (e) => {
      if (window.innerWidth <= 860 && 
          !sidebar.contains(e.target) && 
          !menuToggle.contains(e.target) && 
          sidebar.classList.contains('open')) {
        sidebar.classList.remove('open');
      }
    });
  }

  // 3. 多页面侧边栏高亮联动
  const currentPath = window.location.pathname.split('/').pop() || 'index.html';
  const navLinks = document.querySelectorAll('.nav-item');
  navLinks.forEach(link => {
    const href = link.getAttribute('href');
    if (href && (href === currentPath || href.startsWith(currentPath + '#') || (currentPath === '' && href === 'index.html'))) {
      link.classList.add('active');
    } else {
      link.classList.remove('active');
    }
  });

  // 4. 代码复制功能
  const copyButtons = document.querySelectorAll('.copy-btn');
  copyButtons.forEach(btn => {
    btn.addEventListener('click', async () => {
      const codeWrapper = btn.closest('.code-wrapper');
      const codeElem = codeWrapper ? codeWrapper.querySelector('code') : null;
      if (!codeElem) return;
      const code = codeElem.innerText;
      try {
        await navigator.clipboard.writeText(code);
        const originalText = btn.innerText;
        btn.innerText = '已复制!';
        btn.classList.add('copied');
        setTimeout(() => {
          btn.innerText = originalText;
          btn.classList.remove('copied');
        }, 2000);
      } catch (err) {
        console.error('复制失败', err);
      }
    });
  });

  // 5. 右侧目录 (TOC) 滚动高亮
  const sections = document.querySelectorAll('section.doc-section');
  const tocLinks = document.querySelectorAll('.toc-link');

  if (sections.length > 0 && tocLinks.length > 0) {
    const observerOptions = {
      root: null,
      rootMargin: '-80px 0px -70% 0px',
      threshold: 0
    };

    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          const id = entry.target.getAttribute('id');
          tocLinks.forEach(link => {
            const href = link.getAttribute('href');
            if (href === `#${id}` || href.endsWith(`#${id}`)) {
              link.classList.add('active');
            } else {
              link.classList.remove('active');
            }
          });
        }
      });
    }, observerOptions);

    sections.forEach(sec => observer.observe(sec));
  }

  // 6. 页面内关键词实时搜索过滤
  const searchInput = document.getElementById('searchInput');
  if (searchInput) {
    searchInput.addEventListener('input', (e) => {
      const query = e.target.value.trim().toLowerCase();
      if (!query) {
        sections.forEach(sec => sec.style.display = 'block');
        return;
      }

      sections.forEach(sec => {
        const text = sec.innerText.toLowerCase();
        if (text.includes(query)) {
          sec.style.display = 'block';
        } else {
          sec.style.display = 'none';
        }
      });
    });
  }

  // 7. 推进器 RPM 模拟换算器初始化（若当前页面包含该组件）
  initThrusterSimulator();
});

function initThrusterSimulator() {
  const fxSlider = document.getElementById('simFx');
  const fzSlider = document.getElementById('simFz');
  const yawSlider = document.getElementById('simYaw');
  
  if (!fxSlider || !fzSlider || !yawSlider) return;

  function updateSimulation() {
    const fx = parseFloat(fxSlider.value);
    const fz = parseFloat(fzSlider.value);
    const yaw = parseFloat(yawSlider.value);

    const fxVal = document.getElementById('simFxVal');
    const fzVal = document.getElementById('simFzVal');
    const yawVal = document.getElementById('simYawVal');

    if (fxVal) fxVal.innerText = fx.toFixed(1) + ' N';
    if (fzVal) fzVal.innerText = fz.toFixed(1) + ' N';
    if (yawVal) yawVal.innerText = yaw.toFixed(1) + ' N·m';

    const nominalMaxThrust = 500.0;
    const maxRpm = 6000.0;
    const rpmPerN = maxRpm / nominalMaxThrust;

    const f1 = (-0.355 * fx + 0.5 * yaw);
    const f2 = (-0.355 * fx - 0.5 * yaw);
    const f3 = (0.352 * fx + 0.5 * yaw);
    const f4 = (0.352 * fx - 0.5 * yaw);
    const f5 = (0.25 * fz);
    const f6 = (0.25 * fz);
    const f7 = (-0.25 * fz);
    const f8 = (-0.25 * fz);

    const forces = [f1, f2, f3, f4, f5, f6, f7, f8];
    const motorDirections = [1, -1, -1, 1, 1, 1, -1, -1];

    for (let i = 0; i < 8; i++) {
      const rawRpm = forces[i] * rpmPerN;
      let finalRpm = 0;
      let status = '停机(死区)';

      if (Math.abs(rawRpm) >= 1000) {
        let clamped = Math.min(Math.max(Math.abs(rawRpm), 1000), 6000);
        finalRpm = Math.sign(rawRpm) * clamped * motorDirections[i];
        status = `${finalRpm > 0 ? '正转' : '反转'} (${Math.round(finalRpm)} RPM)`;
      }

      const valElem = document.getElementById(`rpm_fan${i + 1}`);
      const barElem = document.getElementById(`bar_fan${i + 1}`);
      if (valElem) {
        valElem.innerText = status;
        valElem.style.color = Math.abs(finalRpm) > 0 ? '#38bdf8' : '#6b7280';
      }
      if (barElem) {
        const percent = Math.min(Math.abs(finalRpm) / 6000 * 100, 100);
        barElem.style.width = `${percent}%`;
        barElem.style.background = finalRpm > 0 ? '#10b981' : (finalRpm < 0 ? '#38bdf8' : '#374151');
      }
    }
  }

  fxSlider.addEventListener('input', updateSimulation);
  fzSlider.addEventListener('input', updateSimulation);
  yawSlider.addEventListener('input', updateSimulation);
  updateSimulation();
}
