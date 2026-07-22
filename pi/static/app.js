/* app.js — Library Robot UI Logic */

// ─── State Configuration ───────────────────────────────────────────────────
const STATE_CONFIG = {
  IDLE:        { icon: '🔵', colour: '#7a8099', desc: 'Waiting for a book request',          progress: 0   },
  SCANNING:    { icon: '🔄', colour: '#4f8ef7', desc: 'Rotating to scan for the book…',      progress: 25  },
  ALIGNING:    { icon: '🎯', colour: '#f59e0b', desc: 'Marker found — aligning robot…',      progress: 50  },
  APPROACHING: { icon: '🚗', colour: '#22c55e', desc: 'Driving toward the book…',            progress: 75  },
  ARRIVED:     { icon: '✅', colour: '#16a34a', desc: 'Arrived! Your book is right ahead.',  progress: 100 },
  STOPPED:     { icon: '🛑', colour: '#ef4444', desc: 'Obstacle detected — robot stopped.',  progress: 75  },
};

let currentState    = 'IDLE';
let pollingInterval = null;

// ─── Init ──────────────────────────────────────────────────────────────────
window.addEventListener('DOMContentLoaded', () => {
  loadBooks();
  startPolling();
});

// ─── Load books into dropdown ──────────────────────────────────────────────
function loadBooks() {
  fetch('/books')
    .then(r => r.json())
    .then(books => {
      const select = document.getElementById('book-select');
      select.innerHTML = '<option value="" disabled selected>— Choose a book —</option>';
      books.forEach(title => {
        const opt = document.createElement('option');
        opt.value = title;
        opt.textContent = title;
        select.appendChild(opt);
      });
    })
    .catch(() => showToast('⚠️ Could not load book list'));
}

// ─── Send robot ────────────────────────────────────────────────────────────
function sendRobot() {
  const select = document.getElementById('book-select');
  const title  = select.value;

  if (!title) {
    showToast('📖 Please select a book first');
    return;
  }

  const btn = document.getElementById('send-btn');
  btn.disabled = true;
  btn.innerHTML = '<span class="btn-icon">⏳</span> Sending…';

  fetch('/request_book', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ title }),
  })
    .then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        showToast(`🚀 Robot dispatched to: ${title}`);
      } else {
        showToast(`❌ Error: ${data.message}`);
      }
    })
    .catch(() => showToast('❌ Failed to send request'))
    .finally(() => {
      btn.disabled = false;
      btn.innerHTML = '<span class="btn-icon">🚀</span> Send Robot';
    });
}

// ─── Reset robot ───────────────────────────────────────────────────────────
function resetRobot() {
  fetch('/reset', { method: 'POST' })
    .then(r => r.json())
    .then(() => showToast('↺ Robot reset to dock'))
    .catch(() => showToast('❌ Reset failed'));
}

// ─── Status polling ────────────────────────────────────────────────────────
function startPolling() {
  pollingInterval = setInterval(pollStatus, 900);
  pollStatus(); // immediate first call
}

function pollStatus() {
  fetch('/status')
    .then(r => r.json())
    .then(data => updateUI(data.state))
    .catch(() => {
      // Server unreachable — grey out the badge
      document.getElementById('connection-badge').textContent = '● Offline';
      document.getElementById('connection-badge').style.color = '#ef4444';
    });
}

// ─── Update all UI elements to reflect state ───────────────────────────────
function updateUI(state) {
  if (state === currentState) return; // no change
  currentState = state;

  const cfg = STATE_CONFIG[state] || STATE_CONFIG.IDLE;

  // Status display
  document.getElementById('status-icon').textContent  = cfg.icon;
  document.getElementById('status-state').textContent = state;
  document.getElementById('status-state').style.color = cfg.colour;
  document.getElementById('status-desc').textContent  = cfg.desc;

  // Progress bar
  document.getElementById('progress-fill').style.width = cfg.progress + '%';

  // Camera overlay badge
  const badge = document.getElementById('overlay-state-badge');
  badge.textContent  = state;
  badge.className    = `overlay-badge ${state.toLowerCase()}`;

  // Disable send button while robot is active
  const sendBtn = document.getElementById('send-btn');
  const isActive = !['IDLE', 'ARRIVED', 'STOPPED'].includes(state);
  sendBtn.disabled = isActive;

  // Connection badge
  document.getElementById('connection-badge').textContent = '● Live';
  document.getElementById('connection-badge').style.color = '';

  // Show toast on arrival
  if (state === 'ARRIVED') showToast('✅ Robot has arrived at the book!');
  if (state === 'STOPPED') showToast('🛑 Obstacle detected — robot stopped');
}

// ─── Toast notification ────────────────────────────────────────────────────
let toastTimeout = null;
function showToast(message) {
  const toast = document.getElementById('toast');
  toast.textContent = message;
  toast.classList.add('show');

  clearTimeout(toastTimeout);
  toastTimeout = setTimeout(() => toast.classList.remove('show'), 3000);
}
