const STATE_CONFIG = {
  IDLE:     { icon: '●', colour: '#7a8099', desc: 'Enter a book to begin', progress: 0 },
  MOVING:   { icon: '➜', colour: '#22c55e', desc: 'Following encoder distance target', progress: 55 },
  TURNING:  { icon: '↻', colour: '#a855f7', desc: 'Turning with the MPU6500', progress: 70 },
  ARRIVED:  { icon: '✓', colour: '#16a34a', desc: 'Book location reached; return follows', progress: 100 },
  DWELLING: { icon: '◷', colour: '#16a34a', desc: 'Waiting at the book location', progress: 100 },
  DOCKED:   { icon: '⌂', colour: '#38bdf8', desc: 'Return complete — robot is at Dock', progress: 100 },
  STOPPED:  { icon: '!', colour: '#ef4444', desc: 'Safety stop — inspect before resetting', progress: 0 },
};

let currentState = 'IDLE';
let toastTimeout = null;
let searchTimer = null;

window.addEventListener('DOMContentLoaded', () => {
  loadCatalogue();
  pollStatus();
  setInterval(pollStatus, 700);
  const input = document.getElementById('book-search');
  input.addEventListener('input', () => {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(() => searchCatalogue(input.value), 180);
  });
  input.addEventListener('keydown', event => {
    if (event.key === 'Enter') sendRobot();
  });
});

async function loadCatalogue() {
  try {
    const mode = await fetch('/navigation_mode').then(r => r.json());
    if (mode.marker_scanning) throw new Error('Unexpected marker mode');
    await searchCatalogue('');
    if (!mode.grid_configured) {
      showToast(`Grid dimensions pending: ${mode.missing.join(', ')}`);
    }
  } catch (_) {
    showToast('Could not load the fixed-grid catalogue');
  }
}

async function searchCatalogue(query) {
  try {
    const results = await fetch(`/search_books?q=${encodeURIComponent(query)}`).then(r => r.json());
    const options = document.getElementById('book-options');
    options.innerHTML = '';
    results.forEach(book => {
      const option = document.createElement('option');
      option.value = book.title;
      option.label = `${book.book_id} · ${book.location_code}`;
      options.appendChild(option);
    });
    const result = document.getElementById('search-result');
    if (!query) {
      result.textContent = `${results.length} numbered book${results.length === 1 ? '' : 's'} available`;
    } else if (results.length === 1) {
      const book = results[0];
      result.textContent = `${book.title} · ${book.book_id} · ${book.location_code}`;
    } else {
      result.textContent = results.length
        ? `${results.length} matches — choose a more specific book`
        : 'No numbered book found';
    }
  } catch (_) {
    document.getElementById('search-result').textContent = 'Search unavailable';
  }
}

async function sendRobot() {
  const query = document.getElementById('book-search').value.trim();
  if (!query) return showToast('Enter a book title or number');
  const button = document.getElementById('send-btn');
  button.disabled = true;
  let started = false;
  try {
    const response = await fetch('/request_book', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({query}),
    });
    const data = await response.json();
    if (!response.ok) {
      showToast(data.message);
      return;
    }
    document.getElementById('book-search').value = data.title;
    document.getElementById('search-result').textContent =
      `${data.title} · ${data.book_code} · ${data.location_code}`;
    started = true;
    showToast(`Route started: ${data.location_code}`);
    await pollStatus();
  } catch (_) {
    showToast('Failed to start the route');
  } finally {
    if (!started) button.disabled = false;
  }
}

async function resetRobot() {
  try {
    await fetch('/reset', {method: 'POST'});
    showToast('Robot stopped and mission cleared');
    await pollStatus();
  } catch (_) {
    showToast('Reset failed');
  }
}

async function pollStatus() {
  try {
    const data = await fetch('/status').then(r => r.json());
    updateUI(data);
    const badge = document.getElementById('connection-badge');
    badge.textContent = '● Fixed Grid';
    badge.style.color = '';
  } catch (_) {
    const badge = document.getElementById('connection-badge');
    badge.textContent = '● Offline';
    badge.style.color = '#ef4444';
  }
}

function updateUI(data) {
  const state = data.state || 'IDLE';
  const previous = currentState;
  currentState = state;
  const cfg = STATE_CONFIG[state] || STATE_CONFIG.IDLE;
  document.getElementById('status-icon').textContent = cfg.icon;
  document.getElementById('status-state').textContent = state;
  document.getElementById('status-state').style.color = cfg.colour;
  document.getElementById('status-desc').textContent = cfg.desc;
  document.getElementById('progress-fill').style.width = cfg.progress + '%';
  document.getElementById('reason-text').textContent = data.reason ? `Reason: ${data.reason}` : '';
  const overlay = document.getElementById('overlay-state-badge');
  overlay.textContent = state;
  overlay.className = `overlay-badge ${state.toLowerCase()}`;
  document.getElementById('mission-book').textContent = data.book
    ? `${data.book} · ${data.location_code}`
    : '—';
  document.getElementById('mission-location').textContent = data.box_id
    ? `Box ${data.box_id} / Layer ${data.layer} / Position ${data.position}`
    : '—';
  document.getElementById('mission-phase').textContent = data.phase || '—';
  document.getElementById('mission-marker').textContent = data.current_action || '—';
  document.getElementById('mission-progress').textContent =
    data.step_count ? `${data.step_index} / ${data.step_count}` : '—';
  document.getElementById('mission-route').textContent = data.current_step_label || '—';
  document.getElementById('send-btn').disabled =
    !['IDLE', 'DOCKED', 'STOPPED'].includes(state);
  if (state !== previous && state === 'ARRIVED') {
    showToast(`Book location reached: ${data.location_code}`);
    speak(`Book location reached. Box ${data.box_id}, layer ${data.layer}, position ${data.position}.`);
  }
  if (state !== previous && state === 'DOCKED') {
    showToast('Robot returned to Dock');
    speak('Return complete. The robot is back at the dock.');
  }
  if (state !== previous && state === 'STOPPED') {
    showToast(`Safety stop: ${data.reason || 'unknown reason'}`);
  }
}

function speak(message) {
  if ('speechSynthesis' in window) {
    window.speechSynthesis.cancel();
    window.speechSynthesis.speak(new SpeechSynthesisUtterance(message));
  }
}

function showToast(message) {
  const toast = document.getElementById('toast');
  toast.textContent = message;
  toast.classList.add('show');
  clearTimeout(toastTimeout);
  toastTimeout = setTimeout(() => toast.classList.remove('show'), 3500);
}
