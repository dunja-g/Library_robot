const STATE_CONFIG = {
  IDLE:        { icon: '●', colour: '#7a8099', desc: 'Waiting for a book request', progress: 0 },
  SCANNING:    { icon: '⌕', colour: '#4f8ef7', desc: 'Scanning for the next ArUco marker', progress: 25 },
  ALIGNING:    { icon: '◎', colour: '#f59e0b', desc: 'Aligning with the correct marker', progress: 45 },
  APPROACHING: { icon: '➜', colour: '#22c55e', desc: 'Approaching the verified waypoint', progress: 65 },
  MOVING:      { icon: '➜', colour: '#22c55e', desc: 'Following the encoder distance target', progress: 65 },
  TURNING:     { icon: '↻', colour: '#a855f7', desc: 'Executing the configured timed turn', progress: 55 },
  ARRIVED:     { icon: '✓', colour: '#16a34a', desc: 'Destination reached; return follows', progress: 100 },
  DWELLING:    { icon: '◷', colour: '#16a34a', desc: 'Waiting at the book destination', progress: 100 },
  DOCKED:      { icon: '⌂', colour: '#38bdf8', desc: 'Return complete — robot is at Dock', progress: 100 },
  STOPPED:     { icon: '!', colour: '#ef4444', desc: 'Safety stop — inspect before resetting', progress: 0 },
};

let currentState = 'IDLE';
let toastTimeout = null;
let navigationMode = 'aruco';

window.addEventListener('DOMContentLoaded', () => { loadDestinations(); pollStatus(); setInterval(pollStatus, 700); });

async function loadDestinations() {
  try {
    const modeData = await fetch('/navigation_mode').then(r => r.json());
    navigationMode = modeData.mode;
    const isGrid = navigationMode === 'grid';
    const destinations = await fetch(isGrid ? '/boxes' : '/books').then(r => r.json());
    const select = document.getElementById('book-select');
    select.innerHTML = `<option value="" disabled selected>Choose ${isGrid ? 'a box' : 'a book'}</option>`;
    destinations.forEach(value => { const option = document.createElement('option'); option.value = value; option.textContent = value; select.appendChild(option); });
    document.getElementById('destination-select-label').textContent = isGrid ? 'Select a Box' : 'Select a Book';
    document.getElementById('destination-summary-label').textContent = isGrid ? 'Box' : 'Book';
    document.getElementById('current-target-label').textContent = isGrid ? 'Current action' : 'Current marker';
    if (isGrid && !modeData.grid_configured) {
      showToast(`Grid calibration pending: ${modeData.missing.join(', ')}`);
    }
  } catch (_) { showToast('Could not load destinations'); }
}

async function sendRobot() {
  const destination = document.getElementById('book-select').value;
  if (!destination) return showToast('Select a destination first');
  const button = document.getElementById('send-btn');
  button.disabled = true;
  try {
    const isGrid = navigationMode === 'grid';
    const response = await fetch(isGrid ? '/request_box' : '/request_book', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(isGrid ? {box_id: destination} : {title: destination}),
    });
    const data = await response.json();
    if (response.ok) {
      const summary = isGrid ? `box ${data.box_id}` : data.route.join(' → ');
      showToast(`Robot dispatched: ${summary}`);
    } else {
      showToast(data.message);
    }
    await pollStatus();
  } catch (_) { showToast('Failed to send the request'); }
}

async function resetRobot() {
  try { await fetch('/reset', {method: 'POST'}); showToast('Robot stopped and mission cleared'); await pollStatus(); }
  catch (_) { showToast('Reset failed'); }
}

async function pollStatus() {
  try {
    const data = await fetch('/status').then(r => r.json());
    updateUI(data);
    const badge = document.getElementById('connection-badge'); badge.textContent = '● Live'; badge.style.color = '';
  } catch (_) {
    const badge = document.getElementById('connection-badge'); badge.textContent = '● Offline'; badge.style.color = '#ef4444';
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
  const overlay = document.getElementById('overlay-state-badge'); overlay.textContent = state; overlay.className = `overlay-badge ${state.toLowerCase()}`;
  document.getElementById('mission-book').textContent = data.box_id || data.book || '—';
  const loc = data.location;
  document.getElementById('mission-location').textContent = data.box_id
    ? `Row ${data.row} / Column ${data.column}`
    : (loc ? `${loc.zone} / ${loc.shelf} / L${loc.level} / S${loc.slot}` : '—');
  document.getElementById('mission-phase').textContent = data.phase || '—';
  document.getElementById('mission-marker').textContent = data.current_action || (data.current_marker_id == null ? '—' : `${data.current_marker_id} · ${data.current_marker_name || ''}`);
  document.getElementById('mission-progress').textContent = data.step_count ? `${data.step_index} / ${data.step_count}` : (data.waypoint_count ? `${data.waypoint_index} / ${data.waypoint_count}` : '—');
  const activeRoute = data.phase === 'RETURNING' ? data.return_route : data.route;
  document.getElementById('mission-route').textContent = data.current_step_label || (activeRoute ? activeRoute.join(' → ') : '—');
  document.getElementById('send-btn').disabled = !['IDLE', 'DOCKED', 'STOPPED'].includes(state);
  if (state !== previous && state === 'ARRIVED') { showToast('Book destination reached'); speak('Book destination reached. The robot will now return to the dock.'); }
  if (state !== previous && state === 'DOCKED') { showToast('Robot returned to Dock'); speak('Return complete. The robot is back at the dock.'); }
  if (state !== previous && state === 'STOPPED') showToast(`Safety stop: ${data.reason || 'unknown reason'}`);
}

function speak(message) {
  if ('speechSynthesis' in window) { window.speechSynthesis.cancel(); window.speechSynthesis.speak(new SpeechSynthesisUtterance(message)); }
}

function showToast(message) {
  const toast = document.getElementById('toast'); toast.textContent = message; toast.classList.add('show');
  clearTimeout(toastTimeout); toastTimeout = setTimeout(() => toast.classList.remove('show'), 3500);
}
