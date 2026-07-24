const STATE_CONFIG = {
  IDLE: { icon: '●', label: 'Idle', colour: '#64748b', desc: 'Enter a book to begin' },
  MOVING: { icon: '➜', label: 'Navigating', colour: '#0b55c4', desc: 'Driving to the next grid point' },
  TURNING: { icon: '↻', label: 'Turning', colour: '#7c3aed', desc: 'Turning with MPU6500 feedback' },
  ARRIVED: { icon: '✓', label: 'Arrived', colour: '#159a61', desc: 'Book location reached' },
  DWELLING: { icon: '✓', label: 'Arrived', colour: '#159a61', desc: 'Waiting at the book location' },
  RETURNING: { icon: '⌂', label: 'Returning', colour: '#067a91', desc: 'Returning to the dock' },
  DOCKED: { icon: '✓', label: 'Docked', colour: '#159a61', desc: 'Robot is back at the dock' },
  STOPPED: { icon: '!', label: 'Stopped', colour: '#dc2626', desc: 'Safety stop — inspect the robot' },
};

const ROW_Y = { 4: 55, 3: 150, 2: 250, 1: 350 };
const COL_X = { A: 70, B: 270 };
const ACTIVE_STATES = new Set(['MOVING', 'TURNING', 'ARRIVED', 'DWELLING', 'RETURNING']);
const SEARCH_ALIASES = {
  'Computer Vision': 'Robotics, Vision and Control',
};
const BOOK_FALLBACK = {
  subtitle: 'Smart Library Collection',
  authors: ['Smart Library Catalogue'],
  rating: null,
  reviews: null,
  tags: ['Library Collection'],
  description: 'A numbered collection entry ready for fixed-grid robot navigation.',
};
const TRANSLATIONS = {
  en: {
    search: 'Search',
    myLibrary: 'My Library',
    collections: 'Collections',
    support: 'Support',
    popular: 'Popular searches:',
    location: 'Location in Library',
    send: 'Send Robot to This Book',
    add: '☆  Add to My List',
    map: 'Library Map',
    currentTask: 'Current Task',
    distance: 'Distance Progress',
    camera: 'Live Camera',
    robotStatus: 'Robot Status',
    trending: 'Trending Books',
  },
  zh: {
    search: '搜索',
    myLibrary: '我的书库',
    collections: '馆藏',
    support: '帮助',
    popular: '热门搜索：',
    location: '馆内位置',
    send: '派遣机器人前往此书',
    add: '☆  加入我的书单',
    map: '图书馆地图',
    currentTask: '当前任务',
    distance: '距离进度',
    camera: '实时摄像头',
    robotStatus: '机器人状态',
    trending: '热门书籍',
  },
};

let catalogue = [];
let selectedBook = null;
let currentState = 'IDLE';
let currentLanguage = localStorage.getItem('smartLibraryLanguage') || 'en';
let toastTimeout = null;
let searchTimer = null;
window.currentStudent = null;

window.addEventListener('DOMContentLoaded', () => {
  const input = document.getElementById('book-search');
  input.addEventListener('input', () => {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(() => previewSearch(false), 160);
  });
  input.addEventListener('keydown', event => {
    if (event.key === 'Enter') previewSearch(true);
  });
  applyLanguage();
  loadCatalogue();
  pollStatus();
  window.setInterval(pollStatus, 1000);
  initCheckin();
  loadStudentPortal();
  window.setInterval(loadStudentPortal, 5000);
});

async function loadCatalogue() {
  try {
    const [modeResponse, booksResponse] = await Promise.all([
      fetch('/navigation_mode'),
      fetch('/search_books?q='),
    ]);
    if (!modeResponse.ok || !booksResponse.ok) throw new Error('Catalogue unavailable');
    const mode = await modeResponse.json();
    catalogue = await booksResponse.json();
    buildBookOptions(catalogue);
    if (catalogue.length) {
      selectBook(catalogue[0]);
      document.getElementById('book-search').value = catalogue[0].title;
    }
    if (!mode.grid_configured) {
      showToast(`Grid calibration needed: ${mode.missing.join(', ')}`);
    }
  } catch (_) {
    showToast('Could not load the fixed-grid catalogue');
  }
}

function buildBookOptions(books) {
  const options = document.getElementById('book-options');
  options.replaceChildren();
  books.forEach(book => {
    const option = document.createElement('option');
    option.value = book.title;
    option.label = `${book.book_id} · ${book.location_code}`;
    options.appendChild(option);
  });
}

function useSuggestion(query) {
  document.getElementById('book-search').value = query;
  previewSearch(true, SEARCH_ALIASES[query] || query);
}

function clearSearch() {
  document.getElementById('book-search').value = '';
  if (catalogue.length) selectBook(catalogue[0]);
  document.getElementById('book-search').focus();
}

function addToList() {
  if (!selectedBook) return showToast('Select a book first');
  const saved = JSON.parse(localStorage.getItem('smartLibrarySavedBooks') || '[]');
  if (!saved.some(book => book.book_id === selectedBook.book_id)) {
    saved.push({
      title: selectedBook.title,
      book_id: selectedBook.book_id,
      location_code: selectedBook.location_code,
    });
    localStorage.setItem('smartLibrarySavedBooks', JSON.stringify(saved));
  }
  showToast(
    currentLanguage === 'zh'
      ? `已将《${selectedBook.title}》加入书单`
      : `${selectedBook.title} added to My Library`
  );
}

function toggleLanguage() {
  currentLanguage = currentLanguage === 'en' ? 'zh' : 'en';
  localStorage.setItem('smartLibraryLanguage', currentLanguage);
  applyLanguage();
  if (selectedBook) selectBook(selectedBook);
}

function applyLanguage() {
  const copy = TRANSLATIONS[currentLanguage];
  const navLabels = document.querySelectorAll('.nav-label');
  if (navLabels.length >= 4) {
    navLabels[0].textContent = copy.search;
    navLabels[1].textContent = copy.myLibrary;
    navLabels[2].textContent = copy.collections;
    navLabels[3].textContent = copy.support;
  }
  const popularLabel = document.getElementById('popular-label') || document.querySelector('.popular-row > span');
  if (popularLabel) popularLabel.textContent = copy.popular;
  const locHeading = document.getElementById('location-heading') || document.querySelector('.location-card small b');
  if (locHeading) locHeading.textContent = copy.location;
  const searchBtn = document.getElementById('search-btn');
  if (searchBtn) searchBtn.textContent = copy.search;
  const sendBtnLabel = document.getElementById('send-btn-label');
  if (sendBtnLabel) sendBtnLabel.textContent = copy.send;
  const addBtnLabel = document.getElementById('add-btn-label');
  if (addBtnLabel) addBtnLabel.textContent = copy.add;
  const mapHeading = document.getElementById('map-heading') || document.querySelector('.map-panel h2');
  if (mapHeading) mapHeading.textContent = copy.map;
  const taskHeading = document.getElementById('task-heading') || document.querySelector('.task-panel h2');
  if (taskHeading) taskHeading.textContent = copy.currentTask;
  const distHeading = document.getElementById('distance-heading') || document.querySelector('.progress-panel h2');
  if (distHeading) distHeading.textContent = copy.distance;
  const camHeading = document.getElementById('camera-heading') || document.querySelector('.camera-panel h2');
  if (camHeading) camHeading.textContent = copy.camera;
  const robotStatusHeading = document.getElementById('robot-status-heading') || document.querySelector('.robot-panel h2');
  if (robotStatusHeading) robotStatusHeading.textContent = copy.robotStatus;
  const trendingTitle = document.getElementById('trending-title');
  if (trendingTitle) trendingTitle.textContent = copy.trending;
  const langToggle = document.getElementById('language-toggle');
  if (langToggle) {
    langToggle.setAttribute(
      'aria-label',
      currentLanguage === 'en' ? 'Switch to Chinese' : '切换到英文'
    );
  }
}

async function previewSearch(showFeedback = true, queryOverride = null) {
  const query = document.getElementById('book-search').value.trim();
  try {
    const response = await fetch(
      `/search_books?q=${encodeURIComponent(queryOverride === null ? query : queryOverride)}`
    );
    if (!response.ok) throw new Error('Search failed');
    const results = await response.json();
    buildBookOptions(results.length ? results : catalogue);
    if (!results.length) {
      selectedBook = null;
      document.getElementById('search-result').textContent = 'No numbered book found';
      document.getElementById('send-btn').disabled = true;
      if (showFeedback) showToast('No book matches that title, ID, or location code');
      return;
    }
    selectBook(results[0]);
    if (showFeedback) showToast(`Found ${results[0].title} at ${results[0].location_code}`);
  } catch (_) {
    if (showFeedback) showToast('Search is temporarily unavailable');
  }
}

function selectBook(book) {
  selectedBook = book;
  const box = String(book.box_id || '').toUpperCase();
  const row = Number.parseInt(box, 10);
  const column = box.slice(-1);
  const detail = { ...BOOK_FALLBACK, ...book };
  document.getElementById('book-title').textContent = book.title;
  document.getElementById('cover-title').textContent = book.title.toUpperCase();
  document.getElementById('book-subtitle').textContent = detail.subtitle;
  document.getElementById('book-meta').textContent = detail.authors.join(', ');
  const rating = document.querySelector('.rating');
  rating.querySelector('b').textContent = detail.rating ? '★★★★★' : '★★★★☆';
  rating.querySelector('span').textContent = detail.rating
    ? `${detail.rating}  (${detail.reviews} reviews)`
    : 'Library collection';
  const tags = document.querySelector('.book-tags');
  tags.replaceChildren(...detail.tags.map(tag => {
    const element = document.createElement('span');
    element.textContent = tag;
    return element;
  }));
  document.getElementById('book-description').textContent = detail.description;
  document.getElementById('search-result').textContent =
    `${book.title} · ${book.book_id} · ${book.location_code}`;
  document.getElementById('book-location').textContent =
    currentLanguage === 'zh'
      ? `纸箱 ${box} / 第 ${book.layer} 层 / 位置 ${book.position}`
      : `Box ${box} / Layer ${book.layer} / Position ${book.position}`;
  document.getElementById('location-code').textContent =
    currentLanguage === 'zh'
      ? `机器人导航目标：${box}（第 ${row} 行，${column} 列）`
      : `Robot navigation target: ${box} (Row ${row}, Column ${column})`;
  document.getElementById('task-location').textContent = book.location_code;
  document.getElementById('task-query-copy').textContent =
    `${box} / Layer ${book.layer} / Position ${book.position}`;
  document.getElementById('task-plan-copy').textContent =
    `Dock → Row ${row} → ${column === 'A' ? 'Left' : 'Right'} → ${box}`;
  document.getElementById('task-arrive-copy').textContent =
    `Stop at ${box}, layer ${book.layer}, position ${book.position}`;
  document.getElementById('progress-target').textContent = `Target: ${box}`;
  document.getElementById('step-progress').textContent = `Est. time: ~${20 + row * 8} s`;
  document.getElementById('camera-target').textContent = box;
  document.getElementById('quick-code').textContent = book.location_code;
  updateMap(box);
  document.getElementById('send-btn').disabled = ACTIVE_STATES.has(currentState);
}

function updateMap(box) {
  document.querySelectorAll('.shelf-box').forEach(element => {
    element.classList.toggle('target', element.dataset.box === box);
  });
  const match = /^([1-4])([AB])$/.exec(box);
  if (!match) return;
  const targetY = ROW_Y[Number(match[1])];
  const targetX = COL_X[match[2]];
  document.getElementById('route-path').setAttribute(
    'd', `M 170 438 L 170 ${targetY} L ${targetX} ${targetY}`
  );
  document.getElementById('return-path').setAttribute(
    'd', `M ${targetX} ${targetY + 9} L 180 ${targetY + 9} L 180 438`
  );
}

async function sendRobot() {
  if (!window.currentStudent) {
    showToast('Please check in first');
    return;
  }

  const typedQuery = document.getElementById('book-search').value.trim();
  const selectedMatchesInput = selectedBook && (
    [selectedBook.title, selectedBook.book_id, selectedBook.location_code]
      .some(value => value.toLowerCase() === typedQuery.toLowerCase())
    || SEARCH_ALIASES[typedQuery] === selectedBook.title
  );
  const query = selectedMatchesInput ? selectedBook.book_id : typedQuery;
  if (!query) {
    showToast('Enter a book title, book ID, or location code');
    return;
  }
  const button = document.getElementById('send-btn');
  button.disabled = true;
  try {
    const response = await fetch('/api/borrow', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ student_id: window.currentStudent.id, book_query: query }),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) {
      showToast(data.message || 'Unable to start this route');
      return;
    }
    // Update local object structure based on backend spec or map to existing if needed
    // Usually the response includes `book` which has book_code/location_code
    const bookInfo = data.book || data;
    const matchingBook = catalogue.find(book =>
      book.book_id === bookInfo.book_code || book.location_code === bookInfo.location_code
    );
    if (matchingBook) selectBook(matchingBook);
    document.getElementById('book-search').value = data.title;
    showToast(`Robot dispatched to ${data.location_code}`);
    await pollStatus();
  } catch (_) {
    showToast('Failed to start the route');
  } finally {
    if (!ACTIVE_STATES.has(currentState)) button.disabled = false;
  }
}

async function resetRobot() {
  try {
    const response = await fetch('/reset', { method: 'POST' });
    if (!response.ok) throw new Error('Reset failed');
    showToast('Robot stopped and mission cleared');
    await pollStatus();
  } catch (_) {
    showToast('Emergency reset failed');
  }
}

async function pollStatus() {
  try {
    const response = await fetch('/status');
    if (!response.ok) throw new Error('Offline');
    const data = await response.json();
    updateUI(data);
    const badge = document.getElementById('connection-badge');
    badge.textContent = '● Fixed Grid';
    badge.classList.remove('offline');
  } catch (_) {
    const badge = document.getElementById('connection-badge');
    badge.textContent = '● Offline';
    badge.classList.add('offline');
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
  document.getElementById('status-desc').textContent = data.current_step_label || cfg.desc;
  document.getElementById('reason-text').textContent = data.reason ? `Reason: ${data.reason}` : '';
  document.getElementById('task-action').textContent = data.current_action || cfg.desc;
  if (selectedBook && data.step_count) {
    const row = Number.parseInt(selectedBook.box_id, 10);
    document.getElementById('step-progress').textContent =
      `Est. time: ~${20 + row * 8} s · Step ${data.step_index}/${data.step_count}`;
  }

  const statePill = document.getElementById('robot-state');
  statePill.textContent = cfg.label;
  statePill.style.setProperty('--state-colour', cfg.colour);

  const cameraBadge = document.getElementById('overlay-state-badge');
  cameraBadge.textContent = state;
  cameraBadge.dataset.state = state;

  if (data.box_id) {
    const missionBook = catalogue.find(book =>
      book.location_code === data.location_code || book.title === data.book
    );
    if (missionBook) selectBook(missionBook);
    document.getElementById('camera-target').textContent = data.box_id;
  }
  
  if (data.current_student) {
    if (!window.currentStudent || window.currentStudent.id !== data.current_student.id) {
      window.currentStudent = null; // force update
      onStudentCheckedIn(data.current_student);
    }
  } else if (!data.current_student && window.currentStudent) {
    window.currentStudent = null;
    document.getElementById('student-badge').style.display = 'none';
    document.querySelector('.profile-icon').style.display = 'flex';
    initCheckin();
  }

  const telemetry = data.telemetry || {};
  const missionComplete = state === 'ARRIVED' || state === 'DWELLING' || state === 'DOCKED';
  const progress = missionComplete
    ? 100
    : Number.isFinite(Number(telemetry.segment_progress_percent))
      ? Math.max(0, Math.min(100, Number(telemetry.segment_progress_percent)))
      : 0;
  document.getElementById('progress-percent').textContent = `${Math.round(progress)}%`;
  document.getElementById('progress-fill').style.width = `${progress}%`;
  updateSensorCards(telemetry);
  updateTimeline(state);

  document.getElementById('send-btn').disabled =
    ACTIVE_STATES.has(state) || !selectedBook;
  document.getElementById('reset-btn').style.display =
    state === 'STOPPED' ? 'block' : 'none';

  if (state !== previous && state === 'ARRIVED') {
    showToast(`Book location reached: ${data.location_code}`);
    speak(`Book location reached. Box ${data.box_id}, layer ${data.layer}, position ${data.position}.`);
  } else if (state !== previous && state === 'DOCKED') {
    showToast('Robot returned to Dock');
    speak('Return complete. The robot is back at the dock.');
  } else if (state !== previous && state === 'STOPPED') {
    showToast(`Safety stop: ${data.reason || 'inspect the robot'}`);
  }
}

function updateSensorCards(telemetry) {
  const encoders = telemetry.encoders || {};
  setHealth('encoder-health', encoders.status || 'Waiting');
  document.getElementById('encoder-detail').textContent =
    `L ${formatReading(encoders.left)} · R ${formatReading(encoders.right)}`;

  const imu = telemetry.imu || {};
  setHealth('imu-health', imu.status || 'Ready');

  const ultrasonic = telemetry.ultrasonic || {};
  setHealth('ultrasonic-health', ultrasonic.status || 'Waiting');
  document.getElementById('ultrasonic-detail').textContent =
    `L ${formatReading(ultrasonic.left)} · C ${formatReading(ultrasonic.center)} · R ${formatReading(ultrasonic.right)}`;
}

function setHealth(id, status) {
  const element = document.getElementById(id);
  const normalised = String(status);
  element.textContent = normalised;
  element.dataset.health = /stop|error|blocked|stall/i.test(normalised) ? 'bad' : 'ok';
}

function formatReading(value) {
  return value === null || value === undefined ? '—' : value;
}

function updateTimeline(state) {
  const ids = ['task-query', 'task-plan', 'task-navigate', 'task-arrive', 'task-return'];
  ids.forEach(id => document.getElementById(id).classList.remove('active', 'done'));
  const completeThrough = index => {
    for (let i = 0; i <= index; i += 1) document.getElementById(ids[i]).classList.add('done');
  };

  if (state === 'IDLE' || state === 'STOPPED') {
    document.getElementById('task-query').classList.add('active');
  } else if (state === 'MOVING' || state === 'TURNING') {
    completeThrough(1);
    document.getElementById('task-navigate').classList.add('active');
  } else if (state === 'ARRIVED' || state === 'DWELLING') {
    completeThrough(2);
    document.getElementById('task-arrive').classList.add('active');
  } else if (state === 'RETURNING') {
    completeThrough(3);
    document.getElementById('task-return').classList.add('active');
  } else if (state === 'DOCKED') {
    completeThrough(4);
  }

  const searchStep = document.getElementById('journey-search');
  const navigateStep = document.getElementById('journey-navigate');
  const arriveStep = document.getElementById('journey-arrive');
  [searchStep, navigateStep, arriveStep].forEach(step => step.classList.remove('active', 'done'));
  if (state === 'IDLE' || state === 'STOPPED') {
    searchStep.classList.add('active');
  } else if (state === 'MOVING' || state === 'TURNING' || state === 'RETURNING') {
    searchStep.classList.add('done');
    navigateStep.classList.add('active');
  } else {
    searchStep.classList.add('done');
    navigateStep.classList.add('done');
    arriveStep.classList.add(state === 'DOCKED' ? 'done' : 'active');
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
  toastTimeout = window.setTimeout(() => toast.classList.remove('show'), 3500);
}

// Student Check-In and Portal logic
function initCheckin() {
  document.getElementById('checkin-panel').style.display = 'block';
  document.getElementById('book-panel').style.display = 'none';
  document.getElementById('workspace-panel').style.display = 'none';
  document.getElementById('return-panel').style.display = 'none';
}

async function manualCheckin() {
  const qrCode = document.getElementById('manual-student-id').value.trim();
  if (!qrCode) return;
  try {
    const res = await fetch('/api/checkin', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ qr_code: qrCode })
    });
    const data = await res.json();
    if (data.ok && data.student) {
      onStudentCheckedIn(data.student);
    } else {
      showToast('Check-in failed');
    }
  } catch (err) {
    showToast('Error connecting to check-in');
  }
}

function onStudentCheckedIn(student) {
  if (window.currentStudent && window.currentStudent.id === student.id) return;
  window.currentStudent = student;
  
  document.getElementById('checkin-panel').style.display = 'none';
  
  showToast(`Welcome, ${student.name}!`);
  
  const badge = document.getElementById('student-badge');
  badge.textContent = student.name;
  badge.style.display = 'inline-block';
  document.querySelector('.profile-icon').style.display = 'none';

  if (student.has_book) {
    document.getElementById('return-panel').style.display = 'block';
    document.getElementById('borrowed-book-title').textContent = student.borrowed_book_id || 'A book';
    document.getElementById('book-panel').style.display = 'none';
    document.getElementById('workspace-panel').style.display = 'none';
  } else {
    document.getElementById('return-panel').style.display = 'none';
    document.getElementById('book-panel').style.display = 'block';
    document.getElementById('workspace-panel').style.display = 'block';
  }
}

async function returnBook() {
  if (!window.currentStudent) return;
  try {
    const res = await fetch('/api/return_book', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ student_id: window.currentStudent.id })
    });
    const data = await res.json();
    if (data.ok) {
      showToast(`Book returned successfully`);
      window.currentStudent = null;
      document.getElementById('student-badge').style.display = 'none';
      document.querySelector('.profile-icon').style.display = 'flex';
      initCheckin();
    } else {
      showToast('Return failed');
    }
  } catch (err) {
    showToast('Error connecting to server');
  }
}

async function loadStudentPortal() {
  try {
    const res = await fetch('/api/students');
    const students = await res.json();
    const tbody = document.getElementById('student-portal-tbody');
    tbody.innerHTML = '';
    students.forEach(st => {
      const tr = document.createElement('tr');
      const statusClass = st.has_book ? 'status-borrowed' : 'status-available';
      const statusText = st.has_book ? 'Borrowed' : 'Available';
      tr.innerHTML = `
        <td>${st.name}</td>
        <td class="${statusClass}">${statusText}</td>
      `;
      tbody.appendChild(tr);
    });
  } catch (err) {
    console.warn('Could not load student portal', err);
  }
}

