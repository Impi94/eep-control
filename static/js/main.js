/* EEP Control — main.js */

const socket = io();
let currentState = 'idle';
let _sliderDragging = false;
let _lastLogTs = 0;

// ── WebSocket ────────────────────────────────────────────────────────────────

socket.on('connect', () => {
    connDot('connected');
    socket.emit('request_state');
    fetchLog();
});

socket.on('disconnect', () => connDot('disconnected'));

socket.on('state_update', applyState);

socket.on('telemetry', data => {
    applyState(data);
    if (data.system) applySystem(data.system);
});

// ── Состояние ────────────────────────────────────────────────────────────────

const STATE_LABELS = {
    idle: 'Простой', ready: 'Готов', running: 'Работает',
    paused: 'Пауза', finishing: 'Завершение',
    error: 'Ошибка', 'e-stop': 'E-STOP', homing: 'Хоминг'
};

function applyState(data) {
    if (!data) return;
    currentState = data.state;

    // Бейдж состояния
    const badge = document.getElementById('stateBadge');
    badge.className = 'state-badge ' + data.state;
    badge.querySelector('.state-text').textContent = STATE_LABELS[data.state] || data.state;

    // Кнопки управления
    const running  = data.state === 'running';
    const idle     = data.state === 'idle';
    const paused   = data.state === 'paused';
    const estop    = data.state === 'e-stop' || data.state === 'error';
    const busy     = data.state === 'finishing' || data.state === 'homing';

    const btnStart = document.getElementById('btnStart');
    const btnPause = document.getElementById('btnPause');
    const btnStop  = document.getElementById('btnStop');

    btnStart.disabled = running || busy;
    btnPause.disabled = !running;
    btnStop.disabled  = idle || estop || busy;

    // На паузе — кнопка «Старт» становится «Продолжить»
    btnStart.querySelector('.btn-label').textContent = paused ? 'Продолжить' : 'Старт';

    // Датчики
    if (data.sensors) {
        const s = data.sensors;
        const pulse = data.pulse || {};
        const proc  = data.process || {};

        setGauge('vActual',   'vBar',      s.voltage_actual,        pulse.voltage || 300, 1);
        setGauge('iActual',   'iBar',      s.current_actual,         pulse.current || 50,  2);
        setGauge('gapActual', 'gapBar',    s.gap_actual,             1.0,                  3);
        setGauge('elTemp',    'elTempBar', s.electrolyte_temp,       60,                   1);

        setText('vTarget',   pulse.voltage);
        setText('iTarget',   pulse.current);
        setText('gapTarget', proc.gap);

        setText('flowRate',     s.electrolyte_flow.toFixed(1));
        setText('conductivity', s.electrolyte_conductivity.toFixed(1));
        setText('vibration',    s.vibration.toFixed(2));
        setText('tankLevel',    s.tank_level.toFixed(0));
        setText('electrodeWear', s.electrode_wear.toFixed(2));
        setText('energyConsumed', (data.stats || {}).energy_consumed
            ? data.stats.energy_consumed.toFixed(2) : '0.00');
    }

    // Статистика
    if (data.stats) {
        const st = data.stats;
        setText('pulseCount',      st.pulse_count.toLocaleString('ru'));
        setText('materialRemoved', st.material_removed.toFixed(3));
        setText('shortCircuits',   st.short_circuits);
        setText('arcCount',        st.arc_count);
        setText('efficiency',      st.efficiency.toFixed(1) + ' %');
        setText('elapsedTime',     formatTime(st.elapsed_seconds));

        const pct = st.progress || 0;
        document.getElementById('progressBar').style.width = pct + '%';
        setText('progressPercent', pct.toFixed(1) + '%');
    }

    // Синхронизация слайдеров с сервером (пока не тащим)
    if (data.pulse && !_sliderDragging) syncSliders(data.pulse);

    // Ошибки
    if (Array.isArray(data.errors)) updateErrors(data.errors);

    // Индикатор железо/симуляция
    const hwBadge = document.getElementById('hwBadge');
    if (hwBadge) {
        const isHw = data.hw_connected;
        hwBadge.textContent = isHw ? '● Железо' : '○ Симуляция';
        hwBadge.style.color = isHw ? 'var(--success)' : 'var(--warning)';
    }
}

// ── Системная информация RPi ─────────────────────────────────────────────────

function applySystem(sys) {
    setText('sysCpu',  (sys.cpu_percent || 0).toFixed(0) + '%');
    setText('sysTemp', sys.cpu_temp != null ? sys.cpu_temp.toFixed(1) + '°C' : '--°C');
}

// ── Журнал (опрос раз в секунду) ────────────────────────────────────────────

async function fetchLog() {
    try {
        const r = await fetch('/api/log?limit=200');
        if (!r.ok) return;
        const entries = await r.json();
        const fresh = entries.filter(e => e.timestamp > _lastLogTs);
        if (!fresh.length) return;
        _lastLogTs = fresh[fresh.length - 1].timestamp;
        appendLogEntries(fresh);
    } catch (_) {}
}

function appendLogEntries(entries) {
    const box = document.getElementById('logContainer');
    // Убрать заглушку при первом реальном сообщении
    const placeholder = box.querySelector('.log-placeholder');
    if (placeholder) placeholder.remove();

    entries.forEach(e => {
        const div = document.createElement('div');
        div.className = 'log-entry ' + (e.level || 'info');
        div.innerHTML =
            `<span class="log-time">${e.time}</span>` +
            `<span class="log-msg">${esc(e.message)}</span>`;
        box.appendChild(div);
    });

    box.scrollTop = box.scrollHeight;

    // Ограничение DOM — 200 строк
    while (box.children.length > 200) box.removeChild(box.firstChild);
}

setInterval(fetchLog, 1000);

// ── Команды управления ───────────────────────────────────────────────────────

async function controlAction(action) {
    // На паузе Старт = Возобновить
    if (action === 'start' && currentState === 'paused') action = 'resume';

    try {
        const r = await fetch('/api/control', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ action })
        });
        const res = await r.json();
        if (!res.success) console.warn('[control]', res.error);
    } catch (e) {
        console.error('[control] fetch error', e);
    }
}

function emergencyStop() { controlAction('emergency_stop'); }
function resetErrors()   { controlAction('reset_errors'); }

// ── Параметры импульсов ──────────────────────────────────────────────────────

const SLIDER_DEFS = {
    voltage:   { displayId: 'voltageDisplay',  suffix: ' В' },
    current:   { displayId: 'currentDisplay',  suffix: ' А' },
    pulse_on:  { displayId: 'pulseOnDisplay',  suffix: ' мкс' },
    pulse_off: { displayId: 'pulseOffDisplay', suffix: ' мкс' },
};

function updateParam(param, rawValue) {
    _sliderDragging = true;
    const def = SLIDER_DEFS[param];
    if (def) setText(def.displayId, parseFloat(rawValue) + def.suffix);
    clearTimeout(updateParam._t);
    updateParam._t = setTimeout(() => { _sliderDragging = false; }, 600);
}

async function applyPulseParams() {
    const params = {
        voltage:   parseFloat(document.getElementById('voltageSlider').value),
        current:   parseFloat(document.getElementById('currentSlider').value),
        pulse_on:  parseFloat(document.getElementById('pulseOnSlider').value),
        pulse_off: parseFloat(document.getElementById('pulseOffSlider').value),
    };
    await fetch('/api/params/pulse', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(params)
    });
}

function syncSliders(pulse) {
    const map = {
        voltageSlider:  [pulse.voltage,   'voltageDisplay',  ' В'],
        currentSlider:  [pulse.current,   'currentDisplay',  ' А'],
        pulseOnSlider:  [pulse.pulse_on,  'pulseOnDisplay',  ' мкс'],
        pulseOffSlider: [pulse.pulse_off, 'pulseOffDisplay', ' мкс'],
    };
    Object.entries(map).forEach(([sliderId, [val, dispId, sfx]]) => {
        const slider = document.getElementById(sliderId);
        if (slider) slider.value = val;
        setText(dispId, val + sfx);
    });
}

// ── Пресеты ──────────────────────────────────────────────────────────────────

async function loadPreset(name, btn) {
    const r = await fetch('/api/preset/' + name, { method: 'POST' });
    const res = await r.json();
    if (res.success) {
        document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
        if (btn) btn.classList.add('active');
    }
}

// ── Ошибки ───────────────────────────────────────────────────────────────────

function updateErrors(errors) {
    const box = document.getElementById('errorsContainer');
    if (!errors.length) {
        box.innerHTML = '<div class="no-errors">Ошибок нет</div>';
        return;
    }
    box.innerHTML = errors.map(e =>
        `<div class="error-entry">
            <span class="error-code">${esc(e.code)}</span>
            <span class="error-time" style="color:var(--text-dim);margin-left:6px">${e.time}</span>
            <div style="margin-top:2px">${esc(e.message)}</div>
        </div>`
    ).join('');
}

// ── Очистка журнала ──────────────────────────────────────────────────────────

function clearLog() {
    const box = document.getElementById('logContainer');
    box.innerHTML = '<div class="log-placeholder log-entry info">' +
        '<span class="log-time">--:--</span>' +
        '<span class="log-msg">Журнал очищен</span></div>';
    _lastLogTs = Date.now() / 1000;
}

// ── Часы ─────────────────────────────────────────────────────────────────────

function tickClock() {
    const now = new Date();
    setText('clockDisplay',
        String(now.getHours()).padStart(2,'0') + ':' +
        String(now.getMinutes()).padStart(2,'0') + ':' +
        String(now.getSeconds()).padStart(2,'0')
    );
}
setInterval(tickClock, 1000);
tickClock();

// ── Утилиты ──────────────────────────────────────────────────────────────────

function setGauge(valueId, barId, value, max, decimals) {
    const v = value ?? 0;
    setText(valueId, v.toFixed(decimals));
    const bar = document.getElementById(barId);
    if (bar) bar.style.width = Math.min(100, Math.max(0, (v / max) * 100)).toFixed(1) + '%';
}

function setText(id, text) {
    const el = document.getElementById(id);
    if (el && text !== undefined && text !== null) el.textContent = text;
}

function connDot(cls) {
    const dot = document.querySelector('.conn-dot');
    if (dot) dot.className = 'conn-dot ' + cls;
}

function formatTime(sec) {
    sec = sec || 0;
    const h = Math.floor(sec / 3600);
    const m = Math.floor((sec % 3600) / 60);
    const s = sec % 60;
    return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
}

function esc(str) {
    return String(str ?? '')
        .replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
