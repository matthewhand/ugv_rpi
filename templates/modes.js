/**
 * Multi-mode shell: Raw / Chat / Seek + shared navbar persistence.
 */
(function () {
  'use strict';

  var STORAGE_KEY = 'ugv_app_mode';

  function $(id) {
    return document.getElementById(id);
  }

  function setMode(mode) {
    mode = mode || 'raw';
    if (mode !== 'raw' && mode !== 'chat' && mode !== 'seek') mode = 'raw';
    var panels = {
      raw: $('mode-panel-raw'),
      chat: $('mode-panel-chat'),
      seek: $('mode-panel-seek'),
    };
    var tabs = document.querySelectorAll('.ugv-mode-tabs [data-mode]');
    Object.keys(panels).forEach(function (m) {
      var el = panels[m];
      if (!el) return;
      var on = m === mode;
      el.classList.toggle('active', on);
      if (on) el.removeAttribute('hidden');
      else el.setAttribute('hidden', 'hidden');
    });
    tabs.forEach(function (btn) {
      var on = btn.getAttribute('data-mode') === mode;
      btn.classList.toggle('active', on);
      btn.setAttribute('aria-selected', on ? 'true' : 'false');
    });
    try {
      localStorage.setItem(STORAGE_KEY, mode);
    } catch (e) {}
    // Bust-cache MJPEG when entering chat/seek previews
    if (mode === 'chat' && $('chat-live-preview')) {
      $('chat-live-preview').src = '/video_feed?t=' + Date.now();
    }
    if (mode === 'seek' && $('seek-live-preview')) {
      $('seek-live-preview').src = '/video_feed?t=' + Date.now();
    }
  }

  function initModeTabs() {
    var tabs = document.querySelectorAll('.ugv-mode-tabs [data-mode]');
    tabs.forEach(function (btn) {
      btn.addEventListener('click', function () {
        setMode(btn.getAttribute('data-mode'));
      });
    });
    var initial = 'raw';
    try {
      initial = localStorage.getItem(STORAGE_KEY) || 'raw';
    } catch (e) {}
    setMode(initial);
  }

  // ---------- Chat panel ----------
  var chatHistory = [];

  function chatAdd(role, text) {
    var log = $('chat-log');
    if (!log) return;
    var div = document.createElement('div');
    div.className = 'ugv-chat-msg ' + role;
    div.textContent = text;
    log.appendChild(div);
    log.scrollTop = log.scrollHeight;
  }

  function chatSend() {
    var input = $('chat-input');
    var btn = $('chat-send-btn');
    if (!input || !btn) return;
    var message = (input.value || '').trim();
    if (!message) return;
    var attach = $('chat-attach') && $('chat-attach').checked;
    btn.disabled = true;
    chatAdd('user', message);
    input.value = '';
    chatAdd('sys', 'Thinking…');
    fetch('/api/ai/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        message: message,
        history: chatHistory,
        attach_snapshot: !!attach,
      }),
    })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        var sys = document.querySelectorAll('#chat-log .ugv-chat-msg.sys');
        if (sys.length) sys[sys.length - 1].remove();
        if (!d.success) {
          chatAdd('err', d.error || 'chat failed');
          return;
        }
        if (d.snapshot_data_url && $('chat-snap-preview')) {
          $('chat-snap-preview').src = d.snapshot_data_url;
        }
        chatAdd('ai', d.reply || '(empty)');
        if (Array.isArray(d.tool_calls) && d.tool_calls.length) {
          d.tool_calls.forEach(function (tc) {
            chatAdd(
              'sys',
              'tool ' +
                tc.name +
                ' → ' +
                JSON.stringify(tc.result || {}).slice(0, 160)
            );
          });
        }
        chatHistory.push({ role: 'user', content: message });
        chatHistory.push({ role: 'assistant', content: d.reply || '' });
        if (chatHistory.length > 24) chatHistory = chatHistory.slice(-24);
      })
      .catch(function (e) {
        chatAdd('err', String(e.message || e));
      })
      .finally(function () {
        btn.disabled = false;
      });
  }

  function initChat() {
    var send = $('chat-send-btn');
    var clear = $('chat-clear-btn');
    var snap = $('chat-snap-btn');
    var input = $('chat-input');
    if (send) send.addEventListener('click', chatSend);
    if (clear) {
      clear.addEventListener('click', function () {
        chatHistory = [];
        var log = $('chat-log');
        if (log) log.innerHTML = '';
        chatAdd('sys', 'Chat cleared.');
      });
    }
    if (snap) {
      snap.addEventListener('click', function () {
        fetch('/api/snapshot')
          .then(function (r) {
            return r.json();
          })
          .then(function (d) {
            if (d.success && d.data_url && $('chat-snap-preview')) {
              $('chat-snap-preview').src = d.data_url;
            }
          })
          .catch(function () {});
      });
    }
    if (input) {
      input.addEventListener('keydown', function (e) {
        if (e.key === 'Enter' && !e.shiftKey) {
          e.preventDefault();
          chatSend();
        }
      });
    }
    chatAdd('sys', 'Chat mode ready. Attach a still when you want vision context.');
  }

  // ---------- Seek panel ----------
  var seekPollTimer = null;
  var SEEK_REFEREE_KEY = 'ugv_seek_referee';
  var lastSeekCheckSeq = 0;
  var lastSeekStep = -1;
  var seekFireTimer = null;

  function seekLog(msg) {
    var log = $('seek-log');
    if (!log) return;
    var div = document.createElement('div');
    div.className = 'ugv-chat-msg sys';
    div.textContent = msg;
    log.appendChild(div);
    log.scrollTop = log.scrollHeight;
  }

  function getSeekReferee() {
    var llm = $('seek-referee-llm');
    if (llm && llm.checked) return 'llm';
    return 'detector';
  }

  function getSeekGoal() {
    if (getSeekReferee() === 'llm') {
      var t = $('seek-goal-text');
      return (t && t.value) || '';
    }
    var s = $('seek-goal-select');
    return (s && s.value) || '';
  }

  function syncSeekRefereeUI() {
    var ref = getSeekReferee();
    var detWrap = $('seek-goal-detector-wrap');
    var llmWrap = $('seek-goal-llm-wrap');
    if (detWrap) detWrap.hidden = ref !== 'detector';
    if (llmWrap) llmWrap.hidden = ref !== 'llm';
    try {
      localStorage.setItem(SEEK_REFEREE_KEY, ref);
    } catch (e) {}
    // Idle badge reflects selected referee type
    if (!$('seek-detector-bar') || ($('seek-detector-bar').classList.contains('is-running'))) return;
    setDetectorBar('idle', ref === 'llm' ? 'Judge: idle' : 'Detector: idle', '');
  }

  function populateDetectorLabels(labels) {
    var sel = $('seek-goal-select');
    if (!sel) return;
    var preferred = 'dog';
    sel.innerHTML = '';
    (labels || []).forEach(function (lab) {
      var opt = document.createElement('option');
      opt.value = lab;
      opt.textContent = lab;
      if (lab === preferred) opt.selected = true;
      sel.appendChild(opt);
    });
    if (!sel.value && sel.options.length) sel.selectedIndex = 0;
  }

  function setDetectorBar(mode, label, meta) {
    var bar = $('seek-detector-bar');
    var lab = $('seek-detector-label');
    var met = $('seek-detector-meta');
    if (!bar) return;
    bar.classList.remove('is-idle', 'is-running', 'is-firing', 'is-found', 'is-checking');
    bar.classList.add('is-' + (mode || 'idle'));
    if (lab) lab.textContent = label || '';
    if (met) met.textContent = meta || '';
  }

  function pulseDetectorFire(st) {
    var bar = $('seek-detector-bar');
    var el = $('seek-status');
    if (bar) {
      bar.classList.add('is-firing');
      if (seekFireTimer) clearTimeout(seekFireTimer);
      seekFireTimer = setTimeout(function () {
        bar.classList.remove('is-firing');
        // restore running/found/idle after flash
        if (st && st.phase === 'running') bar.classList.add('is-running');
        else if (st && st.phase === 'found') bar.classList.add('is-found');
        else bar.classList.add('is-idle');
      }, 450);
    }
    if (el) {
      el.classList.add('is-firing');
      setTimeout(function () {
        el.classList.remove('is-firing');
      }, 450);
    }
  }

  function formatCheckAge(st) {
    if (!st || !st.last_check_at) return '';
    var age = Math.max(0, (Date.now() / 1000) - Number(st.last_check_at));
    if (age < 1.5) return 'just now';
    if (age < 60) return Math.round(age) + 's ago';
    return Math.round(age / 60) + 'm ago';
  }

  function updateDetectorBar(st, opts) {
    opts = opts || {};
    var phase = (st && st.phase) || 'idle';
    var det = (st && st.last_detection) || {};
    var ref = (st && st.referee) || det.referee || getSeekReferee();
    var isLlm = ref === 'llm' || det.referee === 'llm';
    var name = isLlm ? 'Judge' : 'Detector';
    var meta = [];
    if (st && st.step) meta.push('step ' + st.step);
    if (st && st.seek_phase) meta.push(String(st.seek_phase));
    if (st && st.last_nav && st.last_nav.action) meta.push('nav ' + st.last_nav.action);
    if (st && st.last_check_seq) meta.push('#' + st.last_check_seq);
    var age = formatCheckAge(st);
    if (age) meta.push(age);

    if (opts.checking) {
      setDetectorBar('checking', name + ': checking…', meta.join(' · '));
      return;
    }
    if (phase === 'running') {
      var foundBit = det.found ? 'MATCH' : 'no match';
      var labels = '';
      if (!isLlm && det.labels_found && det.labels_found.length) {
        labels = ' · saw ' + det.labels_found.join(', ');
      } else if (isLlm && det.reason) {
        labels = ' · ' + String(det.reason).slice(0, 60);
      }
      setDetectorBar(
        'running',
        name + ': running (' + foundBit + ')' + labels,
        meta.join(' · ')
      );
    } else if (phase === 'found') {
      setDetectorBar('found', name + ': FOUND', meta.join(' · '));
    } else if (phase === 'stopped' || phase === 'timeout' || phase === 'failed') {
      setDetectorBar(
        'idle',
        name + ': ' + phase,
        meta.join(' · ')
      );
    } else {
      setDetectorBar('idle', name + ': idle', '');
    }
  }

  function renderSeekStatus(st, opts) {
    var el = $('seek-status');
    if (!el || !st) return;
    opts = opts || {};
    var phase = st.phase || 'idle';
    var cls = 'phase-' + phase;
    var det = st.last_detection || {};
    var ref = st.referee || det.referee || '—';
    var seq = st.last_check_seq || 0;
    var step = st.step || 0;
    var fired = false;
    if (seq && seq !== lastSeekCheckSeq) {
      fired = true;
      lastSeekCheckSeq = seq;
    } else if (step && step !== lastSeekStep && phase === 'running') {
      // step advanced even if seq missing (older server)
      fired = lastSeekStep >= 0;
      lastSeekStep = step;
    }
    if (step) lastSeekStep = step;

    var lines = [
      'Phase: ' + phase,
      'Seek cycle: ' + (st.seek_phase || '—'),
      'Nav: ' +
        ((st.last_nav && st.last_nav.action) || '—') +
        (st.last_nav && st.last_nav.reason ? ' — ' + String(st.last_nav.reason).slice(0, 80) : ''),
      'Referee: ' + ref,
      'Goal: ' + (st.goal_label || st.goal_text || '—'),
      'Step: ' +
        (st.step || 0) +
        ' / ' +
        (st.max_steps === 0 || st.max_steps === '0' ? '∞' : st.max_steps || '—'),
      'Message: ' + (st.message || ''),
    ];
    if (st.last_check_seq) {
      lines.push(
        'Detector fires: #' +
          st.last_check_seq +
          (st.last_check_at ? ' · ' + formatCheckAge(st) : '')
      );
    }
    if (det && typeof det === 'object' && Object.keys(det).length) {
      if (ref === 'llm' || det.referee === 'llm') {
        lines.push(
          'Judge found: ' +
            !!det.found +
            (det.reason ? ' — ' + det.reason : '') +
            (det.response_format ? ' [' + det.response_format + ']' : '')
        );
      } else {
        lines.push(
          'Detector found: ' +
            !!det.found +
            ' | labels: ' +
            JSON.stringify(det.labels_found || []) +
            ' | matches: ' +
            (det.match_count || 0)
        );
      }
    }
    if (st.error) lines.push('Error: ' + st.error);
    el.innerHTML =
      '<span class="' +
      cls +
      '">' +
      lines[0] +
      '</span>\n' +
      lines.slice(1).join('\n');

    updateDetectorBar(st, opts);
    if (fired && phase === 'running') {
      pulseDetectorFire(st);
      // brief log line so the fire is visible in the transcript too
      var det = st.last_detection || {};
      var brief =
        'check #' +
        (st.last_check_seq || '?') +
        ' step ' +
        (st.step || '?') +
        ' found=' +
        !!det.found;
      if (det.labels_found && det.labels_found.length) {
        brief += ' labels=' + det.labels_found.join(',');
      }
      seekLog(brief);
    }
  }

  function pollSeek() {
    fetch('/api/ai/seek/status')
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        var st = (d && d.status) || {};
        renderSeekStatus(st);
        if (st.phase && st.phase !== 'running' && seekPollTimer) {
          clearInterval(seekPollTimer);
          seekPollTimer = null;
          seekLog('Seek ended: ' + st.phase + ' — ' + (st.message || ''));
          updateDetectorBar(st);
        }
      })
      .catch(function () {});
  }

  function getSeekOnFound() {
    var sel = $('seek-on-found');
    return (sel && sel.value) || 'none';
  }

  function getSeekOnFoundTts() {
    var inp = $('seek-on-found-tts');
    var v = (inp && inp.value) || '';
    return v.trim() || 'I have found the {goal}.';
  }

  function syncSeekOnFoundUI() {
    var wrap = $('seek-on-found-tts-wrap');
    if (!wrap) return;
    wrap.hidden = getSeekOnFound() !== 'tts';
  }

  function seekStart() {
    var goal = getSeekGoal();
    var referee = getSeekReferee();
    var onFound = getSeekOnFound();
    var onFoundTts = getSeekOnFoundTts();
    lastSeekCheckSeq = 0;
    lastSeekStep = -1;
    seekLog(
      'Starting seek (' +
        referee +
        ') for: ' +
        goal +
        ' · upon found: ' +
        (onFound === 'tts' ? 'TTS “' + onFoundTts + '”' : 'do nothing')
    );
    setDetectorBar(
      'running',
      (referee === 'llm' ? 'Judge' : 'Detector') + ': starting…',
      goal
    );
    fetch('/api/ai/seek/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        goal: goal,
        referee: referee,
        max_steps: 0, // unlimited; stop on found / Stop (timeout_s 0 = no time limit)
        timeout_s: 0,
        on_found: onFound,
        on_found_tts: onFoundTts,
      }),
    })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (!d.success) {
          seekLog('Start failed: ' + (d.error || 'unknown'));
          setDetectorBar('idle', 'Detector: idle', '');
          return;
        }
        renderSeekStatus(d.status || {});
        if (seekPollTimer) clearInterval(seekPollTimer);
        // Poll faster so each detector fire is visible
        seekPollTimer = setInterval(pollSeek, 400);
        pollSeek();
      })
      .catch(function (e) {
        seekLog(String(e.message || e));
        setDetectorBar('idle', 'Detector: idle', '');
      });
  }

  function seekStop() {
    fetch('/api/ai/seek/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        renderSeekStatus((d && d.status) || {});
        seekLog('Stop requested');
      })
      .catch(function (e) {
        seekLog(String(e.message || e));
      });
  }

  function seekCheckOnce() {
    var goal = getSeekGoal();
    var referee = getSeekReferee();
    var name = referee === 'llm' ? 'Judge' : 'Detector';
    setDetectorBar('checking', name + ': checking…', goal);
    seekLog(name + ' check once: ' + goal);
    fetch('/api/ai/seek/check', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ goal: goal, referee: referee }),
    })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (!d.success) {
          seekLog('Check failed: ' + (d.error || ''));
          setDetectorBar('idle', name + ': idle', 'error');
          return;
        }
        var c = d.check || {};
        // One-shot: flash fire + result in bar
        setDetectorBar(
          c.found ? 'found' : 'idle',
          name + ': ' + (c.found ? 'FOUND' : 'no match'),
          c.labels_found && c.labels_found.length
            ? c.labels_found.join(', ')
            : c.reason || ''
        );
        pulseDetectorFire({ phase: c.found ? 'found' : 'idle' });
        setTimeout(function () {
          if (!seekPollTimer) setDetectorBar('idle', name + ': idle', '');
        }, 1200);
        if ((d.referee || referee) === 'llm') {
          seekLog(
            'LLM judge ' +
              JSON.stringify(d.goal_label) +
              ': found=' +
              !!c.found +
              (c.reason ? ' — ' + c.reason : '') +
              (c.response_format ? ' [' + c.response_format + ']' : '')
          );
        } else {
          seekLog(
            'Detector ' +
              d.goal_label +
              ': found=' +
              !!c.found +
              ' labels=' +
              JSON.stringify(c.labels_found || []) +
              ' matches=' +
              (c.match_count || 0)
          );
        }
      })
      .catch(function (e) {
        seekLog(String(e.message || e));
        setDetectorBar('idle', name + ': idle', 'error');
      });
  }

  function initSeek() {
    var start = $('seek-start-btn');
    var stop = $('seek-stop-btn');
    var check = $('seek-check-btn');
    if (start) start.addEventListener('click', seekStart);
    if (stop) stop.addEventListener('click', seekStop);
    if (check) check.addEventListener('click', seekCheckOnce);

    var radios = document.querySelectorAll('input[name="seek-referee"]');
    radios.forEach(function (r) {
      r.addEventListener('change', syncSeekRefereeUI);
    });
    var onFoundSel = $('seek-on-found');
    if (onFoundSel) onFoundSel.addEventListener('change', syncSeekOnFoundUI);
    syncSeekOnFoundUI();
    try {
      var saved = localStorage.getItem(SEEK_REFEREE_KEY);
      if (saved === 'llm' && $('seek-referee-llm')) $('seek-referee-llm').checked = true;
      if (saved === 'detector' && $('seek-referee-detector')) $('seek-referee-detector').checked = true;
    } catch (e) {}
    syncSeekRefereeUI();

    // Default labels if API slow/unavailable
    populateDetectorLabels([
      'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair',
      'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant',
      'sheep', 'sofa', 'train', 'tvmonitor',
    ]);
    fetch('/api/ai/seek/labels')
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (d && d.success && Array.isArray(d.detector_labels)) {
          populateDetectorLabels(d.detector_labels);
        }
      })
      .catch(function () {});

    seekLog(
      'Seek ready. Detector = closed class list; LLM vision = free-text + JSON found true/false.'
    );
  }

  function boot() {
    initModeTabs();
    initChat();
    initSeek();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }

  // export for tests / console
  window.ugvSetMode = setMode;
})();
