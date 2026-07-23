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
    chatAdd('sys', 'Chat mode ready. Tools follow the capability pills on /ai if needed.');
  }

  // ---------- Seek panel ----------
  var seekPollTimer = null;

  function seekLog(msg) {
    var log = $('seek-log');
    if (!log) return;
    var div = document.createElement('div');
    div.className = 'ugv-chat-msg sys';
    div.textContent = msg;
    log.appendChild(div);
    log.scrollTop = log.scrollHeight;
  }

  function renderSeekStatus(st) {
    var el = $('seek-status');
    if (!el || !st) return;
    var phase = st.phase || 'idle';
    var cls = 'phase-' + phase;
    var det = st.last_detection || {};
    var lines = [
      'Phase: ' + phase,
      'Goal: ' + (st.goal_label || st.goal_text || '—'),
      'Step: ' + (st.step || 0) + ' / ' + (st.max_steps || '—'),
      'Message: ' + (st.message || ''),
    ];
    if (det && typeof det === 'object') {
      lines.push(
        'OpenCV found: ' +
          !!det.found +
          ' | labels: ' +
          JSON.stringify(det.labels_found || []) +
          ' | matches: ' +
          (det.match_count || 0)
      );
    }
    if (st.error) lines.push('Error: ' + st.error);
    el.innerHTML =
      '<span class="' +
      cls +
      '">' +
      lines[0] +
      '</span>\n' +
      lines.slice(1).join('\n');
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
        }
      })
      .catch(function () {});
  }

  function seekStart() {
    var goalEl = $('seek-goal');
    var goal = (goalEl && goalEl.value) || '';
    seekLog('Starting seek for: ' + goal);
    fetch('/api/ai/seek/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ goal: goal, max_steps: 12, timeout_s: 180 }),
    })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (!d.success) {
          seekLog('Start failed: ' + (d.error || 'unknown'));
          return;
        }
        renderSeekStatus(d.status || {});
        if (seekPollTimer) clearInterval(seekPollTimer);
        seekPollTimer = setInterval(pollSeek, 800);
      })
      .catch(function (e) {
        seekLog(String(e.message || e));
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
    var goalEl = $('seek-goal');
    var goal = (goalEl && goalEl.value) || '';
    fetch('/api/ai/seek/check', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ goal: goal }),
    })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (!d.success) {
          seekLog('Check failed: ' + (d.error || ''));
          return;
        }
        var c = d.check || {};
        seekLog(
          'Check ' +
            d.goal_label +
            ': found=' +
            !!c.found +
            ' labels=' +
            JSON.stringify(c.labels_found || []) +
            ' matches=' +
            (c.match_count || 0)
        );
      })
      .catch(function (e) {
        seekLog(String(e.message || e));
      });
  }

  function initSeek() {
    var start = $('seek-start-btn');
    var stop = $('seek-stop-btn');
    var check = $('seek-check-btn');
    if (start) start.addEventListener('click', seekStart);
    if (stop) stop.addEventListener('click', seekStop);
    if (check) check.addEventListener('click', seekCheckOnce);
    seekLog('Seek mode ready. OpenCV judges success; LLM only steers.');
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
