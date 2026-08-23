/**
 * Multi-mode shell: Raw / Chat / Seek / Track / Loadout + shared navbar persistence.
 */
(function () {
  'use strict';

  var STORAGE_KEY = 'ugv_app_mode';
  var VIDEO_FEED_URL = '/video_feed';
  var lastSeekPanoDataUrl = '';
  var imgRetryTimers = {};
  // Pan overlay animation — match real gimbal rate (~54°/s, same as app._SEEK_PAN_EST_DPS)
  var PAN_EST_DPS = 54;
  var panAnim = {
    from: 0,
    to: 0,
    startMs: 0,
    durMs: 0,
    active: false,
    lastCmd: null,
    lastNeedle: 0,
    raf: null,
    settling: false,
  };

  function $(id) {
    return document.getElementById(id);
  }

  /** Cache-bust a stream URL so the browser reopens MJPEG. */
  function videoFeedUrl() {
    return VIDEO_FEED_URL + '?t=' + Date.now();
  }

  /**
   * Wire automatic reload when an <img> fails or stalls.
   * opts:
   *   baseUrl  - stream base (default /video_feed)
   *   maxRetries - default 12
   *   minDelayMs - default 800
   *   maxDelayMs - default 8000
   *   isStream   - if true, always use cache-busted baseUrl (MJPEG)
   *   label      - for title tooltip
   *   failEl     - optional element shown after retries exhausted (honest fail)
   */
  function wireImageRetry(img, opts) {
    if (!img || img._ugvRetryWired) return img;
    opts = opts || {};
    var maxRetries = opts.maxRetries != null ? opts.maxRetries : 12;
    var minDelay = opts.minDelayMs != null ? opts.minDelayMs : 800;
    var maxDelay = opts.maxDelayMs != null ? opts.maxDelayMs : 8000;
    var isStream = !!opts.isStream;
    var baseUrl = opts.baseUrl || VIDEO_FEED_URL;
    var key = img.id || ('img_' + Math.random().toString(36).slice(2));
    img._ugvRetryWired = true;
    img._ugvRetryCount = 0;
    img._ugvLastGoodSrc = '';
    img._ugvBaseUrl = baseUrl;
    img._ugvIsStream = isStream;
    img._ugvPaused = !!img._ugvPaused;
    img._ugvRetryKey = key;
    img._ugvFailEl = opts.failEl || null;

    function clearTimer() {
      if (imgRetryTimers[key]) {
        clearTimeout(imgRetryTimers[key]);
        delete imgRetryTimers[key];
      }
    }

    function setFailVisible(on, msg) {
      var el = img._ugvFailEl || opts.failEl;
      if (!el) return;
      if (on) {
        el.hidden = false;
        var m = el.querySelector('.ugv-live-fail-msg');
        if (m && msg) m.textContent = msg;
      } else {
        el.hidden = true;
      }
    }

    function scheduleRetry(reason) {
      if (img._ugvPaused) return;
      if (img._ugvRetryCount >= maxRetries) {
        img.title = (opts.label || 'Image') + ' failed after ' + maxRetries + ' retries';
        img.classList.add('ugv-img-broken');
        setFailVisible(
          true,
          (opts.label || 'Live camera') + ' unavailable — check camera / try Retry'
        );
        return;
      }
      clearTimer();
      var n = img._ugvRetryCount++;
      var delay = Math.min(maxDelay, minDelay * Math.pow(1.45, n));
      img.title = (opts.label || 'Image') + ' reloading… (try ' + (n + 1) + ')';
      imgRetryTimers[key] = setTimeout(function () {
        reloadImage(reason || 'retry');
      }, delay);
    }

    function reloadImage(reason) {
      if (img._ugvPaused && reason !== 'resume' && reason !== 'manual-retry') return;
      img._ugvPaused = false;
      clearTimer();
      setFailVisible(false);
      img.classList.remove('ugv-img-broken');
      var next;
      if (img._ugvIsStream) {
        next = baseUrl + (baseUrl.indexOf('?') >= 0 ? '&' : '?') + 't=' + Date.now() + '&r=' + img._ugvRetryCount;
      } else if (img._ugvLastGoodSrc) {
        // static/data-url: re-apply last good (or pending) source
        next = img._ugvLastGoodSrc;
        // force reload even if same string
        if (img.src === next && next.indexOf('data:') !== 0) {
          next = next.split('#')[0] + '#r=' + Date.now();
        }
      } else if (opts.fallbackUrl) {
        next = opts.fallbackUrl + (opts.fallbackUrl.indexOf('?') >= 0 ? '&' : '?') + 't=' + Date.now();
      } else {
        return;
      }
      try {
        img.src = next;
      } catch (e) {
        scheduleRetry('set-src-error');
      }
    }

    img.addEventListener('load', function () {
      if (img._ugvPaused) return;
      // Streams fire load once connection opens; treat as healthy
      if (img.naturalWidth > 0 || img._ugvIsStream) {
        img._ugvRetryCount = 0;
        clearTimer();
        if (!img._ugvIsStream && img.src) {
          img._ugvLastGoodSrc = img.getAttribute('src') || img.src;
        }
        img.title = opts.label || '';
        img.classList.remove('ugv-img-broken');
        setFailVisible(false);
      }
    });

    img.addEventListener('error', function () {
      if (img._ugvPaused) return;
      // Empty/cleared src while pausing must not thrash retries
      var cur = img.getAttribute('src') || '';
      if (!cur) return;
      img.classList.add('ugv-img-broken');
      scheduleRetry('error');
    });

    img.addEventListener('abort', function () {
      if (img._ugvPaused) return;
      scheduleRetry('abort');
    });

    // Periodic stall check for streams (no frames / zero size)
    if (isStream) {
      setInterval(function () {
        if (document.hidden) return;
        if (!img.isConnected || img._ugvPaused) return;
        // Skip hidden panels (display:none → offsetParent null)
        if (img.offsetParent === null || img.hidden) return;
        if (img.complete && img.naturalWidth === 0 && img.getAttribute('src')) {
          scheduleRetry('stall');
        }
      }, 6000);
    }

    img._ugvReload = reloadImage;
    img._ugvPause = function () {
      img._ugvPaused = true;
      img._ugvRetryCount = 0;
      clearTimer();
      setFailVisible(false);
      img.classList.remove('ugv-img-broken');
      try {
        img.removeAttribute('src');
      } catch (e) {
        try {
          img.src = '';
        } catch (e2) {}
      }
    };
    img._ugvSetSrc = function (src, asStream) {
      if (asStream != null) img._ugvIsStream = !!asStream;
      img._ugvPaused = false;
      img._ugvRetryCount = 0;
      clearTimer();
      setFailVisible(false);
      if (src) {
        img._ugvLastGoodSrc = src;
        img.src = src;
      }
    };
    return img;
  }

  function getRawFeedImg() {
    return document.querySelector(
      '.video img, #video_feed_frame img, .feed_section img'
    );
  }

  function pauseLiveStream(img) {
    if (!img) return;
    if (img._ugvPause) {
      img._ugvPause();
      return;
    }
    img._ugvPaused = true;
    try {
      img.removeAttribute('src');
    } catch (e) {
      try {
        img.src = '';
      } catch (e2) {}
    }
  }

  function resumeLiveStream(img, reason) {
    if (!img) return;
    img._ugvPaused = false;
    if (img._ugvReload) img._ugvReload(reason || 'resume');
    else img.src = videoFeedUrl();
  }

  /**
   * Only one MJPEG consumer at a time: active mode's feed.
   * Hidden panels must not hold /video_feed open (causes blank Chat/Seek).
   */
  function refreshLiveFeeds() {
    var mode = window.ugvAppMode || getActiveMode();
    var chat = $('chat-live-preview');
    var seek = $('seek-live-preview');
    var track = $('track-live-preview');
    var raw = getRawFeedImg();

    if (mode === 'chat') {
      pauseLiveStream(seek);
      pauseLiveStream(track);
      pauseLiveStream(raw);
      resumeLiveStream(chat, 'mode-enter');
    } else if (mode === 'seek') {
      pauseLiveStream(chat);
      pauseLiveStream(track);
      pauseLiveStream(raw);
      resumeLiveStream(seek, 'mode-enter');
    } else if (mode === 'track') {
      pauseLiveStream(chat);
      pauseLiveStream(seek);
      pauseLiveStream(raw);
      resumeLiveStream(track, 'mode-enter');
    } else if (mode === 'loadout') {
      pauseLiveStream(chat);
      pauseLiveStream(seek);
      pauseLiveStream(track);
      pauseLiveStream(raw);
    } else {
      pauseLiveStream(chat);
      pauseLiveStream(seek);
      pauseLiveStream(track);
      if (raw) {
        if (!raw._ugvRetryWired) {
          wireImageRetry(raw, {
            isStream: true,
            baseUrl: VIDEO_FEED_URL,
            label: 'Live camera',
            maxRetries: 24,
          });
        }
        resumeLiveStream(raw, 'mode-enter');
      }
    }
  }

  function getActiveMode() {
    var active = document.querySelector('.ugv-mode-tabs [data-mode].active');
    if (active) return active.getAttribute('data-mode') || 'raw';
    try {
      return localStorage.getItem(STORAGE_KEY) || 'raw';
    } catch (e) {
      return 'raw';
    }
  }

  function setSeekRunningIndicator(running, st) {
    var pill = $('seek-running-pill');
    if (!pill) return;
    if (running) {
      pill.hidden = false;
      var step = st && st.step != null ? st.step : '?';
      var max = st && st.max_steps != null ? st.max_steps : 0;
      var maxLab = max === 0 || max === '0' ? '∞' : String(max);
      pill.textContent = 'Seek running · ' + step + '/' + maxLab;
      pill.setAttribute('aria-hidden', 'false');
    } else {
      pill.hidden = true;
      pill.textContent = 'Seek running';
      pill.setAttribute('aria-hidden', 'true');
    }
  }

  function setMode(mode, opts) {
    opts = opts || {};
    mode = mode || 'raw';
    if (
      mode !== 'raw' &&
      mode !== 'chat' &&
      mode !== 'seek' &&
      mode !== 'track' &&
      mode !== 'loadout'
    ) {
      mode = 'raw';
    }
    var prev = getActiveMode();
    // Leaving Seek while autonomy is running: confirm (or auto-stop when force).
    if (
      prev === 'track' &&
      mode !== 'track' &&
      lastSeenTrackPhase === 'running' &&
      !opts.force
    ) {
      var okT = true;
      try {
        okT = window.confirm('Track is sweeping the camera. Stop Track and switch mode?');
      } catch (e) {
        okT = true;
      }
      if (!okT) return false;
      try { trackStop({ silent: true }); } catch (e2) {}
      lastSeenTrackPhase = 'stopped';
    }
    if (
      prev === 'seek' &&
      mode !== 'seek' &&
      lastSeenSeekPhase === 'running' &&
      !opts.force
    ) {
      var ok = true;
      try {
        ok = window.confirm(
          'Seek is still driving the robot. Stop Seek and switch mode?'
        );
      } catch (e) {
        ok = true;
      }
      if (!ok) {
        // Keep Seek tab selected
        return false;
      }
      // Auto-stop seek before leaving (does not wait for network)
      try {
        seekStop({ silent: true });
      } catch (e) {}
      lastSeenSeekPhase = 'stopped';
      setSeekRunningIndicator(false);
    }
    var panels = {
      raw: $('mode-panel-raw'),
      chat: $('mode-panel-chat'),
      seek: $('mode-panel-seek'),
      track: $('mode-panel-track'),
      loadout: $('mode-panel-loadout'),
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
    window.ugvAppMode = mode;
    // Re-open MJPEG when entering chat/seek (or any mode switch).
    // Loadout has no live feed — pause every MJPEG consumer.
    if (
      mode === 'chat' ||
      mode === 'seek' ||
      mode === 'track' ||
      mode === 'raw' ||
      mode === 'loadout'
    ) {
      refreshLiveFeeds();
    }
    if (mode === 'loadout') {
      window.ugvLoadoutRefresh && window.ugvLoadoutRefresh();
    }
    return true;
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
  var voiceMode = 'off';  // 'off' | 'browser' | 'robot'
  var voiceConfig = { stt_enabled: false, tts_enabled: false };
  var mediaRecorder = null;
  var audioChunks = [];
  var VOICE_MODE_KEY = 'ugv_chat_voice_mode';

  function chatAdd(role, text) {
    var log = $('chat-log');
    if (!log) return;
    var div = document.createElement('div');
    div.className = 'ugv-chat-msg ' + role;
    div.textContent = text;
    log.appendChild(div);
    log.scrollTop = log.scrollHeight;
  }

  function checkVoiceConfig() {
    fetch('/api/voice/config')
      .then(function (r) { return r.json(); })
      .then(function (d) {
        voiceConfig = d;
        var select = $('chat-voice-mode');
        if (!select) return;
        
        var configured = d.stt_enabled || d.tts_enabled;
        var browserOpt = select.querySelector('option[value="browser"]');
        var robotOpt = select.querySelector('option[value="robot"]');
        
        if (configured) {
          if (browserOpt) browserOpt.disabled = false;
          if (robotOpt) robotOpt.disabled = false;
          select.title = 'Voice mode: Off, Browser (mic/speakers), or Robot (device mic/speakers)';
        } else {
          if (browserOpt) browserOpt.disabled = true;
          if (robotOpt) robotOpt.disabled = true;
          select.value = 'off';
          select.title = 'Voice not configured — set UGV_STT_URL and UGV_TTS_URL in .env';
          voiceMode = 'off';
        }
        var voiceBtnEl = $('chat-voice-btn');
        if (voiceBtnEl) voiceBtnEl.hidden = (voiceMode === 'off');
      })
      .catch(function () {
        var select = $('chat-voice-mode');
        if (select) {
          select.value = 'off';
          var browserOpt = select.querySelector('option[value="browser"]');
          var robotOpt = select.querySelector('option[value="robot"]');
          if (browserOpt) browserOpt.disabled = true;
          if (robotOpt) robotOpt.disabled = true;
        }
      });
  }

  function playTTSAudio(text) {
    if (voiceMode === 'off' || !voiceConfig.tts_enabled) return;
    
    if (voiceMode === 'browser') {
      // Browser mode: play audio in browser
      fetch('/api/voice/tts', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: text }),
      })
        .then(function (r) {
          if (!r.ok) throw new Error('TTS failed');
          return r.blob();
        })
        .then(function (audioBlob) {
          var audio = new Audio(URL.createObjectURL(audioBlob));
          audio.play().catch(function (e) {
            console.warn('TTS audio play failed:', e);
          });
        })
        .catch(function (e) {
          console.warn('TTS error:', e);
        });
    } else if (voiceMode === 'robot') {
      // Robot mode: play through robot's speakers
      fetch('/api/voice/robot/speak', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: text }),
      })
        .then(function (r) { return r.json(); })
        .then(function (d) {
          if (!d.success) {
            console.warn('Robot TTS failed:', d.error);
          }
        })
        .catch(function (e) {
          console.warn('Robot TTS error:', e);
        });
    }
  }

  function startVoiceRecording() {
    if (voiceMode === 'off' || !voiceConfig.stt_enabled) return;
    
    if (voiceMode === 'browser') {
      // Browser mode: use browser mic
      audioChunks = [];
      navigator.mediaDevices.getUserMedia({ audio: true })
        .then(function (stream) {
          mediaRecorder = new MediaRecorder(stream);
          mediaRecorder.ondataavailable = function (e) {
            audioChunks.push(e.data);
          };
          mediaRecorder.onstop = function () {
            var audioBlob = new Blob(audioChunks, { type: 'audio/webm' });
            sendBrowserSTT(audioBlob);
            stream.getTracks().forEach(function (track) { track.stop(); });
          };
          mediaRecorder.start();
          chatAdd('sys', 'Recording (browser)… (release to send)');
        })
        .catch(function (e) {
          chatAdd('err', 'Microphone access denied: ' + e.message);
        });
    } else if (voiceMode === 'robot') {
      // Robot mode: use robot's mic
      chatAdd('sys', 'Recording (robot mic)… (5 seconds)');
      fetch('/api/voice/robot/record', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ duration: 5 }),
      })
        .then(function (r) { return r.json(); })
        .then(function (d) {
          if (!d.success) {
            chatAdd('err', 'Robot STT failed: ' + (d.error || 'unknown'));
            return;
          }
          var text = d.text || '';
          if (!text) {
            chatAdd('sys', 'No speech detected.');
            return;
          }
          var input = $('chat-input');
          if (input) input.value = text;
          chatSend();
        })
        .catch(function (e) {
          chatAdd('err', 'Robot STT error: ' + e.message);
        });
    }
  }

  function stopVoiceRecording() {
    // Only applies to browser mode
    if (voiceMode === 'browser' && mediaRecorder && mediaRecorder.state !== 'inactive') {
      mediaRecorder.stop();
    }
  }

  function sendBrowserSTT(audioBlob) {
    chatAdd('sys', 'Transcribing…');
    
    fetch('/api/voice/stt', {
      method: 'POST',
      headers: { 'Content-Type': 'audio/webm' },
      body: audioBlob,
    })
      .then(function (r) { return r.json(); })
      .then(function (d) {
        if (!d.success) {
          chatAdd('err', 'STT failed: ' + (d.error || 'unknown'));
          return;
        }
        var text = d.text || '';
        if (!text) {
          chatAdd('sys', 'No speech detected.');
          return;
        }
        var input = $('chat-input');
        if (input) input.value = text;
        chatSend();
      })
      .catch(function (e) {
        chatAdd('err', 'STT error: ' + e.message);
      });
  }

  function chatSend() {
    var input = $('chat-input');
    var btn = $('chat-send-btn');
    if (!input || !btn) return;
    var message = (input.value || '').trim();
    if (!message) return;
    var attach = $('chat-attach') && $('chat-attach').checked;
    var stillImg = $('chat-snap-preview');
    var stillUrl = '';
    if (
      attach &&
      stillImg &&
      !stillImg.hidden &&
      stillImg.src &&
      stillImg.src.indexOf('data:image') === 0
    ) {
      stillUrl = stillImg.src;
    }
    btn.disabled = true;
    chatAdd('user', message);
    input.value = '';
    chatAdd('sys', attach ? (stillUrl ? 'Thinking with grabbed still…' : 'Thinking (live still)…') : 'Thinking…');
    var payload = {
      message: message,
      history: chatHistory,
      attach_snapshot: !!attach,
    };
    if (stillUrl) payload.snapshot_data_url = stillUrl;
    fetch('/api/ai/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
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
          var snapImg = $('chat-snap-preview');
          snapImg.hidden = false;
          snapImg.src = d.snapshot_data_url;
          var slot = $('chat-still-slot');
          if (slot) slot.classList.remove('is-empty');
        }
        var reply = d.reply || '(empty)';
        chatAdd('ai', reply);
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
        chatHistory.push({ role: 'assistant', content: reply });
        if (chatHistory.length > 24) chatHistory = chatHistory.slice(-24);
        
        // Play TTS for reply if voice mode is enabled (browser or robot)
        if (voiceMode !== 'off' && reply && reply !== '(empty)') {
          playTTSAudio(reply);
        }
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
    var voiceSelect = $('chat-voice-mode');
    var voiceBtn = $('chat-voice-btn');
    
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
              var snapImg2 = $('chat-snap-preview');
              snapImg2.hidden = false;
              snapImg2.src = d.data_url;
              var slot2 = $('chat-still-slot');
              if (slot2) slot2.classList.remove('is-empty');
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
    
    // Voice mode select
    if (voiceSelect) {
      // Restore saved mode
      try {
        var saved = localStorage.getItem(VOICE_MODE_KEY) || 'off';
        if (saved === 'browser' || saved === 'robot' || saved === 'off') {
          voiceSelect.value = saved;
          voiceMode = saved;
        }
      } catch (e) {}
      if (voiceBtn) {
        voiceBtn.hidden = (voiceMode === 'off');
        if (voiceMode === 'browser') {
          voiceBtn.textContent = '🎤 Record (hold)';
          voiceBtn.title = 'Hold to record from browser mic';
        } else if (voiceMode === 'robot') {
          voiceBtn.textContent = '🎤 Record (5s)';
          voiceBtn.title = 'Record 5 seconds from robot mic';
        }
      }

      voiceSelect.addEventListener('change', function () {
        var newMode = voiceSelect.value;
        voiceMode = newMode;
        try {
          localStorage.setItem(VOICE_MODE_KEY, newMode);
        } catch (e) {}
        
        if (voiceBtn) {
          voiceBtn.hidden = (newMode === 'off');
          if (newMode === 'browser') {
            voiceBtn.textContent = '🎤 Record (hold)';
            voiceBtn.title = 'Hold to record from browser mic';
          } else if (newMode === 'robot') {
            voiceBtn.textContent = '🎤 Record (5s)';
            voiceBtn.title = 'Record 5 seconds from robot mic';
          }
        }
        
        if (newMode === 'off') {
          chatAdd('sys', 'Voice disabled.');
        } else if (newMode === 'browser') {
          chatAdd('sys', 'Voice: Browser mode (mic/speakers in browser).');
        } else if (newMode === 'robot') {
          chatAdd('sys', 'Voice: Robot mode (device mic/speakers on the robot).');
        }
      });
    }
    
    // Voice record button
    if (voiceBtn) {
      // For browser mode: hold to record
      voiceBtn.addEventListener('mousedown', function () {
        if (voiceMode === 'browser') startVoiceRecording();
      });
      voiceBtn.addEventListener('touchstart', function (e) {
        e.preventDefault();
        if (voiceMode === 'browser') startVoiceRecording();
      });
      voiceBtn.addEventListener('mouseup', function () {
        if (voiceMode === 'browser') stopVoiceRecording();
      });
      voiceBtn.addEventListener('touchend', function (e) {
        e.preventDefault();
        if (voiceMode === 'browser') stopVoiceRecording();
      });
      voiceBtn.addEventListener('mouseleave', function () {
        if (voiceMode === 'browser') stopVoiceRecording();
      });
      
      // For robot mode: click to start 5s recording
      voiceBtn.addEventListener('click', function () {
        if (voiceMode === 'robot') startVoiceRecording();
      });
    }
    
    // Check voice config on init
    checkVoiceConfig();
    
    chatAdd('sys', 'Chat mode ready. Attach a still when you want vision context.');
  }

  // ---------- Seek panel ----------
  var seekPollTimer = null;
  var SEEK_REFEREE_KEY = 'ugv_seek_referee';
  var SEEK_DRY_KEY = 'ugv_seek_dry_run';
  var lastSeekCheckSeq = 0;
  var lastSeekStep = -1;
  var lastSeekLogSeq = 0;
  var lastSeenSeekPhase = 'idle';
  var lastSeenTrackPhase = 'idle';
  var trackPollTimer = null;
  var lastTrackLogSeq = 0;
  var DETECTOR_LABELS_CACHE = [];
  var seekFireTimer = null;
  var seekHydrated = false;

  function startSeekPolling() {
    if (seekPollTimer) clearInterval(seekPollTimer);
    // Faster poll so pan overlay tracks HW while gimbal is moving
    seekPollTimer = setInterval(pollSeek, 200);
  }

  function stopSeekPolling() {
    if (seekPollTimer) {
      clearInterval(seekPollTimer);
      seekPollTimer = null;
    }
  }

  function setSeekControlsRunning(running) {
    var start = $('seek-start-btn');
    var check = $('seek-check-btn');
    if (start) {
      start.disabled = !!running;
      start.title = running ? 'Seek already running — Stop first' : '';
    }
    if (check) check.disabled = !!running;
    // body class drives sticky log expand + config chrome
    try {
      document.body.classList.toggle('seek-running', !!running);
    } catch (e) {}
    // Lock config while running so UI doesn't imply mid-run retarget
    var lockIds = [
      'seek-mode-detector',
      'seek-mode-detector-llm',
      'seek-mode-llm-vision',
      'seek-goal-select',
      'seek-goal-text',
      'seek-on-found',
      'seek-on-found-tts',
      'seek-llm-scene-nav',
      'seek-llm-nav-interval',
      'seek-multi-image',
      'seek-max-steps',
      'seek-timeout-s',
      'seek-dry-run',
    ];
    lockIds.forEach(function (id) {
      var el = $(id);
      if (el) el.disabled = !!running;
    });
    var cfg = document.querySelector('.ugv-seek-config-card');
    if (cfg) {
      cfg.classList.toggle('is-locked', !!running);
      cfg.title = running ? 'Config locked while Seek is running — Stop to edit' : '';
    }
  }

  function applySeekFormFromStatus(st) {
    if (!st) return;
    var referee = st.referee || 'detector';
    var sceneNav = st.llm_scene_nav !== false && st.llm_scene_nav !== 0;
    var mode = 'detector_llm_nav';
    if (referee === 'llm') mode = 'llm_vision';
    else if (!sceneNav) mode = 'detector';

    var rDet = $('seek-mode-detector');
    var rDetLlm = $('seek-mode-detector-llm');
    var rLlm = $('seek-mode-llm-vision');
    if (mode === 'detector' && rDet) rDet.checked = true;
    else if (mode === 'llm_vision' && rLlm) rLlm.checked = true;
    else if (rDetLlm) rDetLlm.checked = true;
    try {
      localStorage.setItem(SEEK_REFEREE_KEY, mode);
    } catch (e) {}
    syncSeekRefereeUI();

    var goal = st.goal_label || st.goal_text || '';
    if (mode === 'llm_vision') {
      var t = $('seek-goal-text');
      if (t && goal) t.value = goal;
    } else {
      var s = $('seek-goal-select');
      if (s && goal) {
        // select if option exists; else add temporary option
        var found = false;
        for (var i = 0; i < s.options.length; i++) {
          if (s.options[i].value === goal) {
            s.selectedIndex = i;
            found = true;
            break;
          }
        }
        if (!found && goal) {
          var opt = document.createElement('option');
          opt.value = goal;
          opt.textContent = goal;
          opt.selected = true;
          s.appendChild(opt);
        }
      }
    }

    var dryEl = $('seek-dry-run');
    if (dryEl && (st.phase === 'running' || st.phase === 'found' || st.phase === 'timeout')) {
      dryEl.checked = st.dry_run !== false && st.dry_run !== 0;
    }

    var onFoundSel = $('seek-on-found');
    if (onFoundSel && st.on_found) {
      onFoundSel.value = st.on_found === 'tts' ? 'tts' : 'none';
      syncSeekOnFoundUI();
    }
    var ttsInp = $('seek-on-found-tts');
    if (ttsInp && st.on_found_tts) ttsInp.value = st.on_found_tts;

    var sceneCb = $('seek-llm-scene-nav');
    if (sceneCb) sceneCb.checked = !!sceneNav;
    var intervalInp = $('seek-llm-nav-interval');
    if (intervalInp && st.llm_nav_interval) {
      intervalInp.value = String(st.llm_nav_interval);
    }
    var maxInp = $('seek-max-steps');
    if (maxInp && st.max_steps != null) {
      maxInp.value = String(st.max_steps);
    }
    var toInp = $('seek-timeout-s');
    if (toInp && st.timeout_s != null) {
      toInp.value = String(st.timeout_s);
    }
  }

  function replaySeekLogFromStatus(st, opts) {
    opts = opts || {};
    var logEl = $('seek-log');
    if (opts.clear && logEl) logEl.innerHTML = '';
    // Replay entire server ring buffer
    lastSeekLogSeq = 0;
    drainSeekEventLog(st || {});
  }

  function hydrateSeekFromServer(opts) {
    opts = opts || {};
    var soft = !!opts.soft; // tab-focus: don't wipe log / re-banner
    return fetch('/api/ai/seek/status')
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        var st = (d && d.status) || {};
        var phase = st.phase || 'idle';
        var prevPhase = lastSeenSeekPhase;
        lastSeekCheckSeq = Math.max(lastSeekCheckSeq, st.last_check_seq || 0);
        if (st.step !== undefined) lastSeekStep = st.step;

        if (phase === 'idle' && !st.started_at && !st.finished_at) {
          lastSeenSeekPhase = phase;
          setSeekControlsRunning(false);
          seekHydrated = true;
          return st;
        }

        applySeekFormFromStatus(st);

        if (!soft) {
          // Full page load: rebuild log + status from server ring buffer
          replaySeekLogFromStatus(st, { clear: true });
        } else {
          // Catch up any new events only
          drainSeekEventLog(st);
        }

        if (phase === 'running') {
          if (!soft) {
            setMode('seek');
            seekLog(
              '↻ Resumed live seek after refresh · goal=' +
                (st.goal_label || st.goal_text || '?') +
                ' · step ' +
                (st.step || 0) +
                (st.elapsed_s != null ? ' · ' + st.elapsed_s + 's elapsed' : ''),
              'start'
            );
          }
          setSeekControlsRunning(true);
          renderSeekStatus(st);
          if (!seekPollTimer) startSeekPolling();
        } else {
          setSeekControlsRunning(false);
          renderSeekStatus(st);
          if (!soft) {
            seekLog(
              'Last seek: ' +
                phase +
                (st.message ? ' — ' + st.message : '') +
                (st.goal_label ? ' · goal=' + st.goal_label : ''),
              phase === 'found' ? 'found' : phase === 'failed' ? 'warn' : 'sys'
            );
          } else if (prevPhase === 'running' && phase !== 'running') {
            // ended while tab was hidden
            var endKind = phase === 'found' ? 'found' : phase === 'failed' ? 'warn' : 'sys';
            seekLog('Seek ended: ' + phase + ' — ' + (st.message || ''), endKind);
            stopSeekPolling();
          }
        }
        lastSeenSeekPhase = phase;
        seekHydrated = true;
        return st;
      })
      .catch(function () {
        seekHydrated = true;
        return null;
      });
  }

  function seekLog(msg, kind) {
    var log = $('seek-log');
    if (!log) return;
    var div = document.createElement('div');
    var k = (kind || 'sys').toLowerCase();
    div.className = 'ugv-chat-msg sys ugv-seek-log-' + k;
    if (k === 'found' || k === 'tts') div.style.color = '#00e676';
    else if (k === 'nav') div.style.color = '#80d8ff';
    else if (k === 'drive') div.style.color = '#ffd54f';
    else if (k === 'detect') div.style.color = '#ce93d8';
    else if (k === 'warn' || k === 'error') div.style.color = '#ff8a80';
    div.textContent = msg;
    log.appendChild(div);
    // Cap DOM lines so long seeks stay snappy
    while (log.childNodes.length > 200) {
      log.removeChild(log.firstChild);
    }
    log.scrollTop = log.scrollHeight;
  }

  function drainSeekEventLog(st) {
    var events = (st && st.event_log) || [];
    if (!events.length) return;
    for (var i = 0; i < events.length; i++) {
      var ev = events[i];
      if (!ev || typeof ev.seq !== 'number') continue;
      if (ev.seq <= lastSeekLogSeq) continue;
      lastSeekLogSeq = ev.seq;
      var prefix = ev.kind ? '[' + String(ev.kind).toUpperCase() + '] ' : '';
      seekLog(prefix + (ev.text || ''), ev.kind || 'info');
    }
  }

  function getSelectedSeekMode() {
    var rDetector = $('seek-mode-detector');
    var rLlmVision = $('seek-mode-llm-vision');
    if (rDetector && rDetector.checked) return 'detector';
    if (rLlmVision && rLlmVision.checked) return 'llm_vision';
    return 'detector_llm_nav'; // Default: Object Detector + LLM Nav
  }

  function getSeekReferee() {
    return getSelectedSeekMode() === 'llm_vision' ? 'llm' : 'detector';
  }

  function getSeekGoal() {
    if (getSelectedSeekMode() === 'llm_vision') {
      var t = $('seek-goal-text');
      return (t && t.value) || '';
    }
    var s = $('seek-goal-select');
    return (s && s.value) || '';
  }

  var SEEK_CAMERA_HINTS = {
    detector:
      'Mode <strong>a</strong> (classic detector): each step L / front / R stills. ' +
      'MobileNet-SSD decides “found”. Heuristic nav (no LLM). Works without an API key.',
    detector_llm_nav:
      'Mode <strong>b</strong> (detector + LLM nav): each step a <strong>front</strong> still → forced JSON ' +
      '(short hop / long hop / subject). Sides only if front is blocked, then rotate and recentre. ' +
      'MobileNet-SSD referee decides “found”.',
    llm_vision:
      'Mode <strong>c</strong> (LLM vision): free-text goal. Same front-first hops as b, but the ' +
      'vision LLM also judges “found”. Uncheck scene nav to scan without driving.'
  };

  function syncSeekCameraHint() {
    var el = $('seek-hint-camera');
    if (!el) return;
    var mode = getSelectedSeekMode();
    var html = SEEK_CAMERA_HINTS[mode] || SEEK_CAMERA_HINTS.detector_llm_nav;
    el.innerHTML = html;
  }

  function syncSeekRefereeUI() {
    var mode = getSelectedSeekMode();
    var ref = getSeekReferee();
    var detWrap = $('seek-goal-detector-wrap');
    var llmWrap = $('seek-goal-llm-wrap');

    if (detWrap) detWrap.hidden = (mode === 'llm_vision');
    if (llmWrap) llmWrap.hidden = (mode !== 'llm_vision');

    // Mode a: scene-nav always off and checkbox disabled (payload forces false).
    // Modes b/c: checkbox is live and included in start payload.
    var sceneCb = $('seek-llm-scene-nav');
    var intervalInp = $('seek-llm-nav-interval');
    if (sceneCb) {
      if (mode === 'detector') {
        sceneCb.checked = false;
        sceneCb.disabled = true;
      } else {
        sceneCb.disabled = false;
      }
    }
    if (intervalInp) {
      intervalInp.disabled = mode === 'detector' || (sceneCb && !sceneCb.checked);
    }

    syncSeekCameraHint();

    try {
      localStorage.setItem(SEEK_REFEREE_KEY, mode);
    } catch (e) {}

    if (!$('seek-detector-bar') || ($('seek-detector-bar').classList.contains('is-running'))) return;
    setDetectorBar('idle', ref === 'llm' ? 'Judge: idle' : 'Detector: idle', '');
  }

  function populateDetectorLabels(labels) {
    var sel = $('seek-goal-select');
    if (!sel) return;
    var preferred = 'person';
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
    if (st && st.last_nav && st.last_nav.action) {
      if (st.last_nav.summary) {
        meta.push(st.last_nav.summary);
      } else {
        var nd = st.last_nav.drive_distance ? '/' + st.last_nav.drive_distance : '';
        meta.push('nav ' + st.last_nav.action + nd);
      }
    }
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

  function formatPanDeg(deg) {
    if (deg == null || deg === '' || isNaN(Number(deg))) return '—';
    var n = Number(deg);
    var s = (n >= 0 ? '+' : '') + n.toFixed(Math.abs(n) % 1 === 0 ? 0 : 1);
    return s + '°';
  }

  function panLabelFromDeg(deg) {
    if (deg == null || isNaN(Number(deg))) return '—';
    var p = Number(deg);
    if (Math.abs(p) < 15) return 'FRONT';
    if (p <= -90) return 'REAR-L';
    if (p >= 90) return 'REAR-R';
    if (p < 0) return 'LEFT';
    return 'RIGHT';
  }

  function easeInOut(t) {
    return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
  }

  /** Duration ms for a pan span at the real gimbal rate (~54°/s). */
  function panDurationMs(fromDeg, toDeg) {
    var span = Math.abs(Number(toDeg) - Number(fromDeg));
    if (!isFinite(span) || span < 0.5) return 0;
    // Match physical pan; small pad so UI finishes with the servo, not before
    var ms = (span / PAN_EST_DPS) * 1000 * 1.05;
    return Math.max(180, Math.min(5500, Math.round(ms)));
  }

  /** Start or retarget needle animation toward toDeg from current needle. */
  function startPanAnim(toDeg) {
    toDeg = Number(toDeg);
    if (isNaN(toDeg)) return;
    var from = panAnim.lastNeedle;
    // Already at target
    if (Math.abs(toDeg - from) < 0.8 && !panAnim.active) {
      panAnim.lastNeedle = toDeg;
      panAnim.lastCmd = toDeg;
      panAnim.to = toDeg;
      return;
    }
    // Same target already animating — keep going
    if (panAnim.active && Math.abs(panAnim.to - toDeg) < 1.5) {
      return;
    }
    panAnim.from = from;
    panAnim.to = toDeg;
    panAnim.startMs = Date.now();
    panAnim.durMs = panDurationMs(from, toDeg);
    panAnim.active = panAnim.durMs > 0;
    panAnim.lastCmd = toDeg;
    if (panAnim.active && !panAnim.raf) {
      panAnim.raf = requestAnimationFrame(tickPanOverlayAnim);
    }
    if (!panAnim.active) {
      panAnim.lastNeedle = toDeg;
    }
  }

  /** Current animated angle (call every frame while active). */
  function samplePanAnim() {
    if (!panAnim.active) return panAnim.lastNeedle;
    if (panAnim.durMs <= 0) {
      panAnim.active = false;
      panAnim.lastNeedle = panAnim.to;
      return panAnim.to;
    }
    var t = (Date.now() - panAnim.startMs) / panAnim.durMs;
    if (t >= 1) {
      panAnim.active = false;
      panAnim.lastNeedle = panAnim.to;
      return panAnim.to;
    }
    // Mild ease so it doesn't look robotic, still finishes in panDurationMs
    var e = easeInOut(Math.min(1, Math.max(0, t)));
    var a = panAnim.from + (panAnim.to - panAnim.from) * e;
    panAnim.lastNeedle = a;
    return a;
  }

  function paintPanNeedle(showDeg, settled, cmd, hw, isEst) {
    var labelEl = $('seek-pan-label');
    var degEl = $('seek-pan-deg');
    var hwEl = $('seek-pan-hw');
    var needle = $('seek-pan-needle');
    var overlay = $('seek-pan-overlay');
    if (!overlay) return;

    var label = panLabelFromDeg(showDeg);
    var targetLabel = panLabelFromDeg(cmd);

    if (labelEl) {
      if (!settled && cmd != null && Math.abs(Number(cmd) - showDeg) > 8) {
        labelEl.textContent = (label || '—') + ' → ' + (targetLabel || '');
      } else {
        labelEl.textContent = targetLabel || label || '—';
      }
    }
    if (degEl) degEl.textContent = formatPanDeg(showDeg);
    if (hwEl) {
      var bits = [];
      if (!settled && cmd != null) bits.push('cmd ' + formatPanDeg(cmd));
      if (hw != null && !isNaN(Number(hw))) bits.push('hw ' + formatPanDeg(hw));
      if (isEst && !settled) bits.push('anim');
      if (settled) bits.push('ok');
      // Show remaining anim time while panning
      if (panAnim.active && panAnim.durMs > 0) {
        var left = Math.max(0, panAnim.durMs - (Date.now() - panAnim.startMs));
        bits.push((left / 1000).toFixed(1) + 's');
      }
      hwEl.textContent = bits.join(' · ');
    }
    if (needle && showDeg != null && !isNaN(Number(showDeg))) {
      needle.style.transition = 'none';
      needle.style.transform = 'rotate(' + Number(showDeg) + 'deg)';
    }
    overlay.classList.toggle('is-settling', !settled || panAnim.active);
    overlay.classList.toggle(
      'is-rear',
      label === 'REAR-L' ||
        label === 'REAR-R' ||
        targetLabel === 'REAR-L' ||
        targetLabel === 'REAR-R'
    );
  }

  function tickPanOverlayAnim() {
    if (!panAnim.active) {
      panAnim.raf = null;
      paintPanNeedle(
        panAnim.lastNeedle,
        !panAnim.settling,
        panAnim.lastCmd,
        null,
        true
      );
      return;
    }
    var a = samplePanAnim();
    paintPanNeedle(a, false, panAnim.lastCmd, null, true);
    if (panAnim.active) {
      panAnim.raf = requestAnimationFrame(tickPanOverlayAnim);
    } else {
      panAnim.raf = null;
      paintPanNeedle(panAnim.lastNeedle, !panAnim.settling, panAnim.lastCmd, null, false);
    }
  }

  /**
   * Pan overlay — status.cam_aim is SoT for target; needle animates to cmd
   * at ~54°/s (same rate as physical pan) so motion matches pan time.
   */
  function renderSeekPanOverlay(st) {
    st = st || {};
    var overlay = $('seek-pan-overlay');
    if (!overlay) return;

    var aim = st.cam_aim || null;
    if (!aim && st.cam_pan_deg != null) {
      aim = {
        cmd: st.cam_pan_deg,
        live: st.cam_pan_live_deg != null ? st.cam_pan_live_deg : st.cam_pan_deg,
        hw: st.cam_pan_hw_deg,
        settled: st.cam_pan_settled,
      };
    }
    if (!aim || aim.cmd == null) {
      overlay.classList.remove('is-settling');
      return;
    }

    var cmd = Number(aim.cmd);
    var hw = aim.hw != null ? Number(aim.hw) : null;
    var settled = aim.settled === true;
    panAnim.settling = !settled;

    if (settled) {
      // Arrived — ease remaining gap quickly if any, else snap
      if (Math.abs(panAnim.lastNeedle - cmd) > 2) {
        // Short finish anim (~0.25s) so it doesn't jump
        panAnim.from = panAnim.lastNeedle;
        panAnim.to = cmd;
        panAnim.startMs = Date.now();
        panAnim.durMs = Math.min(280, panDurationMs(panAnim.from, cmd) || 200);
        panAnim.active = true;
        panAnim.lastCmd = cmd;
        if (!panAnim.raf) panAnim.raf = requestAnimationFrame(tickPanOverlayAnim);
      } else {
        panAnim.active = false;
        panAnim.lastCmd = cmd;
        panAnim.lastNeedle = cmd;
        paintPanNeedle(cmd, true, cmd, hw, false);
      }
      return;
    }

    // New command while panning: animate needle to cmd over physical pan time
    startPanAnim(cmd);
    if (panAnim.active) {
      if (!panAnim.raf) panAnim.raf = requestAnimationFrame(tickPanOverlayAnim);
      // paint current frame immediately (rAF will continue)
      paintPanNeedle(samplePanAnim(), false, cmd, hw, true);
    } else {
      paintPanNeedle(cmd, false, cmd, hw, false);
    }
  }

  function renderSeekPanorama(st) {
    var imgEl = $('seek-pano-img');
    var cardEl = $('seek-panorama-card');
    var badgeEl = $('seek-pano-badge');
    var dataUrl = st.panorama_data_url;
    var hasTarget = false;
    if (Array.isArray(st.last_views)) {
      hasTarget = st.last_views.some(function (v) { return v.has_target; });
    }
    if (st.last_detection && st.last_detection.found) {
      hasTarget = true;
    }

    if (imgEl) {
      if (!imgEl._ugvRetryWired) {
        wireImageRetry(imgEl, {
          isStream: false,
          label: 'Panorama',
          maxRetries: 6,
          minDelayMs: 400,
        });
      }
      if (dataUrl) {
        lastSeekPanoDataUrl = dataUrl;
        // Only re-assign if changed — avoids flicker; retry handler re-applies on error
        if (imgEl._ugvLastGoodSrc !== dataUrl || imgEl.naturalWidth === 0) {
          if (imgEl._ugvSetSrc) imgEl._ugvSetSrc(dataUrl, false);
          else imgEl.src = dataUrl;
        }
        imgEl.style.display = 'block';
      } else if (lastSeekPanoDataUrl && (!imgEl.src || imgEl.naturalWidth === 0)) {
        // Status lost the pano blob briefly — re-show last good frame
        if (imgEl._ugvSetSrc) imgEl._ugvSetSrc(lastSeekPanoDataUrl, false);
        else imgEl.src = lastSeekPanoDataUrl;
        imgEl.style.display = 'block';
      }
    }
    if (cardEl) {
      if (hasTarget) {
        cardEl.classList.add('has-target');
      } else {
        cardEl.classList.remove('has-target');
      }
    }
    if (badgeEl) {
      badgeEl.hidden = !hasTarget;
    }
  }

  function renderSeekStatus(st, opts) {
    st = st || {};
    opts = opts || {};
    var fired = false;
    var curSeq = st.last_check_seq || 0;
    var curStep = st.step !== undefined ? st.step : -1;
    if (
      curSeq > 0 &&
      (curSeq > lastSeekCheckSeq ||
        (curSeq === lastSeekCheckSeq && curStep > lastSeekStep))
    ) {
      lastSeekCheckSeq = curSeq;
      lastSeekStep = curStep;
      fired = true;
    }
    var phase = st.phase || 'idle';
    var el = $('seek-status');
    if (!el) return;
    var cls = 'phase-' + phase;
    var ref = st.referee || 'detector';
    var det = st.last_detection;
    var nav = st.last_nav || {};
    var maxSteps = st.max_steps;
    var maxLab =
      maxSteps === 0 || maxSteps === '0' ? '∞' : maxSteps != null ? String(maxSteps) : '—';
    var stepNum = st.step || 0;
    var stepsLeft =
      maxSteps && maxSteps !== 0 && maxSteps !== '0'
        ? Math.max(0, Number(maxSteps) - Number(stepNum))
        : null;
    var timeLeftLab = '';
    if (st.timeout_s && Number(st.timeout_s) > 0 && st.started_at) {
      var elapsed = Date.now() / 1000 - Number(st.started_at);
      var left = Math.max(0, Number(st.timeout_s) - elapsed);
      timeLeftLab = ' · time left ~' + Math.round(left) + 's';
    } else if (st.timeout_s && Number(st.timeout_s) > 0) {
      timeLeftLab = ' · timeout ' + st.timeout_s + 's';
    }
    var lines = [
      'Phase: ' + phase.toUpperCase(),
      'Referee: ' + ref,
      'Goal: ' + (st.goal_label || st.goal_text || '—'),
      'Step: ' +
        stepNum +
        ' / ' +
        maxLab +
        (stepsLeft != null ? ' (' + stepsLeft + ' left)' : '') +
        timeLeftLab,
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
        var raw = det.raw_detections || [];
        lines.push(
          'Detector found: ' +
            !!det.found +
            ' | matches: ' +
            (det.match_count || 0) +
            (raw.length ? ' | saw: ' + raw.join(', ') : '') +
            (!raw.length && det.labels_found
              ? ' | labels: ' + JSON.stringify(det.labels_found)
              : '')
        );
      }
    }
    if (nav && nav.action) {
      var navLine =
        'Nav: ' +
        (nav.summary ||
          nav.action + '/' + (nav.drive_distance || '?') +
          (nav.magnitude ? ' ' + nav.magnitude : ''));
      if (nav.drive_distance) {
        navLine += ' · tier=' + nav.drive_distance;
      }
      if (nav.turn_deg) navLine += ' · ~' + nav.turn_deg + '°';
      if (nav.duration_ms && !nav.turn_deg) navLine += ' · ' + nav.duration_ms + 'ms';
      if (nav.source) navLine += ' [' + nav.source + ']';
      if (nav.obstacle_range) navLine += ' · obstacle=' + nav.obstacle_range;
      if (nav.open_side) navLine += ' · open=' + nav.open_side;
      if (nav.path_clear_forward === false) navLine += ' · path BLOCKED';
      if (nav.stuck) navLine += ' · STUCK';
      if (nav.safety_override) navLine += ' · SAFETY';
      if (nav.reason) navLine += ' — ' + String(nav.reason).slice(0, 120);
      lines.push(navLine);
    }
    if (st.on_found_phrase) {
      lines.push(
        'TTS: ' +
          (st.on_found_done ? 'spoken' : 'pending/failed') +
          ' — “' +
          st.on_found_phrase +
          '”' +
          (st.on_found_error ? ' ERR: ' + st.on_found_error : '')
      );
    }
    if (st.cam_aim && st.cam_aim.cmd != null) {
      var aim = st.cam_aim;
      lines.push(
        'Cam pan: ' +
          (aim.label || panLabelFromDeg(aim.cmd)) +
          ' ' +
          formatPanDeg(aim.cmd) +
          (aim.settled === false ? ' (moving)' : '') +
          (aim.hw != null ? ' (hw ' + formatPanDeg(aim.hw) + ')' : '')
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

    updateDetectorBar(st, opts);
    renderSeekPanorama(st);
    renderSeekPanOverlay(st);
    drainSeekEventLog(st);

    if (fired && phase === 'running') {
      pulseDetectorFire(st);
    }
  }

  function pollSeek() {
    fetch('/api/ai/seek/status')
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        var st = (d && d.status) || {};
        var phase = st.phase || 'idle';
        renderSeekStatus(st);
        setSeekControlsRunning(phase === 'running');
        setSeekRunningIndicator(phase === 'running', st);

        // Only announce end on transition out of running (not on every poll)
        if (lastSeenSeekPhase === 'running' && phase !== 'running') {
          stopSeekPolling();
          drainSeekEventLog(st);
          var endKind = phase === 'found' ? 'found' : phase === 'failed' ? 'warn' : 'sys';
          seekLog('Seek ended: ' + phase + ' — ' + (st.message || ''), endKind);
          if (phase === 'found' && st.on_found_phrase) {
            seekLog(
              'Announcement: ' +
                (st.on_found_done ? 'OK' : 'FAILED') +
                ' — “' +
                st.on_found_phrase +
                '”' +
                (st.on_found_error ? ' (' + st.on_found_error + ')' : ''),
              'tts'
            );
          }
          updateDetectorBar(st);
          setSeekRunningIndicator(false);
        } else if (phase === 'running' && !seekPollTimer) {
          // Server still running but poll was lost (e.g. tab background) — resume
          startSeekPolling();
        }
        lastSeenSeekPhase = phase;
      })
      .catch(function () {});
  }

  function getSeekOnFound() {
    var sel = $('seek-on-found');
    return (sel && sel.value) || 'tts';
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

  // Finite pilot defaults — match server DEFAULT_SEEK_* (0 still = unlimited if chosen)
  var DEFAULT_UI_SEEK_MAX_STEPS = 30;
  var DEFAULT_UI_SEEK_TIMEOUT_S = 300;

  function getSeekMaxSteps() {
    var inp = $('seek-max-steps');
    if (!inp) return DEFAULT_UI_SEEK_MAX_STEPS;
    var v = parseInt(inp.value, 10);
    if (isNaN(v) || v < 0) return DEFAULT_UI_SEEK_MAX_STEPS;
    return v; // 0 = unlimited (explicit)
  }

  function getSeekTimeoutS() {
    var inp = $('seek-timeout-s');
    if (!inp) return DEFAULT_UI_SEEK_TIMEOUT_S;
    var v = parseFloat(inp.value);
    if (isNaN(v) || v < 0) return DEFAULT_UI_SEEK_TIMEOUT_S;
    return v; // 0 = no limit (explicit)
  }

  function getLlmSceneNavEnabled(mode) {
    // Mode a (detector-only): always off. Modes b/c: honor the checkbox.
    if (mode === 'detector') return false;
    var sceneCb = $('seek-llm-scene-nav');
    if (sceneCb) return !!sceneCb.checked;
    return mode !== 'detector';
  }

  function seekStart() {
    var mode = getSelectedSeekMode();
    var goal = getSeekGoal();
    if (!(goal || '').trim()) {
      seekLog('Start blocked: goal is empty', 'warn');
      return;
    }
    var referee = (mode === 'llm_vision') ? 'llm' : 'detector';
    var llmSceneNav = getLlmSceneNavEnabled(mode);
    var onFound = getSeekOnFound();
    var onFoundTts = getSeekOnFoundTts();
    var llmNavInterval = parseInt(($('seek-llm-nav-interval') && $('seek-llm-nav-interval').value) || '10', 10);
    if (isNaN(llmNavInterval) || llmNavInterval < 1) llmNavInterval = 10;
    var seekMultiImage = !!($('seek-multi-image') && $('seek-multi-image').checked);
    var maxSteps = getSeekMaxSteps();
    var timeoutS = getSeekTimeoutS();
    var dryRun = true;
    var dryEl = $('seek-dry-run');
    if (dryEl) dryRun = !!dryEl.checked;
    if (!dryRun) {
      var okLive = window.confirm(
        'Dry run is OFF. Seek will drive the chassis.\n\n'
        + 'Cancel unless you meant a live test.'
      );
      if (!okLive) {
        seekLog('Start cancelled — dry run left off, live drive not confirmed', 'warn');
        return;
      }
    }
    lastSeekCheckSeq = 0;
    lastSeekStep = -1;
    lastSeekLogSeq = 0;
    lastSeenSeekPhase = 'running';
    var logEl = $('seek-log');
    if (logEl) logEl.innerHTML = '';
    seekLog(
      'Starting seek [' + mode + '] (' +
        referee +
        ') for: ' +
        goal +
        ' · scene nav: ' + (llmSceneNav ? 'on' : 'disabled') +
        ' · limits: ' + (maxSteps === 0 ? '∞ steps' : maxSteps + ' steps') +
        ' / ' + (timeoutS === 0 ? 'no timeout' : timeoutS + 's') +
        ' · ' + (dryRun ? 'DRY-RUN no drive' : 'LIVE DRIVE') +
        ' · upon found: ' +
        (onFound === 'tts' ? 'TTS “' + onFoundTts + '”' : 'do nothing'),
      'start'
    );
    setDetectorBar(
      'running',
      (referee === 'llm' ? 'Judge' : 'Detector') + ': starting…',
      goal
    );
    setSeekControlsRunning(true);
    setSeekRunningIndicator(true, { step: 0, max_steps: maxSteps });
    fetch('/api/ai/seek/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        goal: goal,
        referee: referee,
        max_steps: maxSteps,
        timeout_s: timeoutS,
        on_found: onFound,
        on_found_tts: onFoundTts,
        llm_scene_nav: llmSceneNav,
        llm_nav_interval: llmNavInterval,
        seek_multi_image: seekMultiImage,
        dry_run: dryRun,
        confirm_live: !dryRun,
      }),
    })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (!d.success) {
          seekLog('Start failed: ' + (d.error || 'unknown'));
          setDetectorBar('idle', 'Detector: idle', '');
          setSeekControlsRunning(false);
          setSeekRunningIndicator(false);
          lastSeenSeekPhase = 'idle';
          return;
        }
        // If server already had a run, adopt its log after start response
        var st = d.status || {};
        lastSeenSeekPhase = st.phase || 'running';
        renderSeekStatus(st);
        drainSeekEventLog(st);
        startSeekPolling();
        pollSeek();
      })
      .catch(function (e) {
        seekLog(String(e.message || e));
        setDetectorBar('idle', 'Detector: idle', '');
        setSeekControlsRunning(false);
        setSeekRunningIndicator(false);
        lastSeenSeekPhase = 'idle';
      });
  }

  function seekStop(opts) {
    opts = opts || {};
    if (!opts.silent) {
      seekLog('Stop requested', 'warn');
    }
    // Prefer global emergency STOP (zeros + lock clear + seek cancel)
    if (typeof window.ugvEmergencyStop === 'function' && !opts.seekOnly) {
      window.ugvEmergencyStop();
      return;
    }
    fetch('/api/ai/seek/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        var st = (d && d.status) || {};
        renderSeekStatus(st);
        setSeekRunningIndicator(st.phase === 'running', st);
        // Keep polling until phase leaves running so end message + TTS status appear
        if (!seekPollTimer && st.phase === 'running') startSeekPolling();
      })
      .catch(function (e) {
        if (!opts.silent) seekLog(String(e.message || e));
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

    var radios = document.querySelectorAll('input[name="seek-mode-type"]');
    radios.forEach(function (r) {
      r.addEventListener('change', syncSeekRefereeUI);
    });
    var sceneCbInit = $('seek-llm-scene-nav');
    if (sceneCbInit) {
      sceneCbInit.addEventListener('change', function () {
        syncSeekRefereeUI();
      });
    }
    var onFoundSel = $('seek-on-found');
    if (onFoundSel) onFoundSel.addEventListener('change', syncSeekOnFoundUI);
    syncSeekOnFoundUI();
    try {
      var saved = localStorage.getItem(SEEK_REFEREE_KEY);
      if (saved === 'detector' && $('seek-mode-detector')) $('seek-mode-detector').checked = true;
      if (saved === 'detector_llm_nav' && $('seek-mode-detector-llm')) $('seek-mode-detector-llm').checked = true;
      if (saved === 'llm_vision' && $('seek-mode-llm-vision')) $('seek-mode-llm-vision').checked = true;
    } catch (e) {}
    try {
      var drySaved = localStorage.getItem(SEEK_DRY_KEY);
      var dryEl = $('seek-dry-run');
      if (dryEl) {
        dryEl.checked = drySaved !== '0';
        dryEl.addEventListener('change', function () {
          try {
            localStorage.setItem(SEEK_DRY_KEY, dryEl.checked ? '1' : '0');
          } catch (e2) {}
        });
      }
    } catch (e) {}
    syncSeekRefereeUI();

    // Default labels if API slow/unavailable
    populateDetectorLabels([
      'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair',
      'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant',
      'sheep', 'sofa', 'train', 'tvmonitor',
    ]);

    // Load labels, then rehydrate live seek state from the server (survives refresh)
    fetch('/api/ai/seek/labels')
      .then(function (r) {
        return r.json();
      })
      .then(function (d) {
        if (d && d.success && Array.isArray(d.detector_labels)) {
          populateDetectorLabels(d.detector_labels);
        }
      })
      .catch(function () {})
      .then(function () {
        return hydrateSeekFromServer();
      })
      .then(function (st) {
        if (!st || ((st.phase || 'idle') === 'idle' && !st.started_at)) {
          seekLog(
            'Seek ready. Detector = closed class list; LLM vision = free-text + JSON found true/false.'
          );
        }
      });

    // Re-sync when tab becomes visible again (laptop sleep / background tab)
    document.addEventListener('visibilitychange', function () {
      if (document.visibilityState === 'visible') {
        hydrateSeekFromServer({ soft: true });
        refreshLiveFeeds();
        // Re-apply last panorama if the img went blank while hidden
        var pano = $('seek-pano-img');
        if (pano && lastSeekPanoDataUrl && pano.naturalWidth === 0) {
          if (pano._ugvSetSrc) pano._ugvSetSrc(lastSeekPanoDataUrl, false);
          else pano.src = lastSeekPanoDataUrl;
        }
      }
    });
  }

  function wireManualRetry(btn, img) {
    if (!btn || !img || btn._ugvRetryClick) return;
    btn._ugvRetryClick = true;
    btn.addEventListener('click', function () {
      img._ugvPaused = false;
      img._ugvRetryCount = 0;
      if (img._ugvFailEl) img._ugvFailEl.hidden = true;
      img.classList.remove('ugv-img-broken');
      if (img._ugvReload) img._ugvReload('manual-retry');
      else img.src = videoFeedUrl();
    });
  }

  function initLiveImageRetries() {
    var seekImg = $('seek-live-preview');
    var chatImg = $('chat-live-preview');
    wireImageRetry(seekImg, {
      isStream: true,
      baseUrl: VIDEO_FEED_URL,
      label: 'Seek live camera',
      maxRetries: 24,
      failEl: $('seek-live-fail'),
    });
    wireImageRetry(chatImg, {
      isStream: true,
      baseUrl: VIDEO_FEED_URL,
      label: 'Chat live camera',
      maxRetries: 24,
      failEl: $('chat-live-fail'),
    });
    wireManualRetry($('chat-live-retry'), chatImg);
    wireManualRetry($('seek-live-retry'), seekImg);
    wireImageRetry($('seek-pano-img'), {
      isStream: false,
      label: 'Panorama',
      maxRetries: 8,
      minDelayMs: 400,
    });
    // Raw dashboard MJPEG if present (do not match chat/seek — those are mode-gated)
    var raw = getRawFeedImg();
    if (raw) {
      wireImageRetry(raw, {
        isStream: true,
        baseUrl: VIDEO_FEED_URL,
        label: 'Live camera',
        maxRetries: 24,
      });
    }
  }

  function wirePtzAimBridge() {
    function apply(aim) {
      if (!aim) return;
      renderSeekPanOverlay({ cam_aim: aim });
    }
    try {
      if (typeof socket !== 'undefined' && socket && socket.on) {
        socket.on('ptz_aim', apply);
      }
    } catch (e) { /* socket may load later */ }
    // Idle Seek tab: pick up /api/ptz + /api/status even when Seek is not running
    setInterval(function () {
      if (getActiveMode() !== 'seek') return;
      if (lastSeenSeekPhase === 'running') return;
      fetch('/api/status')
        .then(function (r) { return r.json(); })
        .then(function (d) {
          if (d && d.ptz) apply(d.ptz);
        })
        .catch(function () {});
    }, 700);
  }

  function trackLog(msg, kind) {
    var log = $('track-log');
    if (!log) return;
    var div = document.createElement('div');
    div.className = 'ugv-chat-msg sys ugv-seek-log-' + (kind || 'info');
    div.textContent = msg;
    log.appendChild(div);
    while (log.childNodes.length > 80) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
  }

  function drainTrackLog(st) {
    var evs = (st && st.event_log) || [];
    evs.forEach(function (ev) {
      var seq = ev.seq || 0;
      if (seq > lastTrackLogSeq) {
        lastTrackLogSeq = seq;
        trackLog(ev.text || '', ev.kind || 'info');
      }
    });
  }

  function updateTrackRefereeHint() {
    var inp = $('track-goal-text');
    var hint = $('track-referee-hint');
    if (!hint) return;
    var g = ((inp && inp.value) || '').trim().toLowerCase();
    if (!g) {
      hint.textContent = 'Referee: —';
      return;
    }
    var det = DETECTOR_LABELS_CACHE.indexOf(g) >= 0;
    // aliases handled server-side; hint is best-effort
    hint.textContent = det
      ? 'Referee: MobileNet-SSD detector (' + g + ')'
      : 'Referee: vision LLM (not a VOC class)';
  }

  function renderTrackStatus(st) {
    st = st || {};
    var el = $('track-status');
    var phase = st.phase || 'idle';
    if (el) {
      el.innerHTML =
        'Phase: <span class="phase-' +
        phase +
        '">' +
        phase +
        '</span>' +
        (st.message ? ' — ' + String(st.message).slice(0, 80) : '') +
        (st.locked ? ' · LOCKED' : '');
    }
    var bar = $('track-detector-bar');
    var lab = $('track-detector-label');
    var met = $('track-detector-meta');
    if (lab) {
      lab.textContent =
        (st.referee === 'llm' ? 'LLM' : 'Detector') +
        ': ' +
        (phase === 'running' ? (st.locked ? 'locked' : 'scanning') : phase);
    }
    if (met) {
      met.textContent = st.goal_label
        ? st.goal_label + (st.step ? ' · scan ' + st.step : '')
        : '';
    }
    if (bar) {
      bar.classList.toggle('is-running', phase === 'running');
    }
    drainTrackLog(st);
  }

  function startTrackPolling() {
    if (trackPollTimer) clearInterval(trackPollTimer);
    trackPollTimer = setInterval(pollTrack, 400);
  }
  function stopTrackPolling() {
    if (trackPollTimer) {
      clearInterval(trackPollTimer);
      trackPollTimer = null;
    }
  }

  function pollTrack() {
    fetch('/api/ai/track/status')
      .then(function (r) { return r.json(); })
      .then(function (d) {
        var st = (d && d.status) || {};
        var phase = st.phase || 'idle';
        renderTrackStatus(st);
        if (lastSeenTrackPhase === 'running' && phase !== 'running') {
          stopTrackPolling();
          drainTrackLog(st);
          trackLog('Track ended: ' + phase + ' — ' + (st.message || ''), phase === 'found' ? 'found' : 'sys');
        } else if (phase === 'running' && !trackPollTimer) {
          startTrackPolling();
        }
        lastSeenTrackPhase = phase;
      })
      .catch(function () {});
  }

  function trackStart() {
    var inp = $('track-goal-text');
    var goal = ((inp && inp.value) || '').trim();
    if (!goal) {
      trackLog('Start blocked: goal is empty', 'warn');
      return;
    }
    lastTrackLogSeq = 0;
    var logEl = $('track-log');
    if (logEl) logEl.innerHTML = '';
    var maxSteps = parseInt(($('track-max-steps') && $('track-max-steps').value) || '40', 10);
    var timeoutS = parseFloat(($('track-timeout-s') && $('track-timeout-s').value) || '180');
    trackLog('Starting track for: ' + goal, 'start');
    fetch('/api/ai/track/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ goal: goal, max_steps: maxSteps, timeout_s: timeoutS }),
    })
      .then(function (r) { return r.json(); })
      .then(function (d) {
        if (!d.success) {
          trackLog('Start failed: ' + (d.error || 'unknown'), 'warn');
          return;
        }
        renderTrackStatus(d.status || {});
        startTrackPolling();
        pollTrack();
      })
      .catch(function (e) {
        trackLog(String(e.message || e), 'warn');
      });
  }

  function trackStop(opts) {
    opts = opts || {};
    if (!opts.silent) trackLog('Stop requested', 'warn');
    fetch('/api/ai/track/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
      .then(function (r) { return r.json(); })
      .then(function (d) {
        renderTrackStatus((d && d.status) || { phase: 'stopped' });
      })
      .catch(function (e) {
        if (!opts.silent) trackLog(String(e.message || e), 'warn');
      });
  }

  function trackCheckOnce() {
    var inp = $('track-goal-text');
    var goal = ((inp && inp.value) || '').trim();
    if (!goal) {
      trackLog('Check blocked: goal is empty', 'warn');
      return;
    }
    trackLog('Check once: ' + goal);
    fetch('/api/ai/track/check', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ goal: goal }),
    })
      .then(function (r) { return r.json(); })
      .then(function (d) {
        if (!d.success) {
          trackLog('Check failed: ' + (d.error || ''), 'warn');
          return;
        }
        var c = d.check || {};
        trackLog(
          (d.referee || '') +
            ' · found=' +
            !!c.found +
            (c.raw_detections && c.raw_detections.length
              ? ' · saw ' + c.raw_detections.join(', ')
              : c.reason
                ? ' — ' + c.reason
                : '')
        );
      })
      .catch(function (e) {
        trackLog(String(e.message || e), 'warn');
      });
  }

  function initTrack() {
    var start = $('track-start-btn');
    var stop = $('track-stop-btn');
    var check = $('track-check-btn');
    if (start) start.addEventListener('click', trackStart);
    if (stop) stop.addEventListener('click', function () { trackStop({}); });
    if (check) check.addEventListener('click', trackCheckOnce);
    var goalInp = $('track-goal-text');
    if (goalInp) goalInp.addEventListener('input', updateTrackRefereeHint);
    var trackImg = $('track-live-preview');
    wireImageRetry(trackImg, {
      isStream: true,
      baseUrl: VIDEO_FEED_URL,
      label: 'Live camera',
      maxRetries: 24,
      failEl: $('track-live-fail'),
    });
    wireManualRetry($('track-live-retry'), trackImg);
    fetch('/api/ai/seek/labels')
      .then(function (r) { return r.json(); })
      .then(function (d) {
        var labs = d.detector_labels || d.labels || [];
        DETECTOR_LABELS_CACHE = labs.map(function (x) { return String(x).toLowerCase(); });
        var dl = $('track-goal-list');
        if (dl) {
          dl.innerHTML = '';
          labs.forEach(function (lab) {
            var o = document.createElement('option');
            o.value = lab;
            dl.appendChild(o);
          });
        }
        updateTrackRefereeHint();
      })
      .catch(function () {});
    pollTrack();
  }

  function boot() {
    // Wire retries before first mode enter so refreshLiveFeeds uses _ugvReload
    initLiveImageRetries();
    initModeTabs();
    initChat();
    initSeek();
    initTrack();
    refreshLiveFeeds();
    wirePtzAimBridge();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }

  // After emergency STOP, refresh seek UI / pill from server
  window.ugvOnEmergencyStop = function () {
    lastSeenSeekPhase = 'stopped';
    lastSeenTrackPhase = 'stopped';
    setSeekRunningIndicator(false);
    setSeekControlsRunning(false);
    hydrateSeekFromServer({ soft: true });
    try { pollTrack(); } catch (e) {}
  };

  // export for tests / console / screenshot catalog
  window.ugvSetMode = setMode;
  window.ugvGetActiveMode = getActiveMode;
  window.ugvRefreshLiveFeeds = refreshLiveFeeds;
  window.ugvSeekStop = seekStop;
  window.ugvSetSeekRunningIndicator = setSeekRunningIndicator;
  // Locks Seek config (mode/goal/limits/etc.) while running — Stop to edit
  window.ugvSetSeekControlsRunning = setSeekControlsRunning;
})();
