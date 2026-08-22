/**
 * Hangar / loadout bay: chassis + attachment software profile.
 * GET/POST /api/loadout. Drive signs stay in config.yaml.
 * RoArm USB starts only when attachment=roarm2 is saved.
 */
(function () {
  'use strict';

  var ARM_WANTED_MESSAGE = 'RoArm USB — drivers start on SAVE (attachment=roarm2)';
  var ARM_STARTED_MESSAGE = 'RoArm USB started';
  var API = '/api/loadout';

  var BASE_META = {
    rover: { label: 'Wheeled rover', drive: 'wheels', main_type: 2, robot: 'UGV Rover' },
    beast: { label: 'Tracked beast', drive: 'tracks', main_type: 3, robot: 'UGV Beast' },
  };
  var ATTACH_META = {
    none: { label: 'None', module_type: 0 },
    ptz: { label: 'PTZ turret', module_type: 2 },
    roarm2: { label: 'RoArm-M2', module_type: 1 },
  };

  var selection = {
    base: 'rover',
    attachment: 'ptz',
    use_lidar: false,
    camera_prefer: 'auto',
  };

  var serverOk = false;
  var dirty = false;
  var lastSigns = { drive_linear_sign: null, drive_angular_sign: null };

  function $(id) {
    return document.getElementById(id);
  }

  function svgRoverBase() {
    // Top-right is camera-forward. Rectangular hull, 4 wheels, axles, nose chevron.
    return [
      '<g class="lo-layer lo-rover" data-stencil="rover">',
      '<ellipse class="lo-ground" cx="200" cy="222" rx="132" ry="8"/>',
      '<circle class="lo-wheel lo-wheel-far" cx="104" cy="188" r="16"/>',
      '<circle class="lo-wheel lo-wheel-far" cx="268" cy="188" r="16"/>',
      '<circle class="lo-hub lo-wheel-far" cx="104" cy="188" r="6"/>',
      '<circle class="lo-hub lo-wheel-far" cx="268" cy="188" r="6"/>',
      '<line class="lo-axle" x1="104" y1="188" x2="268" y2="188"/>',
      '<rect class="lo-hull-top" x="108" y="108" width="196" height="22" rx="2"/>',
      '<rect class="lo-hull" x="92" y="126" width="216" height="52" rx="4"/>',
      '<rect class="lo-deck" x="108" y="132" width="168" height="28" rx="2"/>',
      '<line class="lo-stroke-thin" x1="118" y1="146" x2="260" y2="146"/>',
      '<line class="lo-stroke-thin" x1="184" y1="134" x2="184" y2="158"/>',
      '<rect class="lo-hatch" x="126" y="136" width="46" height="20" rx="1"/>',
      '<polygon class="lo-chevron" points="300,134 334,152 300,170"/>',
      '<polygon class="lo-chevron-void" points="298,140 320,152 298,164"/>',
      '<rect class="lo-cam" x="278" y="140" width="20" height="24" rx="2"/>',
      '<circle class="lo-lens" cx="296" cy="152" r="5"/>',
      '<circle class="lo-lens-core" cx="296" cy="152" r="2"/>',
      '<line class="lo-axle" x1="118" y1="168" x2="118" y2="204"/>',
      '<line class="lo-axle" x1="274" y1="168" x2="274" y2="204"/>',
      '<circle class="lo-wheel" cx="118" cy="204" r="22"/>',
      '<circle class="lo-wheel" cx="274" cy="204" r="22"/>',
      '<circle class="lo-hub" cx="118" cy="204" r="8"/>',
      '<circle class="lo-hub" cx="274" cy="204" r="8"/>',
      '<line class="lo-stroke-thin" x1="102" y1="204" x2="134" y2="204"/>',
      '<line class="lo-stroke-thin" x1="258" y1="204" x2="290" y2="204"/>',
      '<line class="lo-stroke-thin" x1="118" y1="188" x2="118" y2="220"/>',
      '<line class="lo-stroke-thin" x1="274" y1="188" x2="274" y2="220"/>',
      '<text class="lo-caption" x="200" y="28">ROVER  ·  WHEELS  ·  MAIN 2</text>',
      '</g>',
    ].join('');
  }

  function svgBeastBase() {
    // Lower, wider hull. Left+right tracks with chevrons, idler + sprocket circles.
    var chev = '';
    var i;
    for (i = 0; i < 7; i++) {
      var x = 86 + i * 34;
      chev +=
        '<polyline class="lo-track-chev" points="' +
        (x - 7) +
        ',176 ' +
        x +
        ',184 ' +
        (x - 7) +
        ',192"/>';
      chev +=
        '<polyline class="lo-track-chev" points="' +
        (x - 7) +
        ',210 ' +
        x +
        ',218 ' +
        (x - 7) +
        ',226"/>';
    }
    return [
      '<g class="lo-layer lo-beast" data-stencil="beast">',
      '<ellipse class="lo-ground" cx="200" cy="236" rx="156" ry="8"/>',
      '<rect class="lo-track lo-track-far" x="48" y="168" width="304" height="28" rx="14"/>',
      '<rect class="lo-hull-top" x="78" y="118" width="244" height="18" rx="2"/>',
      '<rect class="lo-hull lo-hull-wide" x="58" y="132" width="284" height="40" rx="4"/>',
      '<rect class="lo-deck" x="86" y="136" width="212" height="24" rx="2"/>',
      '<line class="lo-stroke-thin" x1="98" y1="148" x2="282" y2="148"/>',
      '<rect class="lo-hatch" x="128" y="138" width="56" height="16" rx="1"/>',
      '<polygon class="lo-chevron" points="334,136 368,152 334,168"/>',
      '<polygon class="lo-chevron-void" points="332,142 352,152 332,162"/>',
      '<rect class="lo-track" x="42" y="198" width="316" height="34" rx="17"/>',
      '<rect class="lo-track-inner" x="58" y="206" width="284" height="18" rx="9"/>',
      chev,
      '<circle class="lo-sprocket" cx="64" cy="184" r="13"/>',
      '<circle class="lo-sprocket" cx="336" cy="184" r="13"/>',
      '<circle class="lo-sprocket" cx="62" cy="215" r="16"/>',
      '<circle class="lo-sprocket" cx="338" cy="215" r="16"/>',
      '<circle class="lo-hub" cx="64" cy="184" r="4"/>',
      '<circle class="lo-hub" cx="336" cy="184" r="4"/>',
      '<circle class="lo-hub" cx="62" cy="215" r="5"/>',
      '<circle class="lo-hub" cx="338" cy="215" r="5"/>',
      '<circle class="lo-idler" cx="148" cy="215" r="7"/>',
      '<circle class="lo-idler" cx="200" cy="215" r="7"/>',
      '<circle class="lo-idler" cx="252" cy="215" r="7"/>',
      '<text class="lo-caption" x="200" y="28">BEAST  ·  TRACKS  ·  MAIN 3</text>',
      '</g>',
    ].join('');
  }

  function svgPtz() {
    // Dome + barrel on a deck ring, camera-forward (right).
    return [
      '<g class="lo-layer lo-ptz" data-stencil="ptz">',
      '<rect class="lo-mount" x="228" y="128" width="36" height="10" rx="1"/>',
      '<ellipse class="lo-ring" cx="246" cy="128" rx="28" ry="8"/>',
      '<ellipse class="lo-ring-inner" cx="246" cy="128" rx="18" ry="5"/>',
      '<path class="lo-dome" d="M224,122 C224,96 268,96 268,122 L268,128 C268,134 224,134 224,128 Z"/>',
      '<ellipse class="lo-dome-cap" cx="246" cy="102" rx="16" ry="7"/>',
      '<rect class="lo-barrel" x="262" y="110" width="52" height="14" rx="3"/>',
      '<rect class="lo-barrel-tip" x="312" y="113" width="14" height="8" rx="1"/>',
      '<circle class="lo-lens-core" cx="326" cy="117" r="2.4"/>',
      '<line class="lo-stroke-thin" x1="246" y1="100" x2="246" y2="128"/>',
      '</g>',
    ].join('');
  }

  function svgRoarm() {
    // Base yaw, shoulder, elbow, gripper — parked over the deck.
    return [
      '<g class="lo-layer lo-roarm" data-stencil="roarm2">',
      '<ellipse class="lo-ring" cx="168" cy="136" rx="22" ry="7"/>',
      '<rect class="lo-yaw" x="156" y="108" width="24" height="30" rx="3"/>',
      '<circle class="lo-joint" cx="168" cy="112" r="8"/>',
      '<path class="lo-arm-seg" d="M168 112 L142 72"/>',
      '<circle class="lo-joint" cx="142" cy="72" r="6"/>',
      '<path class="lo-arm-seg" d="M142 72 L232 48"/>',
      '<circle class="lo-joint" cx="232" cy="48" r="5"/>',
      '<path class="lo-arm-seg lo-arm-fore" d="M232 48 L278 70"/>',
      '<path class="lo-grip" d="M276 64 L300 56"/>',
      '<path class="lo-grip" d="M276 76 L300 86"/>',
      '<path class="lo-grip-pad" d="M296 54 L306 52 L308 60 L298 60 Z"/>',
      '<path class="lo-grip-pad" d="M296 88 L306 90 L308 82 L298 82 Z"/>',
      '</g>',
    ].join('');
  }

  function svgNoneAttach() {
    return [
      '<g class="lo-layer lo-none" data-stencil="none">',
      '<rect class="lo-empty-pad" x="214" y="122" width="48" height="16" rx="2"/>',
      '<line class="lo-empty-x" x1="222" y1="126" x2="254" y2="134"/>',
      '<line class="lo-empty-x" x1="254" y1="126" x2="222" y2="134"/>',
      '</g>',
    ].join('');
  }

  function svgGhostDeck() {
    return [
      '<g class="lo-layer lo-ghost" opacity="0.32">',
      '<rect class="lo-hull" x="92" y="126" width="216" height="52" rx="4"/>',
      '<polygon class="lo-chevron" points="300,134 334,152 300,170"/>',
      '<circle class="lo-wheel" cx="118" cy="204" r="22"/>',
      '<circle class="lo-wheel" cx="274" cy="204" r="22"/>',
      '</g>',
    ].join('');
  }

  function wrapThumb(inner) {
    return (
      '<svg class="loadout-thumb-svg" viewBox="0 0 400 260" aria-hidden="true">' +
      inner +
      '</svg>'
    );
  }

  function formatSign(v) {
    if (v == null || v === '') return '—';
    var n = Number(v);
    if (isNaN(n)) return String(v);
    if (n > 0) return '+' + String(n);
    return String(n);
  }

  function comboTitle() {
    var b = selection.base === 'beast' ? 'BEAST' : 'ROVER';
    var a = 'NONE';
    if (selection.attachment === 'ptz') a = 'PTZ';
    else if (selection.attachment === 'roarm2') a = 'ROARM-M2';
    return b + ' + ' + a;
  }

  function comboMeta() {
    var bm = BASE_META[selection.base] || BASE_META.rover;
    var am = ATTACH_META[selection.attachment] || ATTACH_META.none;
    return (
      'main_type ' +
      bm.main_type +
      ' · module_type ' +
      am.module_type +
      ' · ' +
      bm.drive
    );
  }

  function setStatus(text, kind) {
    var el = $('loadout-status');
    if (!el) return;
    el.textContent = text || '';
    el.className = 'loadout-status' + (kind ? ' is-' + kind : '');
  }

  function syncLidarBox() {
    var cb = $('loadout-use-lidar');
    if (cb) cb.checked = !!selection.use_lidar;
  }

  function renderSigns() {
    var lin = $('loadout-lin-sign');
    var ang = $('loadout-ang-sign');
    if (lin) lin.textContent = formatSign(lastSigns.drive_linear_sign);
    if (ang) ang.textContent = formatSign(lastSigns.drive_angular_sign);
  }

  function renderCards() {
    var bases = document.querySelectorAll('#mode-panel-loadout [data-base]');
    var atts = document.querySelectorAll('#mode-panel-loadout [data-attachment]');
    bases.forEach(function (btn) {
      var on = btn.getAttribute('data-base') === selection.base;
      btn.classList.toggle('is-selected', on);
      btn.setAttribute('aria-pressed', on ? 'true' : 'false');
    });
    atts.forEach(function (btn) {
      var on = btn.getAttribute('data-attachment') === selection.attachment;
      btn.classList.toggle('is-selected', on);
      btn.setAttribute('aria-pressed', on ? 'true' : 'false');
    });
  }

  function renderStage() {
    var baseG = $('loadout-base-layer');
    var attG = $('loadout-attach-layer');
    if (baseG) {
      baseG.innerHTML = selection.base === 'beast' ? svgBeastBase() : svgRoverBase();
    }
    if (attG) {
      if (selection.base === 'beast') attG.setAttribute('transform', 'translate(8, 6)');
      else attG.removeAttribute('transform');
      if (selection.attachment === 'ptz') attG.innerHTML = svgPtz();
      else if (selection.attachment === 'roarm2') attG.innerHTML = svgRoarm();
      else attG.innerHTML = svgNoneAttach();
    }
    var banner = $('loadout-arm-banner');
    if (banner) {
      banner.hidden = selection.attachment !== 'roarm2';
      if (selection.attachment === 'roarm2') {
        banner.textContent = selection._roarm_started ? ARM_STARTED_MESSAGE : ARM_WANTED_MESSAGE;
      }
    }
    var title = $('loadout-stage-title');
    var meta = $('loadout-stage-meta');
    if (title) title.textContent = comboTitle();
    if (meta) meta.textContent = comboMeta();
    renderCards();
    syncLidarBox();
    syncCameraPrefer();
    renderSigns();
  }

  function syncCameraPrefer() {
    var sel = $('loadout-camera-prefer');
    if (sel) sel.value = selection.camera_prefer || 'auto';
  }

  function fillThumbs() {
    var rover = $('loadout-thumb-rover');
    var beast = $('loadout-thumb-beast');
    var none = $('loadout-thumb-none');
    var ptz = $('loadout-thumb-ptz');
    var arm = $('loadout-thumb-roarm2');
    if (rover) rover.innerHTML = wrapThumb(svgRoverBase());
    if (beast) beast.innerHTML = wrapThumb(svgBeastBase());
    if (none) none.innerHTML = wrapThumb(svgGhostDeck() + svgNoneAttach());
    if (ptz) ptz.innerHTML = wrapThumb(svgGhostDeck() + svgPtz());
    if (arm) arm.innerHTML = wrapThumb(svgGhostDeck() + svgRoarm());
  }

  function getSelection() {
    return {
      base: selection.base,
      attachment: selection.attachment,
      use_lidar: !!selection.use_lidar,
      camera_prefer: selection.camera_prefer || 'auto',
    };
  }

  function applyPayload(data) {
    if (!data || typeof data !== 'object') return;
    var lo = data.loadout && typeof data.loadout === 'object' ? data.loadout : data;
    if (lo.base === 'rover' || lo.base === 'beast') selection.base = lo.base;
    if (lo.attachment === 'none' || lo.attachment === 'ptz' || lo.attachment === 'roarm2') {
      selection.attachment = lo.attachment;
    }
    if (typeof lo.use_lidar !== 'undefined') selection.use_lidar = !!lo.use_lidar;
    if (lo.camera_prefer) selection.camera_prefer = String(lo.camera_prefer);
    selection._roarm_started = !!data.roarm_started;
    if (typeof data.drive_linear_sign !== 'undefined') {
      lastSigns.drive_linear_sign = data.drive_linear_sign;
    }
    if (typeof data.drive_angular_sign !== 'undefined') {
      lastSigns.drive_angular_sign = data.drive_angular_sign;
    }
  }

  function payloadError(data, http) {
    if (data && (data.error || data.message)) return String(data.error || data.message);
    if (http && !http.ok) return 'HTTP ' + http.status;
    return 'request failed';
  }

  function parseResponse(res) {
    return res.text().then(function (text) {
      var data = {};
      if (text) {
        try {
          data = JSON.parse(text);
        } catch (e) {
          data = { error: text.slice(0, 180) || res.statusText };
        }
      }
      return { http: res, data: data };
    });
  }

  function refresh() {
    setStatus('loading…', 'busy');
    return fetch(API, { method: 'GET', headers: { Accept: 'application/json' } })
      .then(parseResponse)
      .then(function (pack) {
        if (
          !pack.http.ok ||
          pack.data.ok === false ||
          pack.data.success === false
        ) {
          serverOk = false;
          dirty = true;
          renderStage();
          setStatus('GET /api/loadout failed — editing locally', 'warn');
          return;
        }
        serverOk = true;
        applyPayload(pack.data);
        dirty = false;
        renderStage();
        var note = comboTitle().toLowerCase();
        setStatus('synced · ' + note, 'ok');
      })
      .catch(function () {
        serverOk = false;
        dirty = true;
        renderStage();
        setStatus('GET /api/loadout failed — editing locally', 'warn');
      });
  }

  function save() {
    var body = getSelection();
    var btn = $('loadout-save-btn');
    if (btn) btn.disabled = true;
    setStatus('saving…', 'busy');
    return fetch(API, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', Accept: 'application/json' },
      body: JSON.stringify(body),
    })
      .then(parseResponse)
      .then(function (pack) {
        var failed =
          !pack.http.ok ||
          pack.data.ok === false ||
          pack.data.success === false;
        if (failed) {
          setStatus('save failed: ' + payloadError(pack.data, pack.http), 'err');
          return;
        }
        serverOk = true;
        applyPayload(pack.data);
        dirty = false;
        renderStage();
        setStatus('saved · ' + comboTitle().toLowerCase(), 'ok');
      })
      .catch(function (e) {
        setStatus('save failed: ' + (e && e.message ? e.message : 'network'), 'err');
      })
      .finally(function () {
        if (btn) btn.disabled = false;
      });
  }

  function markDirty() {
    dirty = true;
    renderStage();
    var prefix = serverOk ? 'draft' : 'local draft';
    setStatus(prefix + ' · unsaved · ' + comboTitle().toLowerCase(), 'draft');
  }

  function wireUi() {
    var hangar = document.querySelector('#mode-panel-loadout .loadout-hangar');
    if (!hangar || hangar._ugvLoadoutWired) return;
    hangar._ugvLoadoutWired = true;
    hangar.addEventListener('click', function (ev) {
      var t = ev.target;
      if (!t || !t.closest) return;
      var baseBtn = t.closest('[data-base]');
      if (baseBtn && hangar.contains(baseBtn)) {
        var b = baseBtn.getAttribute('data-base');
        if (b === 'rover' || b === 'beast') {
          if (selection.base !== b) {
            selection.base = b;
            markDirty();
          }
        }
        return;
      }
      var attBtn = t.closest('[data-attachment]');
      if (attBtn && hangar.contains(attBtn)) {
        var a = attBtn.getAttribute('data-attachment');
        if (a === 'none' || a === 'ptz' || a === 'roarm2') {
          if (selection.attachment !== a) {
            selection.attachment = a;
            markDirty();
          }
        }
      }
    });
    var lidar = $('loadout-use-lidar');
    if (lidar) {
      lidar.addEventListener('change', function () {
        selection.use_lidar = !!lidar.checked;
        markDirty();
      });
    }
    var cam = $('loadout-camera-prefer');
    if (cam) {
      cam.addEventListener('change', function () {
        var v = String(cam.value || 'auto').toLowerCase();
        if (v === 'auto' || v === 'csi' || v === 'usb') {
          selection.camera_prefer = v;
          markDirty();
        }
      });
    }
    var saveBtn = $('loadout-save-btn');
    if (saveBtn) {
      saveBtn.addEventListener('click', function () {
        save();
      });
    }
  }

  function boot() {
    wireUi();
    fillThumbs();
    renderStage();
    if (window.ugvAppMode === 'loadout') refresh();
  }

  window.ugvLoadoutRefresh = refresh;
  window.ugvLoadoutGetSelection = getSelection;

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }
})();
