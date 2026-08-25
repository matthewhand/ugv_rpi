/* UGV 3D twin — one model for /3d and the hangar overlay.
 *
 * RoArm-M2-S FK matches Waveshare firmware EEMode=0 (clamp):
 *   polarToCartesian(l2, π/2 − (shoulder + t2rad))
 *   polarToCartesian(l3, π/2 − (elbow + shoulder))
 *   yaw base: X+ forward, Y+ left, Z+ up (from shoulder; L1 is the base column).
 * Source: waveshareteam/roarm_m2 RoArm-M2_config.h / RoArm-M2_module.h
 *
 * Three.js mapping (Y-up): robot X+ → +Z, robot Y+ → +X, robot Z+ → +Y.
 */
(function (global) {
    'use strict';

    var L1 = 0.12606;
    var L2A = 0.23682;
    var L2B = 0.03000;
    var L3A = 0.28015;
    var L3B = 0.00173;
    var L2 = Math.sqrt(L2A * L2A + L2B * L2B);
    var L3 = Math.sqrt(L3A * L3A + L3B * L3B);
    var T2 = Math.atan2(L2B, L2A);
    var HAND_CLOSED = 3.1416;
    var HAND_OPEN = 1.08;
    var DEG = 180 / Math.PI;

    var HOME = { base: 0, shoulder: 0, elbow: 1.5708, hand: 3.1416 };
    var POSES = {
        home: HOME,
        travel_tuck: { base: 0, shoulder: -0.62, elbow: 0.88, hand: 3.05 }
    };

    function clamp(v, lo, hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    function eZrToJoints(e, z, r, defaults) {
        defaults = defaults || {};
        var defaultE = defaults.e != null ? defaults.e : 60;
        var defaultZ = defaults.z != null ? defaults.z : 24;
        var defaultR = defaults.r != null ? defaults.r : 0;
        var base = clamp(((r - defaultR) * Math.PI) / 180, -1.2, 1.2);
        var shoulder = clamp(-(z - defaultZ) * 0.012, -0.9, 0.9);
        var eSpan = Math.max(1, 450 - defaultE);
        var eNorm = clamp((e - defaultE) / eSpan, 0, 1);
        var elbow = clamp(1.5708 - eNorm * 0.55, 0.85, 2.2);
        var hand = defaults.hand != null ? defaults.hand : HOME.hand;
        return { base: base, shoulder: shoulder, elbow: elbow, hand: hand };
    }

    function fk(joints) {
        var base = joints.base || 0;
        var shoulder = joints.shoulder || 0;
        var elbow = joints.elbow != null ? joints.elbow : HOME.elbow;
        var th2 = Math.PI / 2 - (shoulder + T2);
        var aOut = L2 * Math.cos(th2);
        var bOut = L2 * Math.sin(th2);
        var th3 = Math.PI / 2 - (elbow + shoulder);
        var cOut = L3 * Math.cos(th3);
        var dOut = L3 * Math.sin(th3);
        var rEe = aOut + cOut;
        var zEe = bOut + dOut;
        return {
            x: rEe * Math.cos(base),
            y: rEe * Math.sin(base),
            z: zEe,
            zWorld: L1 + zEe,
            elbowR: aOut,
            elbowZ: bOut,
            r: rEe
        };
    }

    function robotToThree(x, y, z) {
        // robot (X fwd, Y left, Z up) → Three (X right, Y up, Z fwd)
        return new THREE.Vector3(y, z, x);
    }

    function makeMat(color, extras) {
        var o = { color: color, metalness: 0.45, roughness: 0.4 };
        if (extras) Object.keys(extras).forEach(function (k) { o[k] = extras[k]; });
        return new THREE.MeshStandardMaterial(o);
    }

    function makeBone(radius, color) {
        var g = new THREE.CylinderGeometry(radius, radius, 1, 12);
        var m = new THREE.Mesh(g, makeMat(color));
        m.castShadow = true;
        return m;
    }

    function placeBone(mesh, from, to) {
        var dir = new THREE.Vector3().subVectors(to, from);
        var len = Math.max(dir.length(), 1e-4);
        mesh.position.copy(from).add(to).multiplyScalar(0.5);
        mesh.scale.set(1, len, 1);
        mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
    }

    function create(opts) {
        opts = opts || {};
        var container = opts.container || null;
        var canvas = opts.canvas || null;
        var compact = !!opts.compact;
        var hud = opts.hud || {};
        var pollMs = opts.pollMs != null ? opts.pollMs : (compact ? 250 : 200);
        var defaults = {
            e: opts.defaultE != null ? opts.defaultE : 60,
            z: opts.defaultZ != null ? opts.defaultZ : 24,
            r: opts.defaultR != null ? opts.defaultR : 0,
            hand: HOME.hand
        };

        var host = canvas || container;
        if (!host) throw new Error('UgvTwin.create: need canvas or container');

        function viewSize() {
            if (canvas) {
                var cw = canvas.clientWidth || (canvas.parentElement && canvas.parentElement.clientWidth) || 280;
                var ch = canvas.clientHeight || (canvas.parentElement && canvas.parentElement.clientHeight) || 200;
                return { w: Math.max(80, cw), h: Math.max(80, ch) };
            }
            var w = (container && container.clientWidth) || window.innerWidth || 640;
            var h = (container && container.clientHeight) || window.innerHeight || 480;
            return { w: Math.max(120, w), h: Math.max(120, h) };
        }

        var vs0 = viewSize();
        var scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0b0f19);
        scene.fog = compact ? null : new THREE.FogExp2(0x0b0f19, 0.22);

        var camera = new THREE.PerspectiveCamera(compact ? 48 : 50, vs0.w / vs0.h, 0.02, 20);
        var renderer = canvas
            ? new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: false })
            : new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(vs0.w, vs0.h, false);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        renderer.shadowMap.enabled = !compact;
        if (!canvas && container) container.appendChild(renderer.domElement);

        var controls = null;
        if (typeof THREE.OrbitControls === 'function') {
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.08;
            controls.maxPolarAngle = Math.PI * 0.92;
            controls.minDistance = compact ? 0.25 : 0.4;
            controls.maxDistance = compact ? 2.2 : 6;
        }

        scene.add(new THREE.AmbientLight(0xffffff, compact ? 0.85 : 0.7));
        var dir = new THREE.DirectionalLight(0x38bdf8, compact ? 1.0 : 1.2);
        dir.position.set(1.4, 2.2, 1.6);
        scene.add(dir);
        var fill = new THREE.DirectionalLight(0xffc8a0, 0.35);
        fill.position.set(-1.2, 0.8, -0.6);
        scene.add(fill);

        var grid = new THREE.GridHelper(compact ? 1.6 : 4, compact ? 16 : 20, 0x38bdf8, 0x1e293b);
        scene.add(grid);

        var robot = new THREE.Group();
        scene.add(robot);

        var chassisGroup = new THREE.Group();
        robot.add(chassisGroup);
        var armRoot = new THREE.Group();
        robot.add(armRoot);
        var ptGroup = new THREE.Group();
        robot.add(ptGroup);

        var l1Bone = makeBone(0.018, 0x64748b);
        var l2Bone = makeBone(0.014, 0x9aa3b2);
        var l3Bone = makeBone(0.012, 0x6b7280);
        var shoulderBall = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 16), makeMat(0x4FF5C0));
        var elbowBall = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 16), makeMat(0x4FF5C0));
        var eeBall = new THREE.Mesh(
            new THREE.SphereGeometry(0.016, 14, 14),
            makeMat(0xff8c8c, { emissive: 0xff4444, emissiveIntensity: 0.25 })
        );
        var gripA = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.006, 0.055), makeMat(0xfbbf24));
        var gripB = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.006, 0.055), makeMat(0xfbbf24));
        armRoot.add(l1Bone, l2Bone, l3Bone, shoulderBall, elbowBall, eeBall, gripA, gripB);

        var panLink = new THREE.Group();
        var tiltLink = new THREE.Group();
        var panMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 16), makeMat(0x38bdf8));
        var camCube = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, 0.06), makeMat(0x0f172a));
        camCube.position.set(0, 0.02, 0.02);
        tiltLink.position.set(0, 0.04, 0);
        tiltLink.add(camCube);
        panLink.add(panMesh);
        panLink.add(tiltLink);
        ptGroup.add(panLink);

        var trailPts = [];
        var trailLine = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ vertexColors: true, transparent: true })
        );
        scene.add(trailLine);

        var state = {
            chassis: 'rover',
            attachment: 'ptz',
            joints: Object.assign({}, HOME),
            ptzPan: 0,
            ptzTilt: 0,
            connected: false,
            robotName: 'UGV'
        };
        var chassisBuiltFor = '';
        var mount = { y: 0.12, z: 0.04 };
        var running = true;
        var pollTimer = null;

        function clearGroup(g) {
            while (g.children.length) {
                var ch = g.children[0];
                g.remove(ch);
                if (ch.geometry) ch.geometry.dispose();
            }
        }

        function addWheel(parent, x, y, z) {
            var geo = new THREE.CylinderGeometry(0.05, 0.05, 0.04, 20);
            geo.rotateZ(Math.PI / 2);
            var w = new THREE.Mesh(geo, makeMat(0x0f172a, { roughness: 0.9, metalness: 0.1 }));
            w.position.set(x, y, z);
            parent.add(w);
        }

        function addTrack(parent, x) {
            var track = new THREE.Mesh(
                new THREE.BoxGeometry(0.045, 0.055, 0.30),
                makeMat(0x111827, { roughness: 0.95, metalness: 0.05 })
            );
            track.position.set(x, 0.04, 0);
            parent.add(track);
            for (var i = -2; i <= 2; i++) {
                var pad = new THREE.Mesh(
                    new THREE.BoxGeometry(0.05, 0.018, 0.04),
                    makeMat(0x1f2937)
                );
                pad.position.set(x, 0.012, i * 0.055);
                parent.add(pad);
            }
        }

        function rebuildChassis() {
            var key = state.chassis + ':' + state.attachment;
            if (key === chassisBuiltFor) return;
            chassisBuiltFor = key;
            clearGroup(chassisGroup);

            var beast = state.chassis === 'beast';
            var bodyW = beast ? 0.20 : 0.20;
            var bodyH = beast ? 0.10 : 0.09;
            var bodyL = beast ? 0.30 : 0.28;
            var body = new THREE.Mesh(
                new THREE.BoxGeometry(bodyW, bodyH, bodyL),
                makeMat(0x1e293b, { metalness: 0.75, roughness: 0.32 })
            );
            body.position.y = beast ? 0.095 : 0.10;
            chassisGroup.add(body);
            var cover = new THREE.Mesh(
                new THREE.BoxGeometry(bodyW * 0.78, 0.025, bodyL * 0.72),
                makeMat(beast ? 0xb45309 : 0x0284c7)
            );
            cover.position.set(0, body.position.y + bodyH / 2 + 0.01, -0.01);
            chassisGroup.add(cover);

            if (beast) {
                addTrack(chassisGroup, -0.12);
                addTrack(chassisGroup, 0.12);
            } else {
                addWheel(chassisGroup, -0.11, 0.05, 0.09);
                addWheel(chassisGroup, -0.11, 0.05, -0.09);
                addWheel(chassisGroup, 0.11, 0.05, 0.09);
                addWheel(chassisGroup, 0.11, 0.05, -0.09);
            }

            mount.y = body.position.y + bodyH / 2 + 0.01;
            mount.z = beast ? 0.06 : 0.02;
            armRoot.position.set(0, mount.y, mount.z);
            ptGroup.position.set(0, mount.y + 0.02, 0);
        }

        function applyJoints(joints) {
            if (!joints) return;
            state.joints = {
                base: +joints.base || 0,
                shoulder: +joints.shoulder || 0,
                elbow: joints.elbow != null ? +joints.elbow : HOME.elbow,
                hand: joints.hand != null ? +joints.hand : HOME.hand
            };
            defaults.hand = state.joints.hand;
            var p = fk(state.joints);
            var sh = robotToThree(0, 0, L1);
            var el = robotToThree(p.elbowR, 0, L1 + p.elbowZ);
            var ee = robotToThree(p.r, 0, L1 + p.z);
            // yaw whole arm in Three Y (matches +base → left → +X)
            armRoot.rotation.y = state.joints.base;
            var origin = new THREE.Vector3(0, 0, 0);
            placeBone(l1Bone, origin, sh);
            placeBone(l2Bone, sh, el);
            placeBone(l3Bone, el, ee);
            shoulderBall.position.copy(sh);
            elbowBall.position.copy(el);
            eeBall.position.copy(ee);

            var open = clamp((HAND_CLOSED - state.joints.hand) / (HAND_CLOSED - HAND_OPEN), 0, 1);
            var spread = 0.008 + open * 0.028;
            var along = new THREE.Vector3().subVectors(ee, el);
            if (along.lengthSq() < 1e-8) along.set(0, 0, 1);
            along.normalize();
            var side = new THREE.Vector3().crossVectors(along, new THREE.Vector3(0, 1, 0));
            if (side.lengthSq() < 1e-8) side.set(1, 0, 0);
            side.normalize();
            var mid = ee.clone().add(along.clone().multiplyScalar(0.02));
            gripA.position.copy(mid).add(side.clone().multiplyScalar(spread));
            gripB.position.copy(mid).add(side.clone().multiplyScalar(-spread));
            gripA.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), along);
            gripB.quaternion.copy(gripA.quaternion);

            var eeWorld = ee.clone();
            armRoot.updateMatrixWorld(true);
            armRoot.localToWorld(eeWorld);
            if (!trailPts.length || trailPts[trailPts.length - 1].distanceTo(eeWorld) > 0.004) {
                trailPts.push(eeWorld);
                if (trailPts.length > 48) trailPts.shift();
            }
            if (trailPts.length > 1) {
                var pos = [];
                var col = [];
                var c = new THREE.Color(0x4FF5C0);
                for (var i = 0; i < trailPts.length; i++) {
                    pos.push(trailPts[i].x, trailPts[i].y, trailPts[i].z);
                    var a = i / (trailPts.length - 1);
                    col.push(c.r, c.g, c.b, a);
                }
                trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
                trailLine.geometry.setAttribute('color', new THREE.Float32BufferAttribute(col, 4));
                trailLine.geometry.computeBoundingSphere();
            }
            updateHud();
        }

        function applyPtz(panRad, tiltRad) {
            state.ptzPan = panRad || 0;
            state.ptzTilt = tiltRad || 0;
            panLink.rotation.y = state.ptzPan;
            tiltLink.rotation.x = state.ptzTilt;
            updateHud();
        }

        function setAttachmentVisibility() {
            var arm = state.attachment === 'roarm2';
            var pt = state.attachment === 'ptz';
            armRoot.visible = arm;
            trailLine.visible = arm;
            ptGroup.visible = pt;
        }

        function frameCamera() {
            if (compact && state.attachment === 'roarm2') {
                camera.position.set(0.55, 0.48, 0.62);
                if (controls) controls.target.set(0, mount.y + 0.18, mount.z + 0.12);
                else camera.lookAt(0, mount.y + 0.18, mount.z + 0.12);
            } else {
                camera.position.set(1.15, 0.95, 1.35);
                if (controls) controls.target.set(0, 0.18, 0.05);
                else camera.lookAt(0, 0.18, 0.05);
            }
        }

        function setText(id, text) {
            var el = typeof id === 'string' ? document.getElementById(id) : id;
            if (el) el.textContent = text;
        }

        function updateHud() {
            var j = state.joints;
            var attachLabel = state.attachment === 'roarm2' ? 'RoArm-M2' : (state.attachment === 'ptz' ? 'PT gimbal' : 'none');
            setText(hud.chassis || 'chassis-val', (state.chassis || 'rover') + ' · ' + attachLabel);
            setText(hud.attach || 'attach-val', attachLabel);
            setText(hud.base || 'base-val', (j.base * DEG).toFixed(1) + '°');
            setText(hud.shoulder || 'shoulder-val', (j.shoulder * DEG).toFixed(1) + '°');
            setText(hud.elbow || 'elbow-val', (j.elbow * DEG).toFixed(1) + '°');
            var handPct = clamp((HAND_CLOSED - j.hand) / (HAND_CLOSED - HAND_OPEN), 0, 1) * 100;
            setText(hud.hand || 'hand-val', handPct.toFixed(0) + '% open');
            setText(hud.pan || 'pan-val', (state.ptzPan * DEG).toFixed(1) + '°');
            setText(hud.tilt || 'tilt-val', (state.ptzTilt * DEG).toFixed(1) + '°');
            setText(hud.compactJoints || 'roarm-workspace-joints',
                'B ' + (j.base * DEG).toFixed(0) +
                '  S ' + (j.shoulder * DEG).toFixed(0) +
                '  E ' + (j.elbow * DEG).toFixed(0));
            var roarmHud = document.getElementById('hud-roarm');
            var ptzHud = document.getElementById('hud-ptz');
            if (roarmHud) roarmHud.style.display = state.attachment === 'roarm2' ? '' : 'none';
            if (ptzHud) ptzHud.style.display = state.attachment === 'ptz' ? '' : 'none';
            var dot = document.getElementById('status-dot');
            if (dot) {
                if (state.connected || state.attachment !== 'roarm2') dot.classList.add('connected');
                else dot.classList.remove('connected');
            }
        }

        function applySnapshot(d) {
            if (!d || !d.ok) return;
            var chassisChanged = d.chassis && d.chassis !== state.chassis;
            var attachChanged = d.attachment && d.attachment !== state.attachment;
            if (d.chassis) state.chassis = d.chassis;
            if (d.attachment) state.attachment = d.attachment;
            if (d.robot_name) state.robotName = d.robot_name;
            state.connected = !!d.roarm_connected;
            rebuildChassis();
            setAttachmentVisibility();
            if (chassisChanged || attachChanged) frameCamera();
            if (d.joints) applyJoints(d.joints);
            if (d.ptz) {
                var pan = d.ptz.pan_deg != null ? d.ptz.pan_deg : d.ptz.cmd;
                var tilt = d.ptz.tilt_deg != null ? d.ptz.tilt_deg : d.ptz.tilt;
                if (typeof pan === 'number') applyPtz(pan * Math.PI / 180, (typeof tilt === 'number' ? tilt : 0) * Math.PI / 180);
            }
            var bridge = document.getElementById('bridge-status');
            if (bridge) {
                if (state.attachment === 'roarm2') {
                    bridge.innerText = state.connected
                        ? ('RoArm USB' + (d.roarm_port ? ' ' + d.roarm_port : ''))
                        : (d.roarm_started ? 'RoArm starting…' : 'RoArm idle (last pose)');
                } else {
                    bridge.innerText = 'Direct / hangar';
                }
            }
            setText('twin-title-name', state.robotName);
        }

        function poll() {
            fetch('/api/twin')
                .then(function (r) { return r.json(); })
                .then(applySnapshot)
                .catch(function () {});
        }

        function resize() {
            var vs = viewSize();
            camera.aspect = vs.w / vs.h;
            camera.updateProjectionMatrix();
            renderer.setSize(vs.w, vs.h, false);
        }

        function animate() {
            if (!running) return;
            requestAnimationFrame(animate);
            if (controls) controls.update();
            renderer.render(scene, camera);
        }

        rebuildChassis();
        setAttachmentVisibility();
        applyJoints(HOME);
        frameCamera();
        resize();
        animate();
        poll();
        if (pollMs > 0) pollTimer = setInterval(poll, pollMs);

        window.addEventListener('resize', resize);
        var ro = null;
        if (typeof ResizeObserver !== 'undefined' && host) {
            ro = new ResizeObserver(resize);
            ro.observe(host);
            if (canvas && canvas.parentElement) ro.observe(canvas.parentElement);
        }

        return {
            applySnapshot: applySnapshot,
            setJoints: applyJoints,
            setFromEZR: function (e, z, r, hand) {
                applyJoints(eZrToJoints(e, z, r, {
                    e: defaults.e, z: defaults.z, r: defaults.r,
                    hand: hand != null ? hand : defaults.hand
                }));
            },
            setLoadout: function (lo) {
                if (!lo) return;
                if (lo.chassis) state.chassis = lo.chassis;
                if (lo.attachment) state.attachment = lo.attachment;
                rebuildChassis();
                setAttachmentVisibility();
                frameCamera();
            },
            setPtz: applyPtz,
            clearTrail: function () {
                trailPts = [];
                trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
            },
            resize: resize,
            poll: poll,
            destroy: function () {
                running = false;
                if (pollTimer) clearInterval(pollTimer);
                if (ro) ro.disconnect();
                window.removeEventListener('resize', resize);
                renderer.dispose();
            }
        };
    }

    global.UgvTwin = {
        create: create,
        fk: fk,
        eZrToJoints: eZrToJoints,
        HOME: HOME,
        POSES: POSES,
        KIN: { L1: L1, L2A: L2A, L2B: L2B, L3A: L3A, L3B: L3B, L2: L2, L3: L3, T2: T2 }
    };
})(window);
