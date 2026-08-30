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

    function num(v, fallback) {
        var n = Number(v);
        return isFinite(n) ? n : fallback;
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
        var base = num(joints.base, 0);
        var shoulder = num(joints.shoulder, 0);
        var elbow = num(joints.elbow, HOME.elbow);
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
        scene.fog = compact ? null : new THREE.FogExp2(0x0b0f19, 0.08);

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
            if (compact) controls.enablePan = false;
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
        var fwd = new THREE.ArrowHelper(
            new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0.002, 0),
            compact ? 0.22 : 0.35, 0x38bdf8, 0.04, 0.025
        );
        scene.add(fwd);

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

        // Joint rings: full /3d + Twin popup iframe only (not the hangar canvas overlay).
        // Base → yaw, shoulder → lift, elbow → reach. Grip stays a slider.
        var showRings = opts.jointRings != null ? !!opts.jointRings : !canvas;
        var ringDrag = null;
        var ringRay = new THREE.Raycaster();
        var ringPtr = new THREE.Vector2();

        function makeRing(name, radius, color) {
            var geo = new THREE.TorusGeometry(radius, 0.006, 10, 48);
            var mat = new THREE.MeshStandardMaterial({
                color: color, metalness: 0.25, roughness: 0.4,
                transparent: true, opacity: 0.82, side: THREE.DoubleSide,
                depthWrite: false
            });
            var mesh = new THREE.Mesh(geo, mat);
            mesh.name = 'ring-' + name;
            mesh.userData.joint = name;
            mesh.renderOrder = 3;
            var hit = new THREE.Mesh(
                new THREE.TorusGeometry(radius, 0.018, 8, 32),
                new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide })
            );
            hit.userData.joint = name;
            mesh.add(hit);
            return mesh;
        }

        var ringBase = makeRing('base', 0.09, 0x38bdf8);
        var ringShoulder = makeRing('shoulder', 0.075, 0x4FF5C0);
        var ringElbow = makeRing('elbow', 0.065, 0xfbbf24);
        ringBase.rotation.x = Math.PI / 2;
        ringShoulder.rotation.y = Math.PI / 2;
        ringElbow.rotation.y = Math.PI / 2;
        if (showRings) {
            armRoot.add(ringBase, ringShoulder, ringElbow);
        }

        function updateRings() {
            if (!showRings) return;
            var on = state.attachment === 'roarm2';
            ringBase.visible = ringShoulder.visible = ringElbow.visible = on;
            ringBase.position.set(0, 0.012, 0);
            ringShoulder.position.copy(shoulderBall.position);
            ringElbow.position.copy(elbowBall.position);
        }

        function sendIkJog(yaw, lift, reach) {
            if (!yaw && !lift && !reach) return;
            try {
                if (window.parent && window.parent !== window &&
                    typeof window.parent.roarmQueueMove === 'function') {
                    window.parent.roarmQueueMove(yaw, lift, reach);
                    return;
                }
            } catch (e) {}
            if (typeof window.roarmQueueMove === 'function') {
                window.roarmQueueMove(yaw, lift, reach);
                return;
            }
            fetch('/api/arm/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ d_yaw_deg: yaw, d_lift_mm: lift, d_reach_mm: reach })
            }).catch(function () {});
        }

        function ringNdc(ev) {
            var rect = renderer.domElement.getBoundingClientRect();
            ringPtr.x = ((ev.clientX - rect.left) / Math.max(1, rect.width)) * 2 - 1;
            ringPtr.y = -((ev.clientY - rect.top) / Math.max(1, rect.height)) * 2 + 1;
        }

        function ringAngle(ev, joint) {
            ringNdc(ev);
            ringRay.setFromCamera(ringPtr, camera);
            armRoot.updateMatrixWorld(true);
            var axisLocal = joint === 'base' ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
            var axisWorld = axisLocal.clone().transformDirection(armRoot.matrixWorld);
            var mesh = joint === 'base' ? ringBase : (joint === 'shoulder' ? ringShoulder : ringElbow);
            var center = new THREE.Vector3();
            mesh.getWorldPosition(center);
            var plane = new THREE.Plane().setFromNormalAndCoplanarPoint(axisWorld, center);
            var hit = new THREE.Vector3();
            if (!ringRay.ray.intersectPlane(plane, hit)) return null;
            var local = armRoot.worldToLocal(hit.clone());
            var c = armRoot.worldToLocal(center.clone());
            var v = local.sub(c);
            return joint === 'base' ? Math.atan2(v.x, v.z) : Math.atan2(v.y, v.z);
        }

        function pickRing(ev) {
            ringNdc(ev);
            ringRay.setFromCamera(ringPtr, camera);
            var hits = ringRay.intersectObjects([ringBase, ringShoulder, ringElbow], true);
            if (!hits.length) return null;
            var obj = hits[0].object;
            while (obj && !obj.userData.joint) obj = obj.parent;
            return obj ? obj.userData.joint : null;
        }

        function jogRing(joint, dAng) {
            var deg = dAng * 180 / Math.PI;
            var yaw = 0, lift = 0, reach = 0;
            if (joint === 'base') yaw = clamp(deg, -12, 12);
            else if (joint === 'shoulder') lift = clamp(deg * 2.2, -25, 25);
            else if (joint === 'elbow') reach = clamp(deg * 2.8, -30, 30);
            sendIkJog(yaw, lift, reach);
        }

        function onRingDown(ev) {
            if (!showRings || state.attachment !== 'roarm2') return;
            var joint = pickRing(ev);
            if (!joint) return;
            var ang = ringAngle(ev, joint);
            if (ang == null) return;
            ringDrag = { joint: joint, lastAngle: ang };
            if (controls) controls.enabled = false;
            ev.preventDefault();
            ev.stopPropagation();
        }

        function onRingMove(ev) {
            if (!ringDrag) return;
            var ang = ringAngle(ev, ringDrag.joint);
            if (ang == null) return;
            var d = ang - ringDrag.lastAngle;
            if (d > Math.PI) d -= Math.PI * 2;
            if (d < -Math.PI) d += Math.PI * 2;
            ringDrag.lastAngle = ang;
            jogRing(ringDrag.joint, d);
            ev.preventDefault();
        }

        function onRingUp() {
            if (!ringDrag) return;
            ringDrag = null;
            if (controls) controls.enabled = true;
        }

        if (showRings) {
            renderer.domElement.addEventListener('pointerdown', onRingDown);
            window.addEventListener('pointermove', onRingMove);
            window.addEventListener('pointerup', onRingUp);
            window.addEventListener('pointercancel', onRingUp);
        }

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

        var LIDAR_BINS = 36;
        var lidarPos = new Float32Array(LIDAR_BINS * 3);
        var lidarGeo = new THREE.BufferGeometry();
        lidarGeo.setAttribute('position', new THREE.BufferAttribute(lidarPos, 3));
        lidarGeo.setDrawRange(0, 0);
        var lidarPts = new THREE.Points(
            lidarGeo,
            new THREE.PointsMaterial({
                color: 0x38bdf8,
                size: compact ? 0.028 : 0.035,
                sizeAttenuation: true,
                transparent: true,
                opacity: 0.95
            })
        );
        lidarPts.visible = false;
        scene.add(lidarPts);
        var lidarRayPos = new Float32Array(LIDAR_BINS * 2 * 3);
        var lidarRayGeo = new THREE.BufferGeometry();
        lidarRayGeo.setAttribute('position', new THREE.BufferAttribute(lidarRayPos, 3));
        lidarRayGeo.setDrawRange(0, 0);
        var lidarRays = new THREE.LineSegments(
            lidarRayGeo,
            new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: compact ? 0.18 : 0.28 })
        );
        lidarRays.visible = false;
        scene.add(lidarRays);

        var state = {
            chassis: 'rover',
            attachment: 'ptz',
            joints: Object.assign({}, HOME),
            ptzPan: 0,
            ptzTilt: 0,
            connected: false,
            pollOk: false,
            robotName: 'UGV',
            lidar: null
        };
        var chassisBuiltFor = '';
        var mount = { y: 0.12, z: 0.04 };
        var running = true;
        var paused = false;
        var pollTimer = null;
        var localHoldUntil = 0;
        var lastFk = null;

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
            // Front deck: arm sits near the camera-forward edge, not mid-chassis.
            mount.z = bodyL * 0.32;
            armRoot.position.set(0, mount.y, mount.z);
            ptGroup.position.set(0, mount.y + 0.02, 0);
        }

        function applyJoints(joints, fromPoll) {
            if (!joints) return;
            if (!fromPoll) localHoldUntil = Date.now() + 450;
            state.joints = {
                base: num(joints.base, 0),
                shoulder: num(joints.shoulder, 0),
                elbow: num(joints.elbow, HOME.elbow),
                hand: num(joints.hand, HOME.hand)
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
                    col.push(c.r * a, c.g * a, c.b * a);
                }
                trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
                trailLine.geometry.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
                trailLine.geometry.computeBoundingSphere();
            }
            lastFk = p;
            updateRings();
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
            updateRings();
        }

        function frameCamera() {
            if (compact && state.attachment === 'roarm2') {
                camera.position.set(0.72, 0.58, 0.78);
                if (controls) controls.target.set(0, mount.y + 0.22, mount.z + 0.08);
                else camera.lookAt(0, mount.y + 0.22, mount.z + 0.08);
            } else {
                camera.position.set(1.15, 0.95, 1.35);
                if (controls) controls.target.set(0, 0.22, 0.08);
                else camera.lookAt(0, 0.22, 0.08);
            }
        }

        function applyLidarScan(lidar) {
            state.lidar = lidar || null;
            var bins = lidar && lidar.bins_10deg_mm;
            var live = !!(lidar && lidar.open && bins && bins.length);
            lidarPts.visible = live;
            lidarRays.visible = live;
            if (!live) {
                lidarGeo.setDrawRange(0, 0);
                lidarRayGeo.setDrawRange(0, 0);
                return;
            }
            var n = Math.min(LIDAR_BINS, bins.length);
            var i, mm, m, nativeRad, rx, ry, v, deckY = 0.055;
            var shown = 0;
            for (i = 0; i < n; i++) {
                mm = bins[i];
                if (mm == null || mm <= 0) continue;
                m = mm / 1000;
                // API deg is OSD (lidar+180). Native 0° = camera-forward = THREE +Z.
                nativeRad = ((i * 10 + 5) - 180) * Math.PI / 180;
                rx = m * Math.cos(nativeRad);
                ry = m * Math.sin(nativeRad);
                v = robotToThree(rx, ry, deckY);
                lidarPos[shown * 3] = v.x;
                lidarPos[shown * 3 + 1] = v.y;
                lidarPos[shown * 3 + 2] = v.z;
                lidarRayPos[shown * 6] = 0;
                lidarRayPos[shown * 6 + 1] = deckY;
                lidarRayPos[shown * 6 + 2] = 0;
                lidarRayPos[shown * 6 + 3] = v.x;
                lidarRayPos[shown * 6 + 4] = v.y;
                lidarRayPos[shown * 6 + 5] = v.z;
                shown++;
            }
            lidarGeo.setDrawRange(0, shown);
            lidarRayGeo.setDrawRange(0, shown * 2);
            lidarGeo.attributes.position.needsUpdate = true;
            lidarRayGeo.attributes.position.needsUpdate = true;
            lidarGeo.computeBoundingSphere();
        }

        function drawLidarRadar(lidar) {
            var cv = document.getElementById('lidar-radar');
            if (!cv || !cv.getContext) return;
            var ctx = cv.getContext('2d');
            var w = cv.width;
            var h = cv.height;
            var cx = w / 2;
            var cy = h / 2;
            var R = Math.min(w, h) / 2 - 8;
            ctx.clearRect(0, 0, w, h);
            ctx.strokeStyle = 'rgba(56,189,248,0.22)';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();
            ctx.beginPath(); ctx.arc(cx, cy, R * 0.5, 0, Math.PI * 2); ctx.stroke();
            ctx.strokeStyle = '#38bdf8';
            ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - R); ctx.stroke();
            ctx.fillStyle = '#64748b';
            ctx.font = '9px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('fwd', cx, 12);
            var bins = lidar && lidar.bins_10deg_mm;
            if (!bins) return;
            var maxM = 2.5;
            if (lidar.max_mm) maxM = Math.max(1.0, lidar.max_mm / 1000);
            var i, mm, m, nativeRad, sx, sy;
            ctx.fillStyle = '#38bdf8';
            for (i = 0; i < bins.length; i++) {
                mm = bins[i];
                if (mm == null || mm <= 0) continue;
                m = Math.min(mm / 1000, maxM);
                nativeRad = ((i * 10 + 5) - 180) * Math.PI / 180;
                sx = cx - Math.sin(nativeRad) * (m / maxM) * R;
                sy = cy - Math.cos(nativeRad) * (m / maxM) * R;
                ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
            }
            if (lidar.nearest && lidar.nearest.mm != null) {
                nativeRad = (lidar.nearest.deg - 180) * Math.PI / 180;
                m = Math.min(lidar.nearest.mm / 1000, maxM);
                sx = cx - Math.sin(nativeRad) * (m / maxM) * R;
                sy = cy - Math.cos(nativeRad) * (m / maxM) * R;
                ctx.fillStyle = '#fbbf24';
                ctx.beginPath(); ctx.arc(sx, sy, 4, 0, Math.PI * 2); ctx.fill();
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
            if (lastFk) {
                var reachMm = Math.sqrt(lastFk.x * lastFk.x + lastFk.y * lastFk.y) * 1000;
                setText(hud.ee || 'ee-val',
                    reachMm.toFixed(0) + ' mm fwd · ' + (lastFk.zWorld * 1000).toFixed(0) + ' mm up');
            }
            var lidarHud = document.getElementById('hud-lidar');
            var L = state.lidar;
            var lidarShow = !!(L && (L.detected || L.enabled || L.open));
            if (lidarHud) lidarHud.style.display = lidarShow ? '' : 'none';
            if (lidarShow) {
                var lidarLabel = 'off';
                if (L.open) lidarLabel = 'live ' + (L.port ? L.port.replace(/^.*\//, '') : 'USB');
                else if (L.enabled && L.detected) lidarLabel = 'enabled, waiting';
                else if (L.enabled) lidarLabel = 'enabled, no USB';
                else if (L.detected) lidarLabel = 'USB detected';
                setText(hud.lidar || 'lidar-val', lidarLabel);
                var nearTxt = '—';
                if (L.nearest && L.nearest.mm != null) {
                    nearTxt = Math.round(L.nearest.mm) + ' mm';
                    if (L.nearest.deg != null) nearTxt += ' @ ' + Math.round(L.nearest.deg) + '°';
                } else if (L.min_mm != null) {
                    nearTxt = Math.round(L.min_mm) + '–' + Math.round(L.max_mm || L.min_mm) + ' mm';
                }
                setText(hud.lidarNear || 'lidar-near-val', nearTxt);
                drawLidarRadar(L);
            }
            var roarmHud = document.getElementById('hud-roarm');
            var ptzHud = document.getElementById('hud-ptz');
            if (roarmHud) roarmHud.style.display = state.attachment === 'roarm2' ? '' : 'none';
            if (ptzHud) ptzHud.style.display = state.attachment === 'ptz' ? '' : 'none';
            var dot = document.getElementById('status-dot');
            if (dot) {
                var live = state.attachment === 'roarm2' ? state.connected : state.pollOk;
                if (live) dot.classList.add('connected');
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
            state.pollOk = true;
            rebuildChassis();
            setAttachmentVisibility();
            if (chassisChanged || attachChanged) frameCamera();
            // Stick/T:102 preview wins for ~450ms so the poll cannot snap the arm back.
            if (d.joints && Date.now() >= localHoldUntil) applyJoints(d.joints, true);
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
            applyLidarScan(d.lidar);
            updateHud();
        }

        function poll() {
            if (paused) return;
            fetch('/api/twin')
                .then(function (r) { return r.json(); })
                .then(applySnapshot)
                .catch(function () { state.pollOk = false; updateHud(); });
        }

        function resize() {
            var vs = viewSize();
            camera.aspect = vs.w / vs.h;
            camera.updateProjectionMatrix();
            renderer.setSize(vs.w, vs.h, false);
        }

        function animate() {
            if (!running || paused) return;
            requestAnimationFrame(animate);
            if (controls) controls.update();
            renderer.render(scene, camera);
        }

        function setPaused(p) {
            var next = !!p;
            if (next === paused) return;
            paused = next;
            if (!paused) animate();
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
            setJoints: function (j) { applyJoints(j, false); },
            setFromEZR: function (e, z, r, hand) {
                applyJoints(eZrToJoints(e, z, r, {
                    e: defaults.e, z: defaults.z, r: defaults.r,
                    hand: hand != null ? hand : defaults.hand
                }), false);
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
            setPaused: setPaused,
            destroy: function () {
                running = false;
                if (pollTimer) clearInterval(pollTimer);
                if (ro) ro.disconnect();
                window.removeEventListener('resize', resize);
                if (showRings) {
                    renderer.domElement.removeEventListener('pointerdown', onRingDown);
                    window.removeEventListener('pointermove', onRingMove);
                    window.removeEventListener('pointerup', onRingUp);
                    window.removeEventListener('pointercancel', onRingUp);
                }
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
