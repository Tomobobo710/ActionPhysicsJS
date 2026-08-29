/**
 * ActionPhysics rendering adapter: THREE.js  (reference implementation)
 *
 * The harness (render.js) is renderer-agnostic — it owns the runner, the honesty contract, the timeline
 * console, the HUD, and the fixed-timestep stepping. Everything that actually DRAWS lives behind an adapter
 * with this interface, so a host application can supply its own renderer while the harness,
 * UI, and test format stay identical.
 *
 * Adapter interface (all optional except init/clear/render/addBody/syncBody):
 *   init(viewportEl)            -> set up the renderer inside the viewport element
 *   addBody(body)   -> drawable  create a drawable for a physics body (reads body.shape / .position / .rotation)
 *   syncBody(drawable, body)     copy the body's transform onto its drawable (called every frame)
 *   frameCamera(bodies)          aim the camera at the bodies' centroid
 *   clear()                      remove all drawables + debug extras
 *   render()                     draw one frame (called from the harness loop)
 *   drawRay(ray) / drawSwept(sw) / drawSupport(sup)   optional geometry-test debug draws
 *   resize()                     viewport resized
 *
 * This adapter draws ACTIONPHYSICS bodies (body.shape is an ActionPhysics shape; body.position and
 * body.rotation are its vector/quaternion). Everything the renderer knows about shapes lives here, so
 * swapping in a different renderer means writing one of these and nothing else.
 */
(function () {
	var AP = window.ActionPhysics;
	var R = null; // three.js state: { renderer, scene, camera, controls, meshes:[], extras:[] }

	// Self-contained UP-LOCKED orbit controller. Drag to orbit around `target`; world up (+Y) stays up so
	// the horizon is always level. Wheel zooms. Same .update()/.target interface the loop expects.
	function makeOrbit(camera, dom) {
		var target = new THREE.Vector3(0, 0, 0);
		var off = camera.position.clone().sub(target);
		var radius = off.length();
		var yaw = Math.atan2(off.x, off.z);
		var pitch = Math.asin(Math.max(-1, Math.min(1, off.y / radius)));
		var dragging = false, lx = 0, ly = 0;
		var PITCH_LIMIT = Math.PI / 2 - 0.05;
		dom.addEventListener('mousedown', function (e) { dragging = true; lx = e.clientX; ly = e.clientY; });
		window.addEventListener('mouseup', function () { dragging = false; });
		window.addEventListener('mousemove', function (e) {
			if (!dragging) return;
			var dx = e.clientX - lx, dy = e.clientY - ly; lx = e.clientX; ly = e.clientY;
			yaw -= dx * 0.005;
			pitch = Math.max(-PITCH_LIMIT, Math.min(PITCH_LIMIT, pitch + dy * 0.005));
		});
		dom.addEventListener('wheel', function (e) {
			e.preventDefault();
			radius *= (1 + (e.deltaY > 0 ? 0.1 : -0.1));
			radius = Math.max(2, Math.min(200, radius));
		}, { passive: false });
		return {
			target: target,
			update: function () {
				var cp = Math.cos(pitch);
				camera.position.set(
					target.x + radius * cp * Math.sin(yaw),
					target.y + radius * Math.sin(pitch),
					target.z + radius * cp * Math.cos(yaw)
				);
				camera.up.set(0, 1, 0);
				camera.lookAt(target);
			}
		};
	}

	function init(vp) {
		if (R) return R;
		var renderer = new THREE.WebGLRenderer({ antialias: true });
		renderer.setClearColor(0x05070a);
		renderer.setSize(vp.clientWidth, vp.clientHeight);
		vp.appendChild(renderer.domElement);
		var scene = new THREE.Scene();
		var camera = new THREE.PerspectiveCamera(45, vp.clientWidth / vp.clientHeight, 0.1, 2000);
		camera.position.set(9, 7, 16);
		var controls = makeOrbit(camera, renderer.domElement);
		scene.add(new THREE.AmbientLight(0x999999));
		var dir = new THREE.DirectionalLight(0xffffff, 0.7); dir.position.set(6, 12, 8); scene.add(dir);
		scene.add(new THREE.GridHelper(40, 40, 0x223344, 0x151f28));
		R = { renderer: renderer, scene: scene, camera: camera, controls: controls, meshes: [], extras: [], _vp: vp };
		return R;
	}
	function resize() {
		if (!R) return; var vp = R._vp;
		if (!vp.clientWidth) return;
		R.renderer.setSize(vp.clientWidth, vp.clientHeight);
		R.camera.aspect = vp.clientWidth / vp.clientHeight; R.camera.updateProjectionMatrix();
	}

	// Geometry for one primitive shape. Every ActionPhysics shape the tests use is handled here.
	// Property names here must track src/shapes/*.js exactly — this file has no authority of its own.
	function geoForShape(s) {
		if (s instanceof AP.SphereShape) return new THREE.SphereGeometry(s.radius, 24, 16);
		if (s instanceof AP.BoxShape) return new THREE.BoxGeometry(s.halfWidth * 2, s.halfHeight * 2, s.halfDepth * 2);
		if (s instanceof AP.CylinderShape) return new THREE.CylinderGeometry(s.radius, s.radius, s.halfHeight * 2, 24);
		if (s instanceof AP.ConeShape) return new THREE.CylinderGeometry(0, s.radius, s.halfHeight * 2, 24);
		if (s instanceof AP.PlaneShape) {
			// Zero-thickness by construction; draw a thin slab so it's visible. halfW/halfL run
			// along the two axes perpendicular to orientation, same cyclic convention as the shape.
			var thin = 0.02;
			if (s.orientation === 'x') return new THREE.BoxGeometry(thin, s.halfW * 2, s.halfL * 2);
			if (s.orientation === 'y') return new THREE.BoxGeometry(s.halfL * 2, thin, s.halfW * 2);
			return new THREE.BoxGeometry(s.halfW * 2, s.halfL * 2, thin);
		}
		if (s instanceof AP.TriangleShape) {
			var g3 = new THREE.Geometry();
			g3.vertices.push(new THREE.Vector3(s.a.x, s.a.y, s.a.z), new THREE.Vector3(s.b.x, s.b.y, s.b.z), new THREE.Vector3(s.c.x, s.c.y, s.c.z));
			g3.faces.push(new THREE.Face3(0, 1, 2));
			g3.computeFaceNormals();
			return g3;
		}
		if (s instanceof AP.LineSweptShape) return geoForShape(s.shape); // approximate: draw the base shape only
		if (s instanceof AP.ConvexShape) {
			var verts = s.points.map(function (v) { return new THREE.Vector3(v.x, v.y, v.z); });
			return new THREE.ConvexGeometry(verts);
		}
		if (s instanceof AP.MeshShape) {
			var g = new THREE.Geometry();
			s.vertices.forEach(function (v) { g.vertices.push(new THREE.Vector3(v.x, v.y, v.z)); });
			for (var i = 0; i < s.triangleCount; i++) {
				var base = i * 3;
				g.faces.push(new THREE.Face3(s.indices[base], s.indices[base + 1], s.indices[base + 2]));
			}
			g.computeFaceNormals();
			return g;
		}
	}
	function litMesh(geo, color) {
		var mesh = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: color, transparent: true, opacity: 0.82 }));
		mesh.add(new THREE.Mesh(geo.clone(), new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true, transparent: true, opacity: 0.12 })));
		return mesh;
	}
	function meshForBody(b) {
		var s = b.shape, color = b._color ? parseInt(b._color.replace('#', '0x')) : 0x4488ff;
		if (s instanceof AP.CompoundShape) {
			var grp = new THREE.Group();
			s.children.forEach(function (child) {
				var cm = litMesh(geoForShape(child.shape), color);
				cm.position.set(child.localPosition.x, child.localPosition.y, child.localPosition.z);
				cm.quaternion.set(child.localRotation.x, child.localRotation.y, child.localRotation.z, child.localRotation.w);
				grp.add(cm);
			});
			return grp;
		}
		if (s instanceof AP.CapsuleShape) {
			var cap = new THREE.Group();
			cap.add(litMesh(new THREE.CylinderGeometry(s.radius, s.radius, s.segmentHalfLength * 2, 24), color));
			var half = Math.PI / 2;
			var topCap = litMesh(new THREE.SphereGeometry(s.radius, 24, 12, 0, Math.PI * 2, 0, half), color);
			topCap.position.y = s.segmentHalfLength; cap.add(topCap);
			var botCap = litMesh(new THREE.SphereGeometry(s.radius, 24, 12, 0, Math.PI * 2, half, half), color);
			botCap.position.y = -s.segmentHalfLength; cap.add(botCap);
			return cap;
		}
		return litMesh(geoForShape(s), color);
	}

	// ---- interface ----
	function addBody(b) {
		var m = meshForBody(b); m._b = b; R.meshes.push(m); R.scene.add(m); syncBody(m, b); return m;
	}
	function syncBody(m, b) {
		m.position.set(b.position.x, b.position.y, b.position.z);
		m.quaternion.set(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w);
	}
	function clear() {
		if (!R) return;
		R.meshes.forEach(function (m) { R.scene.remove(m); }); R.meshes = [];
		R.extras.forEach(function (m) { R.scene.remove(m); }); R.extras = [];
	}
	// Remove a single drawable (the mesh returned by addBody) — used when a body is destroyed mid-sim
	// (e.g. the controller rebuilds its collider on crouch, so the tall one must stop being drawn).
	function removeBody(m) {
		if (!R || !m) return;
		R.scene.remove(m);
		var i = R.meshes.indexOf(m); if (i !== -1) R.meshes.splice(i, 1);
	}
	function frameCamera(bodies) {
		if (!R || !bodies.length) return;
		var cx = 0, cy = 0, cz = 0; bodies.forEach(function (b) { cx += b.position.x; cy += b.position.y; cz += b.position.z; });
		R.controls.target.set(cx / bodies.length, cy / bodies.length, cz / bodies.length);
	}
	function render() {
		if (!R) return;
		R.controls.update();
		R.renderer.render(R.scene, R.camera);
	}

	// ---- optional geometry-test debug draws ----
	function drawRay(ray) {
		var g = new THREE.Geometry();
		g.vertices.push(new THREE.Vector3(ray.start[0], ray.start[1], ray.start[2]), new THREE.Vector3(ray.stop[0], ray.stop[1], ray.stop[2]));
		var line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x58a6ff })); R.extras.push(line); R.scene.add(line);
		if (ray.hit) { var hm = new THREE.Mesh(new THREE.SphereGeometry(0.18, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff5555 })); hm.position.set(ray.hit[0], ray.hit[1], ray.hit[2]); R.extras.push(hm); R.scene.add(hm); }
	}
	function drawSwept(sw) {
		[sw.start, sw.end].forEach(function (pt) {
			var m = litMesh(geoForShape(sw.shape), 0x45b7d1);
			m.material.opacity = 0.4;
			m.position.set(pt[0], pt[1], pt[2]);
			R.extras.push(m); R.scene.add(m);
		});
		var g = new THREE.Geometry();
		g.vertices.push(new THREE.Vector3(sw.start[0], sw.start[1], sw.start[2]), new THREE.Vector3(sw.end[0], sw.end[1], sw.end[2]));
		var line = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x9fe7ff }));
		R.extras.push(line); R.scene.add(line);
	}
	function drawSupport(sup) {
		var p = new THREE.Vector3(sup.point[0], sup.point[1], sup.point[2]);
		var dir = new THREE.Vector3(sup.dir[0], sup.dir[1], sup.dir[2]).normalize();
		var arrow = new THREE.ArrowHelper(dir, p.clone().sub(dir.clone().multiplyScalar(3)), 3, 0x3fb950, 0.6, 0.35);
		R.extras.push(arrow); R.scene.add(arrow);
		var m = new THREE.Mesh(new THREE.SphereGeometry(0.18, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff5555 }));
		m.position.copy(p); R.extras.push(m); R.scene.add(m);
	}

	window.APThreeAdapter = {
		name: 'three',
		init: init, resize: resize,
		addBody: addBody, syncBody: syncBody, removeBody: removeBody, clear: clear, frameCamera: frameCamera, render: render,
		drawRay: drawRay, drawSwept: drawSwept, drawSupport: drawSupport
	};
})();
