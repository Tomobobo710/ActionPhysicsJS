(function (Runner, PBF, ActionPhysics) {

	var Vector3 = ActionPhysics.Vector3, RigidBody = ActionPhysics.RigidBody,
		BoxShape = ActionPhysics.BoxShape, MeshShape = ActionPhysics.MeshShape;

	function boxMesh(hx, hy, hz, sign) {
		var xs = [-1, 1], ys = [-1, 1], zs = [-1, 1], v = [];
		for (var yi = 0; yi < 2; yi++) for (var zi = 0; zi < 2; zi++) for (var xi = 0; xi < 2; xi++)
			v.push(new Vector3(xs[xi] * hx, ys[yi] * hy, zs[zi] * hz));
		var I = function (xi, yi, zi) { return yi * 4 + zi * 2 + xi; };
		var faces = [];
		function quad(p, rev) {
			var f = rev ? [p[0], p[2], p[1], p[0], p[3], p[2]] : [p[0], p[1], p[2], p[0], p[2], p[3]];
			for (var k = 0; k < 6; k++) faces.push(f[k]);
		}
		quad([I(0, 1, 0), I(0, 1, 1), I(1, 1, 1), I(1, 1, 0)], sign < 0);
		quad([I(0, 0, 0), I(1, 0, 0), I(1, 0, 1), I(0, 0, 1)], sign < 0);
		quad([I(1, 0, 0), I(1, 0, 1), I(1, 1, 1), I(1, 1, 0)], sign > 0);
		quad([I(0, 0, 0), I(0, 1, 0), I(0, 1, 1), I(0, 0, 1)], sign > 0);
		quad([I(0, 0, 1), I(1, 0, 1), I(1, 1, 1), I(0, 1, 1)], sign < 0);
		quad([I(0, 0, 0), I(0, 1, 0), I(1, 1, 0), I(1, 0, 0)], sign < 0);
		return new MeshShape(v, faces);
	}

	function stack(S, sign) {
		var h = S.sc(1.2);
		var w = PBF.makeWorld();
		var inner = new RigidBody(new BoxShape(h, h, h), 0);
		inner.position.set(0, h, 0);
		inner.updateDerived();
		w.addRigidBody(inner);
		var outH = 1.1 * h;
		var shell = new RigidBody(boxMesh(outH, outH, outH, sign), 0);
		shell.position.set(0, h, 0);
		shell.updateDerived();
		w.addRigidBody(shell);
		var shellTop = h + outH;
		var p = PBF.spawn(w, { x: 0, y: shellTop + 5, z: 0 }, { scale: S.SC });
		return { w: w, p: p, shellTop: shellTop, inner: inner, shell: shell };
	}

	PBF.scaleTest('fps/ground-shell', 'GS1', 'character stands on a reversed-winding box mesh', function (t, S) {
		var s = stack(S, -1);
		PBF.renderables(t, s.p, [s.inner, s.shell]);
		var w = s.w, p = s.p;
		PBF.drive(t, p, function () { return {}; });
		t.expect('character is grounded on the shell top', function () {
			if (!p.grounded) return false;
			var feet = p.body.position.y - p.height / 2;
			return { ok: Math.abs(feet - s.shellTop) < S.sc(0.05), detail: 'feet=' + feet.toFixed(3) + ' shellTop=' + s.shellTop.toFixed(3) };
		});
		t.simulate(w, 300);
	}, { page: 'fps/ground-shell', steps: 300, description: 'Drop the character onto a reversed-winding box mesh; it should rest grounded on the top.' });
})(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('../_util_fps.js') : window.PBF,
	typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics);
