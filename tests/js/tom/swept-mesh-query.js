(function (Runner, U) {
	Runner.suite('tom');

	function cubeMesh(H) {
		var v = [
			[-H,-H,-H],[ H,-H,-H],[ H, H,-H],[-H, H,-H],
			[-H,-H, H],[ H,-H, H],[ H, H, H],[-H, H, H]
		];
		var f = [
			0,1,2, 0,2,3,
			4,6,5, 4,7,6,
			0,4,5, 0,5,1,
			3,2,6, 3,6,7,
			0,3,7, 0,7,4,
			1,5,6, 1,6,2
		];
		return { v: v, f: f };
	}

	Runner.test('swept-mesh-query', 'shapeIntersect sees a static mesh', function (t) {
		t.log('Sweep a box straight through a static cube MESH and assert shapeIntersect returns it.');

		var w = t.makeWorld({ gravity: 0 });
		var H = 1;
		var cube = cubeMesh(H);

		var meshBody = t.mesh(w, cube.v, cube.f, 0, U.withMat({ pos: [0, 0, 0], color: '#889' }));

		var box = new t.AP.BoxShape(0.3, 0.3, 0.3);
		var start = t.vec(-3, 0, 0), end = t.vec(3, 0, 0);

		t.expect('swept box returns the static mesh as a hit', function () {
			var hit = w.shapeIntersect(box, start, end);
			var ok = false, detail = 'hit=' + (hit ? (hit.body === meshBody ? 'mesh' : 'other') : 'none');
			if (hit && hit.body === meshBody) {
				var n = hit.normal, nlen = Math.sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
				ok = nlen > 0.5 && isFinite(nlen);
				detail += ' normal=[' + n.x.toFixed(2) + ',' + n.y.toFixed(2) + ',' + n.z.toFixed(2) + '] |n|=' + nlen.toFixed(2);
			}
			return { ok: ok, detail: detail };
		});

		t.simulate(w, 2);
	}, {
		visual: true, steps: 0, page: 'swept-mesh-query',
		description:
			"World.shapeIntersect (the swept-box query used for collide-and-slide) must return a MeshShape " +
			"body it sweeps through. The regression: mesh/compound contacts route through the addContact " +
			"callback and getContact returns undefined, so shapeIntersect saw zero mesh hits and a swept " +
			"body would pass through static meshes. PASS: a box swept through a static cube mesh returns " +
			"at least one hit whose object is that mesh, with a sane surface normal."
	});
})(
	typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner,
	typeof module !== 'undefined' && module.exports ? require('./_util.js') : window.TomUtil
);
