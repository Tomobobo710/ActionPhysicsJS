(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var V = AP.Vector3;
	var DESC = "Flattened-array BVH: parallel typed arrays for node bounds plus left/right/leafIndex " +
		"indices, no pointer chasing. Median-split on the widest axis at build time. query() visits " +
		"every leaf whose node bound intersects the query box, with no false negatives.";
	function test(group, name, fn, opts) {
		opts = opts || {};
		Runner.test(group, name, fn, { page: 'bvh', description: DESC, visual: !!opts.visual, steps: 0 });
	}

	function buildFromBoxes(boxes) {
		var bvh = new AP.BVH();
		bvh.build(boxes.length, function (out, i) {
			out.setFromMinMax(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3], boxes[i][4], boxes[i][5]);
		});
		return bvh;
	}

	function queryHits(bvh, box) {
		var hits = [];
		bvh.query(new AP.AABB().setFromMinMax(box[0], box[1], box[2], box[3], box[4], box[5]), function (i) { hits.push(i); });
		hits.sort(function (a, b) { return a - b; });
		return hits;
	}

	test('collision/bvh', 'an empty tree has no root and yields no query hits', function (t) {
		var bvh = new AP.BVH();
		bvh.build(0, function () {});
		t.checkEqual(bvh.root, -1, 'root is -1 for zero leaves');
		var hits = queryHits(bvh, [-1, -1, -1, 1, 1, 1]);
		t.checkEqual(hits.length, 0, 'querying an empty tree visits nothing');
	});

	test('collision/bvh', 'a single-leaf tree reports that leaf for any overlapping query', function (t) {
		var bvh = buildFromBoxes([[0, 0, 0, 1, 1, 1]]);
		t.checkEqual(queryHits(bvh, [0.5, 0.5, 0.5, 2, 2, 2]).length, 1, 'overlapping query hits the one leaf');
		t.checkEqual(queryHits(bvh, [5, 5, 5, 6, 6, 6]).length, 0, 'non-overlapping query hits nothing');
	});

	test('collision/bvh', 'query returns exactly the leaves whose box overlaps, no false negatives', function (t) {
		var boxes = [
			[0, 0, 0, 1, 1, 1],     // 0
			[2, 0, 0, 3, 1, 1],     // 1 - separate
			[0.5, 0.5, 0.5, 1.5, 1.5, 1.5], // 2 - overlaps 0
			[10, 10, 10, 11, 11, 11] // 3 - far away
		];
		var bvh = buildFromBoxes(boxes);
		var hits = queryHits(bvh, [0, 0, 0, 1, 1, 1]);
		t.checkEqual(hits.length, 2, 'exactly leaves 0 and 2 overlap the query box');
		t.checkTrue(hits.indexOf(0) !== -1 && hits.indexOf(2) !== -1, 'both true overlaps are present');
		t.checkTrue(hits.indexOf(1) === -1 && hits.indexOf(3) === -1, 'non-overlapping leaves are excluded');
	});

	test('collision/bvh', 'a query box containing every leaf hits all of them', function (t) {
		var boxes = [];
		for (var i = 0; i < 20; i++) boxes.push([i, 0, 0, i + 0.5, 1, 1]);
		var bvh = buildFromBoxes(boxes);
		var hits = queryHits(bvh, [-1, -1, -1, 21, 2, 2]);
		t.checkEqual(hits.length, 20, 'every leaf reported when the query covers the whole set');
	});

	test('collision/bvh', 'node bounds are the true union of their leaves (root covers everything)', function (t) {
		var boxes = [[0, 0, 0, 1, 1, 1], [5, -3, 2, 6, -2, 3], [-4, 4, -4, -3, 5, -3]];
		var bvh = buildFromBoxes(boxes);
		var root = bvh.root;
		t.check(bvh.minX[root], -4, 1e-9, 'root minX is the true minimum across all leaves');
		t.check(bvh.maxY[root], 5, 1e-9, 'root maxY is the true maximum across all leaves');
	});

	test('collision/bvh', 'a larger set stays correct (stress: 200 random-ish leaves)', function (t) {
		var boxes = [];
		for (var i = 0; i < 200; i++) {
			var x = (i * 37) % 100, y = (i * 53) % 100, z = (i * 71) % 100;
			boxes.push([x, y, z, x + 1, y + 1, z + 1]);
		}
		var bvh = buildFromBoxes(boxes);
		// Brute-force reference for one query box.
		var q = [10, 10, 10, 15, 15, 15];
		var expected = [];
		for (var j = 0; j < boxes.length; j++) {
			var b = boxes[j];
			if (b[0] <= q[3] && b[3] >= q[0] && b[1] <= q[4] && b[4] >= q[1] && b[2] <= q[5] && b[5] >= q[2]) expected.push(j);
		}
		var hits = queryHits(bvh, q);
		t.checkEqual(hits.length, expected.length, 'BVH query count matches brute-force over 200 leaves');
	});

	// ---- visual: draw the leaf boxes and the query box as bodies (boxes drawn via BoxShape) ----

	test('collision/bvh', 'query result over a small scattered set of leaf boxes', function (t) {
		var boxes = [[0, 0, 0, 1, 1, 1], [3, 0, 0, 4, 1, 1], [0.5, 2, 0, 1.5, 3, 1], [-3, -3, -3, -2, -2, -2]];
		var bvh = buildFromBoxes(boxes);
		boxes.forEach(function (b, i) {
			var cx = (b[0] + b[3]) / 2, cy = (b[1] + b[4]) / 2, cz = (b[2] + b[5]) / 2;
			var hx = (b[3] - b[0]) / 2, hy = (b[4] - b[1]) / 2, hz = (b[5] - b[2]) / 2;
			t.bodies.push({ shape: new AP.BoxShape(hx, hy, hz), position: new V(cx, cy, cz), rotation: new AP.Quaternion(), _color: '#4af' });
		});
		var hits = queryHits(bvh, [0, 0, 0, 1, 1, 1]);
		t.checkEqual(hits.length, 1, 'only leaf 0 overlaps the query box');
	}, { visual: true });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
