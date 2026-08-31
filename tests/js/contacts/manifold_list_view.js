(function (Runner) {
	var DESC = "NarrowPhase exposes the tick's manifolds two ways that must agree: the canonical " +
		"World 'contacts' event (an array via the ContactManifoldList), and a singly-linked-list " +
		"view walked as narrowphase.contact_manifolds.first -> .next_manifold. Both must yield the " +
		"same manifolds in the same order. A small stack of boxes on a floor produces several " +
		"persistent manifolds to compare.";

	Runner.test('collision/manifold-list-view', 'contact_manifolds linked-list view matches the contacts event', function (t) {
		var w = t.makeWorld();
		t.box(w, 10, 0.5, 10, 0, { pos: [0, -0.5, 0], color: '#243B2A' });
		t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 0.5, 0], color: '#4af' });
		t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 1.5, 0], color: '#f84' });
		t.box(w, 0.5, 0.5, 0.5, 1, { pos: [0, 2.5, 0], color: '#8f4' });

		var np = w.narrowphase;
		var lastEventList = null;
		w.addListener('contacts', function (list) { lastEventList = list; });

		function walkView() {
			var out = [];
			for (var m = np.contact_manifolds.first; m; m = m.next_manifold) out.push(m);
			return out;
		}

		t.expect('narrowphase.contact_manifolds is the same object the event delivers', function () {
			return { ok: lastEventList != null && lastEventList === np.contact_manifolds && lastEventList === np.manifolds,
				detail: lastEventList ? 'same identity' : 'no event yet' };
		});

		t.expect('the linked-list view and values() yield identical manifolds in identical order', function () {
			if (lastEventList == null) return { ok: false, detail: 'no event yet' };
			var fromValues = Array.from(lastEventList.values());
			var fromView = walkView();
			if (fromView.length === 0) return { ok: false, detail: 'view empty' };
			if (fromView.length !== fromValues.length) {
				return { ok: false, detail: 'view=' + fromView.length + ' values=' + fromValues.length };
			}
			for (var i = 0; i < fromView.length; i++) {
				if (fromView[i] !== fromValues[i]) return { ok: false, detail: 'mismatch at index ' + i };
			}
			return { ok: true, detail: fromView.length + ' manifolds agree' };
		});

		t.expect('the last node of the view has next_manifold === null (chain is terminated)', function () {
			var view = walkView();
			if (view.length === 0) return { ok: false, detail: 'view empty' };
			return { ok: view[view.length - 1].next_manifold === null, detail: 'tail terminated' };
		});

		t.expect('every node in the view carries live points (empty manifolds were pruned before relink)', function () {
			var view = walkView();
			if (view.length === 0) return { ok: false, detail: 'view empty' };
			for (var i = 0; i < view.length; i++) {
				if (view[i].pointCount === 0) return { ok: false, detail: 'node ' + i + ' has 0 points' };
			}
			return { ok: true, detail: 'all ' + view.length + ' nodes non-empty' };
		});

		t.simulate(w, 90);
	}, { visual: true, steps: 90, page: 'manifold-list-view', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
