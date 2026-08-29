(function (Runner) {
	Runner.suite('collision');
	var DESC = "The engine reports contact lifecycle events on a body: a predicted 'speculativeContact' " +
		"(which a listener can veto by returning false), a real 'contact' when surfaces touch, and an " +
		"'endContact' when they separate. A ball is fired down at a fixed sphere: the first predicted " +
		"contact is vetoed, the real contact then fires after exactly two speculative contacts, and as " +
		"the ball bounces away an endContact fires.";

	Runner.test('collision/contact-events', 'speculative prevention + contact + endContact fire correctly', function (t) {
		var w = t.makeWorld({ gravity: 0 });
		var s1 = t.sphere(w, 1, 0, { color: '#888', restitution: 1 });
		var s2 = t.sphere(w, 1, 1, { pos: [0, 5, 0], vel: [0, -10, 0], color: '#45B7D1', restitution: 1 });

		var seenSpeculative = 0, hasPreventedOne = false, firstContact = null, endContactFired = false;
		s1.addListener('speculativeContact', function () {
			seenSpeculative += 1;
			if (hasPreventedOne === false) { hasPreventedOne = true; return false; }
		});
		s1.addListener('contact', function () { if (firstContact === null) firstContact = { prevented: hasPreventedOne, seen: seenSpeculative }; });
		s1.addListener('endContact', function () { endContactFired = true; });

		t.expect('first predicted contact is prevented, then contact fires after 1 speculative contact', function () {
			return { ok: firstContact !== null && firstContact.prevented === true && firstContact.seen === 1,
				detail: firstContact ? ('contact seen after ' + firstContact.seen + ' speculative') : 'waiting for contact' };
		});
		t.expect('an endContact fires as the ball separates', function () {
			return { ok: endContactFired, detail: endContactFired ? 'endContact fired' : 'waiting for separation' };
		});

		t.simulate(w, 120);
	}, { visual: true, steps: 120, page: 'contact-events', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
