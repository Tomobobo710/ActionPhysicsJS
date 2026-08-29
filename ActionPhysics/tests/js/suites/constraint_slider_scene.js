(function (Runner) {
	Runner.suite('collision');
	var AP = typeof module !== 'undefined' && module.exports ? require('../../../build/actionphysics.js') : window.ActionPhysics;
	var DESC = "A tall base box and a smaller top box are linked by a SliderConstraint along the " +
		"world Y axis and dropped onto a ground plane. The slider constrains perpendicular motion " +
		"while allowing vertical sliding.";

	Runner.test('collision/constraint-slider-scene', 'two boxes linked by a slider constraint stay stable', function (t) {
		var world = t.makeWorld();
		t.box(world, 20, 0.5, 20, 0, { pos: [0, -3, 0], color: '#243B2A' });
		var base = t.box(world, 1, 5, 1, 10, { pos: [0, 5, 0], color: '#B08968' });
		var top = t.box(world, 1, 2, 1, 10, { pos: [3, 12, 0], color: '#4af' });

		var slider = new AP.SliderConstraint(base, new AP.Vector3(0, 1, 0), new AP.Vector3(1.5, 0, 0), top, new AP.Vector3(-1.5, 0, 0));
		world.addConstraint(slider);

		var ticks = 0, everNonFinite = false, maxHeight = 0;
		t.onTick(function (world, tick) {
			ticks = tick;
			if (!isFinite(base.position.y) || !isFinite(top.position.y) || !isFinite(base.position.x) || !isFinite(top.position.x)) { 
				everNonFinite = true; 
				return; 
			}
			if (top.position.y > maxHeight) maxHeight = top.position.y;
			
			if (tick === 1 || tick === 10 || tick === 50 || tick === 100 || tick === 200 || tick === 300) {
				var dx = top.position.x - base.position.x;
				var dy = top.position.y - base.position.y;
				t.log('Tick ' + tick + ': base=[' + base.position.x.toFixed(2) + ',' + base.position.y.toFixed(2) + '], top=[' + top.position.x.toFixed(2) + ',' + top.position.y.toFixed(2) + '], dx=' + dx.toFixed(2) + ', dy=' + dy.toFixed(2));
			}
		});
		
		t.expect('the scene stays numerically finite for 300 ticks', function () {
			if (ticks < 300) return false;
			return { ok: !everNonFinite, detail: 'no NaN/Infinity' };
		});
		
		t.expect('boxes dont launch into the stratosphere', function () {
			if (ticks < 300) return false;
			return { ok: maxHeight < 50, detail: 'max top.y=' + maxHeight.toFixed(2) };
		});
		
		t.expect('the 3-unit X offset is maintained', function () {
			if (ticks < 100) return false;
			var dx = top.position.x - base.position.x;
			return { ok: Math.abs(dx - 3) < 0.1, detail: 'final dx=' + dx.toFixed(4) + ', should be ~3' };
		});

		t.simulate(world, 300);
	}, { visual: true, steps: 300, page: 'constraint-slider-scene', description: DESC });

}(typeof module !== 'undefined' && module.exports ? require('../runner.js') : window.APRunner));
