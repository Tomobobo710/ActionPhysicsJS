var AP = require('../build/actionphysics.js');
var SIZE=10, LAYER_GAP=2.2;
var w = new AP.World(new AP.SAPBroadphase(), new AP.NarrowPhase(), new AP.Solver());
w.gravity = new AP.Vector3(0, -9.8, 0);
var floor = new AP.RigidBody(new AP.BoxShape(20, 0.5, 20), 0);
floor.position.set(0, -0.5, 0); floor.updateDerived(); w.addRigidBody(floor);
var boxes=[];
for (var i = 0; i < SIZE; i++) for (var j = 0; j < SIZE - i; j++) for (var k = 0; k < SIZE - i; k++) {
  var x = 2*j*1.3 - SIZE + i*1.2, y = i*LAYER_GAP + 1, z = 2*k*1.3 - SIZE + i*1.2;
  var b = new AP.RigidBody(new AP.BoxShape(1,1,1), 1);
  b.position.set(x,y,z); b.updateDerived(); w.addRigidBody(b);
  boxes.push({b:b,layer:i,j:j,k:k,x0:x,y0:y,z0:z});
}
var target=null; boxes.forEach(function(o){ if(o.layer===0&&o.j===3&&o.k===4) target=o; });
function rotDeg(bd){var w=Math.abs(bd.rotation.w);if(w>1)w=1;return 2*Math.acos(w)*180/Math.PI;}
for (var t=0;t<20;t++){
  w.step(1/60);
  var av=target.b.angular_velocity, v=target.b.linear_velocity;
  // count how many manifolds touching the floor are ACTIVE (lambda<0) system-wide
  var activeFloorContacts=0, totalManifolds=0;
  for (var m of w.narrowphase.manifolds.values()) {
    totalManifolds++;
    if (m.bodyA.id===floor.id || m.bodyB.id===floor.id) {
      var eng=0;
      m.points.forEach(function(p){ if(p.normalLambda<0) eng++; });
      if (eng>0) activeFloorContacts++;
    }
  }
  console.log('t='+t,'target|w|='+Math.hypot(av.x,av.y,av.z).toFixed(6),'activeFloorContacts='+activeFloorContacts,'totalManifolds='+totalManifolds);
}
