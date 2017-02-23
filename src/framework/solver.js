const BroadPhase = require("../collision/broadphase.js");
const {ManifoldMap} = require("../collision/manifold.js");

module.exports = class Solver {
	constructor() {
		this.applyG = null;
		this.bodies = new Set();
		this.joints = new Set();
		this.manifolds = new ManifoldMap();
		this.broadPhase = new BroadPhase();
	}
	solve(dt) {
		if (this.applyG) {
			this.applyG([...this.bodies]);
		}

		solveBroadPhase.call(this);
		solveNarrowPhase.call(this);
		applyForces.call(this, dt);
		solveVelocities.call(this, dt);
		solvePositions.call(this, dt);
		clearForces.call(this);
	}
	addBody(body) {
		this.bodies.add(body);
		for (let shape of body.shapes) {
			this.broadPhase.insert(shape);
		}
	}
	removeBody(body) {
		this.bodies.delete(body);
		for (let shape of body.shapes) {
			this.broadPhase.remove(shape);
		}
	}
	addJoint(joint) {
		this.joints.add(joint);
	}
	removeJoint(joint) {
		this.joints.delete(joint);
	}
	query(aabb, callback) {
		this.broadPhase.query(aabb, callback);
	}
	debugGetNodes() {
		return this.broadPhase.debugGetNodes();
	}
};

//private functions, call with function.prototype.call
function solveBroadPhase() {
	for (let body of this.bodies) {
		for (let shape of body.shapes) {
			shape.setAABB();
		}
	}
	//the broadphase returns a set of unique pairs whose aabbs are overlapping
	let pairs = this.broadPhase.getPairs();
	//the manifold map will persist any manifolds that already exist,
	//and create new manifolds for pairs which don't have one yet
	for (let pair of pairs) {
		this.manifolds.add(pair);
	}
}

function solveNarrowPhase() {
	//here we iterate through the manifolds, solving the narrowphase
	//any that are not collided are removed from the map
	for (let manifold of this.manifolds) {
		manifold.solve();
		if (!manifold.isCollided) {
			this.manifolds.delete(manifold);
		}
	}
}

function applyForces(dt) {
	for (let body of this.bodies) {
		body.velocity.add(body.force.times(body.mass.iM).times(dt));
		body.angularVelocity += body.torque * body.mass.iI * dt;
	}
}

function solveVelocities(dt) {
	for (let manifold of this.manifolds) {
		manifold.initialize();
	}
	for (let manifold of this.manifolds) {
		manifold.warmStart();
	}
	for (let joint of this.joints) {
		joint.initialize(dt);
	}
	for (let i = 0; i < 8; i++) {
		for (let joint of this.joints) {
			joint.applyImpulse(dt);
		}
		for (let manifold of this.manifolds) {
			manifold.applyImpulse();
		}
	}
}

function solvePositions(dt) {
	for (let body of this.bodies) {
		body.prevPos = body.position.clone();
		body.position.add(body.velocity.times(dt));
		body.transform.radians += body.angularVelocity * dt;
	}
	for (let i = 0; i < 3; i++) {
		for (let manifold of this.manifolds) {
			manifold.positionalCorrection();
		}
		for (let joint of this.joints) {
			joint.positionalCorrection();
		}
	}
}

function clearForces() {
	for (let body of this.bodies) {
		body.force.set({x: 0, y: 0});
		body.torque = 0;
	}
}
