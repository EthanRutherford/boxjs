const {BroadPhase} = require("../collision/broadphase");
const ContactData = require("../collision/contactdata");
const {ManifoldMap} = require("../collision/manifold");

module.exports = class Solver {
	constructor() {
		this.applyG = null;
		this.bodies = new Set();
		this.bodyMap = {};
		this.joints = new Set();
		this.jointMap = {};
		this.shapeMap = {};
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
		collisionCallbacks.call(this, dt);
	}
	addBody(body) {
		this.bodies.add(body);
		this.bodyMap[body.id] = body;
		for (const shape of body.shapes) {
			this.broadPhase.insert(shape);
			this.shapeMap[shape.id] = shape;
		}
	}
	removeBody(body) {
		this.bodies.delete(body);
		delete this.bodyMap[body.id];
		for (const shape of body.shapes) {
			this.broadPhase.remove(shape);
			delete this.shapeMap[shape.id];
		}
	}
	addJoint(joint) {
		this.joints.add(joint);
		this.jointMap[joint.id] = joint;
	}
	removeJoint(joint) {
		this.joints.delete(joint);
		delete this.jointMap[joint.id];
	}
	flush() {
		const bodies = [...this.bodies];
		this.bodies.clear();
		this.joints.clear();
		this.manifolds.clear();
		this.broadPhase.flush();
		return bodies;
	}
	query(aabb, callback) {
		this.broadPhase.query(aabb, callback);
	}
	raycast({p1, p2, callback, shouldCheck = null}) {
		const inputRay = {p1, p2, maxFraction: 1};
		this.broadPhase.raycast(inputRay, (ray, shape) => {
			if (shouldCheck instanceof Function && !shouldCheck(shape)) {
				return -1;	//skip this shape
			}

			const castData = shape.raycast(ray);
			if (castData) {
				castData.shape = shape;
				const override = callback(castData);
				return override != null ? override : castData.fraction;
			}

			return -1;
		});
	}
	debugGetNodes() {
		return this.broadPhase.debugGetNodes();
	}
};

//private functions, call with function.prototype.call
function solveBroadPhase() {
	for (const body of this.bodies) {
		for (const shape of body.shapes) {
			shape.setAABB();
		}
	}
	//the broadphase returns a set of unique pairs whose aabbs are overlapping
	const pairs = this.broadPhase.getPairs();
	//the manifold map will persist any manifolds that already exist,
	//and create new manifolds for pairs which don't have one yet
	for (const pair of pairs) {
		this.manifolds.add(pair);
	}
}

function solveNarrowPhase() {
	//here we iterate through the manifolds, solving the narrowphase
	//any that are not collided are removed from the map
	for (const manifold of this.manifolds) {
		manifold.solve();
		if (!manifold.isCollided) {
			this.manifolds.delete(manifold);
		}
	}
}

function applyForces(dt) {
	for (const body of this.bodies) {
		body.velocity.add(body.force.times(body.mass.iM).times(dt));
		body.angularVelocity += body.torque * body.mass.iI * dt;
	}
}

function solveVelocities(dt) {
	for (const manifold of this.manifolds) {
		manifold.initialize();
	}
	for (const manifold of this.manifolds) {
		manifold.warmStart();
	}
	for (const joint of this.joints) {
		joint.initialize(dt);
	}
	for (let i = 0; i < 8; i++) {
		for (const joint of this.joints) {
			joint.applyImpulse(dt);
		}
		for (const manifold of this.manifolds) {
			manifold.applyImpulse();
		}
	}
}

function solvePositions(dt) {
	for (const body of this.bodies) {
		body.prevPos = body.position.clone();
		body.position.add(body.velocity.times(dt));
		body.prevAngle = body.transform.radians;
		body.transform.radians += body.angularVelocity * dt;
	}
	for (let i = 0; i < 4; i++) {
		for (const manifold of this.manifolds) {
			manifold.positionalCorrection();
		}
		for (const joint of this.joints) {
			joint.positionalCorrection();
		}
	}
}

function clearForces() {
	for (const body of this.bodies) {
		body.force.x = 0;
		body.force.y = 0;
		body.torque = 0;
	}
}

function collisionCallbacks(dt) {
	for (const manifold of this.manifolds) {
		const shapeA = manifold.shapeA;
		const shapeB = manifold.shapeB;
		if (shapeA.body.onCollide instanceof Function) {
			shapeA.body.onCollide(new ContactData(manifold, false, dt));
		}
		if (shapeB.body.onCollide instanceof Function) {
			shapeB.body.onCollide(new ContactData(manifold, true, dt));
		}
	}
}
