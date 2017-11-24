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
		this.broadPhase = new BroadPhase(this.manifolds);
	}
	solve(dt) {
		if (this.applyG) {
			this.applyG([...this.bodies]);
		}

		solveBroadPhase(this);
		solveNarrowPhase(this);
		applyForces(this, dt);
		solveVelocities(this, dt);
		solvePositions(this, dt);
		clearForces(this);
		collisionCallbacks(this, dt);
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

//private functions
function solveBroadPhase(solver) {
	//update aabbs
	for (const body of solver.bodies) {
		for (const shape of body.shapes) {
			shape.setAABB();
		}
	}
	//the broadphase will update unique pairs of overlapping shapes
	//any manifolds which cease to overlap will be removed
	solver.broadPhase.updatePairs();
}

function solveNarrowPhase(solver) {
	//here we iterate through the manifolds, solving the narrowphase
	for (const manifold of solver.manifolds) {
		manifold.solve();
	}
}

function applyForces(solver, dt) {
	for (const body of solver.bodies) {
		body.velocity.add(body.force.times(body.mass.iM).times(dt));
		body.angularVelocity += body.torque * body.mass.iI * dt;
	}
}

function solveVelocities(solver, dt) {
	for (const manifold of solver.manifolds) {
		manifold.initialize();
	}
	for (const manifold of solver.manifolds) {
		manifold.warmStart();
	}
	for (const joint of solver.joints) {
		joint.initialize(dt);
	}
	for (let i = 0; i < 8; i++) {
		for (const joint of solver.joints) {
			joint.applyImpulse(dt);
		}
		for (const manifold of solver.manifolds) {
			manifold.applyImpulse();
		}
	}
}

function solvePositions(solver, dt) {
	for (const body of solver.bodies) {
		body.prevPos = body.position.clone();
		body.position.add(body.velocity.times(dt));
		body.prevAngle = body.transform.radians;
		body.transform.radians += body.angularVelocity * dt;
	}
	for (let i = 0; i < 3; i++) {
		for (const manifold of solver.manifolds) {
			manifold.positionalCorrection();
		}
		for (const joint of solver.joints) {
			joint.positionalCorrection();
		}
	}
}

function clearForces(solver) {
	for (const body of solver.bodies) {
		body.force.x = 0;
		body.force.y = 0;
		body.torque = 0;
	}
}

function collisionCallbacks(solver, dt) {
	for (const manifold of solver.manifolds) {
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
