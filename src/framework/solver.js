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
			this.applyG(getAwakeBodies(this.bodies));
		}

		solveBroadPhase(this);
		solveNarrowPhase(this);
		solveIslands(this, dt);
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
		joint.bodyA.setAsleep(false);
		joint.bodyB.setAsleep(false);
	}
	removeJoint(joint) {
		this.joints.delete(joint);
		delete this.jointMap[joint.id];
		joint.bodyA.setAsleep(false);
		joint.bodyB.setAsleep(false);
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
				return -1;	// skip this shape
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

// private functions
function getAwakeBodies(bodies) {
	const results = [];
	for (const body of bodies) {
		if (!body.isAsleep) {
			results.push(body);
		}
	}

	return results;
}

function solveBroadPhase(solver) {
	// update aabbs
	for (const body of solver.bodies) {
		for (const shape of body.shapes) {
			shape.setAABB();
		}
	}

	// the broadphase will update unique pairs of overlapping shapes.
	// any manifolds which cease to overlap will be removed.
	solver.broadPhase.updatePairs();
}

function solveNarrowPhase(solver) {
	// here we iterate through the manifolds, solving the narrowphase
	for (const manifold of solver.manifolds) {
		manifold.solve();
	}
}

function solveIslands(solver, dt) {
	const unprocessed = new Set(solver.bodies);
	const manifoldMap = {};
	for (const manifold of solver.manifolds) {
		if (manifoldMap[manifold.shapeA.body.id] == null) {
			manifoldMap[manifold.shapeA.body.id] = [];
		}
		if (manifoldMap[manifold.shapeB.body.id] == null) {
			manifoldMap[manifold.shapeB.body.id] = [];
		}

		manifoldMap[manifold.shapeA.body.id].push(manifold);
		manifoldMap[manifold.shapeB.body.id].push(manifold);
	}

	const jointMap = {};
	for (const joint of solver.joints) {
		if (jointMap[joint.bodyA.id] == null) {
			jointMap[joint.bodyA.id] = [];
		}
		if (jointMap[joint.bodyB.id] == null) {
			jointMap[joint.bodyB.id] = [];
		}

		jointMap[joint.bodyA.id].push(joint);
		jointMap[joint.bodyB.id].push(joint);
	}

	for (const body of unprocessed) {
		if (body.isAsleep || body.isStatic) {
			continue;
		}

		const island = {
			bodies: new Set(),
			manifolds: new Set(),
			joints: new Set(),
		};

		makeIsland(island, body, unprocessed, manifoldMap, jointMap);

		applyForces(island, dt);
		solveVelocities(island, dt);
		solvePositions(island, dt);
		clearForces(island, dt);
		collisionCallbacks(island, dt);
		setSleep(island, dt);
	}
}

function makeIsland(island, body, unprocessed, manifoldMap, jointMap) {
	// we don't propogate islands across static bodies
	// in addition, since they're static, they don't need to iterate
	if (body.isStatic) {
		return;
	}

	// any body which is connected with an awake body should be awoken
	island.bodies.add(body);
	unprocessed.delete(body);
	body.setAsleep(false);

	for (const manifold of manifoldMap[body.id] || []) {
		if (island.manifolds.has(manifold) || !manifold.isCollided) {
			continue;
		}

		island.manifolds.add(manifold);

		const other = (manifold.shapeA.body === body ? manifold.shapeB : manifold.shapeA).body;
		if (island.bodies.has(other)) {
			continue;
		}

		makeIsland(island, other, unprocessed, manifoldMap, jointMap);
	}

	for (const joint of jointMap[body.id] || []) {
		if (island.joints.has(joint)) {
			continue;
		}

		island.joints.add(joint);

		const other = joint.bodyA === body ? joint.bodyB : joint.bodyA;
		if (island.bodies.has(other)) {
			continue;
		}

		makeIsland(island, other, unprocessed, manifoldMap, jointMap);
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
		body.prevTrans = body.transform.clone();
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

function setSleep(solver, dt) {
	const angularTolerance = 2 / 180 * Math.PI;
	const linearTolerance = .0001;
	const sleepThreshold = .5;

	let minSleepTime = Infinity;

	for (const body of solver.bodies) {
		if (body.isStatic) {
			continue;
		}

		if (
			Math.abs(body.angularVelocity) > angularTolerance ||
			body.velocity.lsqr > linearTolerance
		) {
			body.sleepTime = 0;
			minSleepTime = 0;
		} else {
			body.sleepTime += dt;
			minSleepTime = Math.min(minSleepTime, body.sleepTime);
		}
	}

	if (minSleepTime >= sleepThreshold) {
		for (const body of solver.bodies) {
			body.setAsleep(true);
		}
	}
}
