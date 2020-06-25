const {BroadPhase} = require("../collision/broadphase");
const ContactData = require("../collision/contactdata");
const {ManifoldMap} = require("../collision/manifold");
const findTimeOfImpact = require("../collision/toi");

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

		// update contact list
		solveBroadPhase(this);
		solveNarrowPhase(this);

		// store previous positions
		for (const body of this.bodies) {
			body.prevPos = body.position.clone();
			body.prevTrans = body.transform.clone();
		}

		// create constraint maps
		const manifoldMap = {};
		const jointMap = {};
		for (const body of this.bodies) {
			manifoldMap[body.id] = [];
			jointMap[body.id] = [];
		}
		for (const manifold of this.manifolds) {
			manifoldMap[manifold.shapeA.body.id].push(manifold);
			manifoldMap[manifold.shapeB.body.id].push(manifold);
		}
		for (const joint of this.joints) {
			jointMap[joint.bodyA.id].push(joint);
			jointMap[joint.bodyB.id].push(joint);
		}

		// solve
		solveIslands(this, manifoldMap, jointMap, dt);
		solveToiIslands(this, manifoldMap, dt);

		// find and call collision callbacks
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
		// fast forward the broadphase and return nodes
		solveBroadPhase(this);
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
		if (!body.isAsleep) {
			for (const shape of body.shapes) {
				shape.setAABB();
			}
		}
	}

	// the broadphase will update unique pairs of overlapping shapes.
	// any manifolds which cease to overlap will be removed.
	solver.broadPhase.updatePairs();
}

function solveNarrowPhase(solver) {
	// here we iterate through the manifolds, solving the narrowphase
	for (const manifold of solver.manifolds) {
		if (!(manifold.shapeA.body.isAsleep && manifold.shapeB.body.isAsleep)) {
			manifold.solve();
		}
	}
}

function solveIslands(solver, manifoldMap, jointMap, dt) {
	const processed = new Set();

	for (const body of solver.bodies) {
		if (body.isAsleep || body.isStatic || processed.has(body)) {
			continue;
		}

		const island = {
			bodies: new Set(),
			manifolds: new Set(),
			joints: new Set(),
		};

		makeIsland(island, body, processed, manifoldMap, jointMap);

		applyForces(island, dt);
		solveVelocities(island, dt);
		solvePositions(island, dt);
		clearForces(island, dt);
		setSleep(island, dt);
	}
}

function makeIsland(island, body, processed, manifoldMap, jointMap) {
	// we don't propogate islands across static bodies
	// in addition, since they're static, they don't need to iterate
	if (body.isStatic || processed.has(body)) {
		return;
	}

	// any body which is connected with an awake body should be awoken
	island.bodies.add(body);
	processed.add(body);
	body.setAsleep(false);

	for (const manifold of manifoldMap[body.id]) {
		if (island.manifolds.has(manifold) || !manifold.isCollided) {
			continue;
		}

		island.manifolds.add(manifold);

		makeIsland(island, manifold.shapeA.body, processed, manifoldMap, jointMap);
		makeIsland(island, manifold.shapeB.body, processed, manifoldMap, jointMap);
	}

	for (const joint of jointMap[body.id]) {
		if (island.joints.has(joint)) {
			continue;
		}

		island.joints.add(joint);

		makeIsland(island, joint.bodyA, processed, manifoldMap, jointMap);
		makeIsland(island, joint.bodyB, processed, manifoldMap, jointMap);
	}
}

function applyForces(solver, dt) {
	for (const body of solver.bodies) {
		body.velocity.add(body.force.times(body.mass.iM).mul(dt));
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
		body.position.add(body.velocity.times(dt));
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

function setSleep(solver, dt) {
	const angularTolerance = 2 / 180 * Math.PI;
	const linearTolerance = .0001;
	const sleepThreshold = .5;

	let minSleepTime = Infinity;

	for (const body of solver.bodies) {
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

function solveToiIslands(solver, manifoldMap, dt) {
	let t0 = 0;

	// set of contacts that have failed toi
	// used to prevent considering the same toi event repeatedly
	const skipSet = new Set();
	while (true) {
		let earliest = null;
		let minFraction = 1;
		for (const manifold of solver.manifolds) {
			if (
				skipSet.has(manifold) ||
				(manifold.shapeA.body.isAsleep && manifold.shapeB.body.isAsleep) ||
				(!manifold.shapeA.body.toi && !manifold.shapeB.body.toi) ||
				manifold.hasSensor
			) {
				continue;
			}

			const relativeSpeed = manifold.shapeA.body.velocity
				.minus(manifold.shapeB.body.velocity).lsqr;
			if (relativeSpeed < 10) {
				continue;
			}

			const result = findTimeOfImpact(t0, 1, manifold.shapeA, manifold.shapeB);
			if (result != null && result < minFraction) {
				minFraction = result;
				earliest = manifold;
			}
		}

		// no more toi impacts found
		if (earliest == null) {
			return;
		}

		t0 = minFraction;
		const bodyA = earliest.shapeA.body;
		const bodyB = earliest.shapeB.body;

		const pA = bodyA.position;
		const tA = bodyA.transform;
		const pB = bodyB.position;
		const tB = bodyB.transform;

		bodyA.position = bodyA.prevPos.lerp(pA, minFraction);
		bodyA.transform = bodyA.prevTrans.lerp(tA, minFraction);
		bodyB.position = bodyB.prevPos.lerp(pB, minFraction);
		bodyB.transform = bodyB.prevTrans.lerp(tB, minFraction);

		// check for contacts at the toi
		earliest.solve(false);

		if (!earliest.isCollided) {
			// restore positions and skip this manifold
			bodyA.position = pA;
			bodyA.transform = tA;
			bodyB.position = pB;
			bodyB.transform = tB;
			skipSet.add(earliest);
			continue;
		}

		// create minimal island
		const island = {
			bodies: new Set([bodyA, bodyB]),
			manifolds: new Set([earliest]),
		};

		const toiBodies = new Set([bodyA, bodyB]);
		for (const body of toiBodies) {
			if (body.isStatic) continue;

			for (const manifold of manifoldMap[body.id]) {
				if (island.manifolds.has(manifold) || !manifold.isCollided) {
					continue;
				}

				const other = manifold.shapeA.body === body ? manifold.shapeB.body : manifold.shapeA.body;

				const pOther = other.position;
				const tOther = other.transform;

				other.position = other.prevPos.lerp(pOther, minFraction);
				other.transform = other.prevTrans.lerp(tOther, minFraction);

				manifold.solve(false);

				if (!manifold.isCollided) {
					// restore position
					other.position = pOther;
					other.transform = tOther;
					continue;
				}

				island.manifolds.add(manifold);
				island.bodies.add(other);
			}
		}

		// do position correction on bodyA and bodyB
		for (let i = 0; i < 20; i++) {
			for (const manifold of island.manifolds) {
				manifold.positionalCorrection(
					toiBodies.has(manifold.shapeA.body),
					toiBodies.has(manifold.shapeB.body),
				);
			}
		}

		// do impulses
		for (const manifold of island.manifolds) {
			manifold.initialize();
		}
		for (let i = 0; i < 8; i++) {
			for (const manifold of island.manifolds) {
				manifold.applyImpulse();
			}
		}

		// compute new positions
		const remDt = (1 - minFraction) * dt;
		for (const body of island.bodies) {
			body.position.add(body.velocity.times(remDt));
			body.transform.radians += body.angularVelocity * remDt;
		}
	}
}

function collisionCallbacks(solver, dt) {
	for (const manifold of solver.manifolds) {
		if (!manifold.isTouching) {
			continue;
		}

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
