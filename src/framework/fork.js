const {Node, AABBTree, PairSet, BroadPhase} = require("../collision/broadphase");
const {ManifoldPoint, Manifold, ManifoldMap} = require("../collision/manifold");
const Solver = require("./solver");
const Body = require("../objects/body");
const MassData = require("../objects/mass");
const {Shape} = require("../objects/shape");

function cloneMass(mass) {
	const clone = new MassData();
	clone.m = mass.m;
	clone.iM = mass.iM;
	clone.i = mass.i;
	clone.iI = mass.iI;
	clone.center = mass.center.clone();
	return clone;
}

function cloneBody(body, map) {
	const clone = new Body({
		position: body.position,
		angle: body.transform.radians,
		velocity: body.velocity,
		angularVelocity: body.angularVelocity,
		shapes: [],
		friction: body.friction,
		restitution: body.restitution,
		density: 0,
		sensor: body.sensor,
		onCollide: body.onCollide,
	});
	body.shapes.forEach((shape) => {
		const clonedShape = shape.clone();
		clonedShape.body = clone;
		map.set(shape, clonedShape);
		clone.shapes.push(clonedShape);
	});
	clone.prevPos = body.prevPos.clone();
	clone.mass = cloneMass(body.mass);
	clone.force = body.force.clone();
	clone.torque = body.torque;
	clone.filterGroup = body.filterGroup;
	clone.exclusionMask = body.exclusionMask;
	return clone;
}

function cloneJoint(joint, map) {
	return joint.clone(map.get(joint.bodyA), map.get(joint.bodyB));
}

function cloneManifoldPoint(point) {
	const clone = new ManifoldPoint(
		point.point.clone(),
		point.lpoint.clone(),
		point.indexA,
		point.indexB,
		point.typeA,
		point.typeB
	);
	clone.bias = point.bias;
	clone.normalImpulse = point.normalImpulse;
	clone.tangentImpulse = point.tangentImpulse;
	clone.normalMass = point.normalMass;
	clone.tangentMass = point.tangentMass;
	return clone;
}

function cloneManifold(manifold, shapeA, shapeB) {
	const clone = new Manifold(shapeA, shapeB);
	clone.type = manifold.type;
	manifold.contacts.forEach((point) => clone.contacts.push(cloneManifoldPoint(point)));
	clone.normal = manifold.normal.clone();
	clone.tangent = manifold.tangent.clone();
	clone.lpoint = manifold.lpoint.clone();
	clone.lnormal = manifold.lnormal.clone();
	clone.ltangent = manifold.ltangent.clone();
	clone.k = manifold.k.clone();
	clone.nMass = manifold.nMass ? manifold.nMass.clone() : null;
	clone.hasSensor = manifold.hasSensor;
	return clone;
}

function cloneManifoldMap(manifolds, map) {
	const clone = new ManifoldMap();
	for (const manifold of manifolds) {
		const [a, b] = Shape.order(map.get(manifold.shapeA), map.get(manifold.shapeB));
		const key = `${a.id}:${b.id}`;
		clone.map.set(key, cloneManifold(manifold, a, b));
	}
	return clone;
}

function cloneNode(parent, node, map, s2n) {
	const clone = Object.create(Node.prototype);
	clone.aabb = node.aabb.clone();
	clone.parent = parent;
	clone.children = [];
	node.children.forEach((child) => {
		clone.children.push(cloneNode(clone, child, map, s2n));
	});
	clone.height = node.height;
	clone.shape = map.get(node.shape);
	if (clone.shape != null) {
		s2n.set(clone.shape, clone);
	}

	return clone;
}

function cloneAABBTree(tree, map, s2n) {
	const clone = new AABBTree();
	clone.count = tree.count;
	if (clone.count === 0) {
		return clone;
	}

	clone.root = cloneNode(null, tree.root, map, s2n);
	return clone;
}

function clonePairSet(pairs, map) {
	const clone = new PairSet();
	for (const pair of pairs) {
		const [a, b] = Shape.order(map.get(pair.a), map.get(pair.b));
		const key = `${a.id}:${b.id}`;
		clone.map.set(key, {a, b});
	}

	return clone;
}

function cloneBroadPhase(broadPhase, map) {
	const clone = new BroadPhase();
	clone.tree = cloneAABBTree(broadPhase.tree, map, clone.shapeToNode);
	clone.pairs = clonePairSet(broadPhase.pairs, map);
	return clone;
}

module.exports = function fork(solver) {
	const clone = Object.create(Solver.prototype);
	clone.applyG = solver.applyG;
	clone.bodies = new Set();
	clone.joints = new Set();
	const shapeMap = new Map();
	const bodyMap = new Map();
	const jointMap = new Map();
	solver.bodies.forEach((body) => {
		const clonedBody = cloneBody(body, shapeMap);
		bodyMap.set(body, clonedBody);
		clone.bodies.add(clonedBody);
	});
	solver.joints.forEach((joint) => {
		const clonedJoint = cloneJoint(joint, bodyMap);
		jointMap.set(joint, clonedJoint);
		clone.joints.add(clonedJoint);
	});
	clone.manifolds = cloneManifoldMap(solver.manifolds, shapeMap);
	clone.broadPhase = cloneBroadPhase(solver.broadPhase, shapeMap);
	return {
		solver: clone,
		bodyMap,
		shapeMap,
		jointMap,
	};
};