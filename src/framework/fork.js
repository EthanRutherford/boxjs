const {Node, AABBTree, PairSet, BroadPhase} = require("../collision/broadphase.js");
const {ManifoldPoint, Manifold, ManifoldMap} = require("../collision/manifold.js");
const Solver = require("./solver.js");
const Body = require("../objects/body.js");
const MassData = require("../objects/mass.js");
const {Shape} = require("../objects/shape.js");

function cloneMass(mass) {
	let clone = new MassData();
	clone.m = mass.m;
	clone.iM = mass.iM;
	clone.i = mass.i;
	clone.iI = mass.iI;
	clone.center = mass.center.clone();
	return clone;
}

function cloneBody(body, map) {
	let clone = new Body({
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
		let clonedShape = shape.clone();
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
	let clone = new ManifoldPoint(
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
	let clone = new Manifold(shapeA, shapeB);
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
	let clone = new ManifoldMap();
	for (let manifold of manifolds) {
		let [a, b] = Shape.order(map.get(manifold.shapeA), map.get(manifold.shapeB));
		let key = `${a.id}:${b.id}`;
		clone.map.set(key, cloneManifold(manifold, a, b));
	}
	return clone;
}

function cloneNode(parent, node, map, s2n) {
	let clone = Object.create(Node.prototype);
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
	let clone = new AABBTree();
	clone.count = tree.count;
	if (clone.count === 0) {
		return clone;
	}

	clone.root = cloneNode(null, tree.root, map, s2n);
	return clone;
}

function clonePairSet(pairs, map) {
	let clone = new PairSet();
	for (let pair of pairs) {
		let [a, b] = Shape.order(map.get(pair.a), map.get(pair.b));
		let key = `${a.id}:${b.id}`;
		clone.map.set(key, {a, b});
	}

	return clone;
}

function cloneBroadPhase(broadPhase, map) {
	let clone = new BroadPhase();
	clone.tree = cloneAABBTree(broadPhase.tree, map, clone.shapeToNode);
	clone.pairs = clonePairSet(broadPhase.pairs, map);
	return clone;
}

module.exports = function fork(solver) {
	let clone = Object.create(Solver.prototype);
	clone.applyG = solver.applyG;
	clone.bodies = new Set();
	clone.joints = new Set();
	let shapeMap = new Map();
	let bodyMap = new Map();
	let jointMap = new Map();
	solver.bodies.forEach((body) => {
		let clonedBody = cloneBody(body, shapeMap);
		bodyMap.set(body, clonedBody);
		clone.bodies.add(clonedBody);
	});
	solver.joints.forEach((joint) => {
		let clonedJoint = cloneJoint(joint, bodyMap);
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
