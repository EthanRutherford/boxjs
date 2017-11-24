const {Node, AABBTree, BroadPhase} = require("../collision/broadphase");
const {ManifoldPoint, Manifold, ManifoldMap} = require("../collision/manifold");
const Solver = require("./solver");
const Body = require("../objects/body");
const MassData = require("../objects/mass");

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
	const clone = Object.create(Body.prototype);
	clone.id = body.id;
	clone.position = body.position.clone();
	clone.prevPos = body.prevPos.clone();
	clone.transform = body.transform.clone();
	clone.prevAngle = body.prevAngle;
	clone.velocity = body.velocity.clone();
	clone.angularVelocity = body.angularVelocity;
	clone.force = body.force.clone();
	clone.torque = body.torque;
	clone.shapes = [];
	body.shapes.forEach((shape) => {
		const clonedShape = shape.clone();
		clonedShape.body = clone;
		clonedShape.setAABB();
		map[shape.id] = clonedShape;
		clone.shapes.push(clonedShape);
	});
	clone.friction = body.friction;
	clone.restitution = body.restitution;
	clone.mass = cloneMass(body.mass);
	clone.sensor = body.sensor;
	clone.filterGroup = body.filterGroup;
	clone.exclusionMask = body.exclusionMask;
	clone.onCollide = body.onCollide;
	return clone;
}

function cloneJoint(joint, map) {
	return joint.clone(map[joint.bodyA.id], map[joint.bodyB.id]);
}

function cloneManifoldPoint(point) {
	const clone = new ManifoldPoint(
		point.point.clone(),
		point.lpoint.clone(),
		point.indexA,
		point.indexB,
		point.typeA,
		point.typeB,
	);
	clone.bias = point.bias;
	clone.normalImpulse = point.normalImpulse;
	clone.tangentImpulse = point.tangentImpulse;
	clone.normalMass = point.normalMass;
	clone.tangentMass = point.tangentMass;
	return clone;
}

function cloneManifold(manifold, shapeA, shapeB, key) {
	const clone = new Manifold(shapeA, shapeB, key);
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
	for (const [key, manifold] of manifolds.map) {
		const a = map[manifold.shapeA.id];
		const b = map[manifold.shapeB.id];
		clone.map.set(key, cloneManifold(manifold, a, b, key));
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
	if (node.shape != null) {
		clone.shape = map[node.shape.id];
		s2n[clone.shape.id] = clone;
	} else {
		clone.shape = null;
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

function cloneBroadPhase(broadPhase, manifolds, map) {
	const clone = new BroadPhase(manifolds);
	clone.tree = cloneAABBTree(broadPhase.tree, map, clone.shapeToNode);
	return clone;
}

module.exports = function fork(solver) {
	const clone = Object.create(Solver.prototype);
	clone.applyG = solver.applyG;
	clone.bodies = new Set();
	clone.bodyMap = {};
	clone.joints = new Set();
	clone.jointMap = {};
	clone.shapeMap = {};
	solver.bodies.forEach((body) => {
		const clonedBody = cloneBody(body, clone.shapeMap);
		clone.bodyMap[body.id] = clonedBody;
		clone.bodies.add(clonedBody);
	});
	solver.joints.forEach((joint) => {
		const clonedJoint = cloneJoint(joint, clone.bodyMap);
		clone.jointMap[joint.id] = clonedJoint;
		clone.joints.add(clonedJoint);
	});
	clone.manifolds = cloneManifoldMap(solver.manifolds, clone.shapeMap);
	clone.broadPhase = cloneBroadPhase(solver.broadPhase, clone.manifolds, clone.shapeMap);
	return clone;
};
