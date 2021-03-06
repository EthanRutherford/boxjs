const {Vector2D} = require("../framework/math");
const {Manifold, ManifoldPoint} = require("./manifold");
const Polygon = require("../objects/polygon");
const Circle = require("../objects/circle");

const baseMap = new Map();

class Collision {
	static getCollider(type1, type2) {
		if (!baseMap.has(type1)) {
			throw {message: "collider not set for type combination", type1, type2};
		}

		const map = baseMap.get(type1);
		if (!map.has(type2)) {
			throw {message: "collider not set for type combination", type1, type2};
		}

		return baseMap.get(type1).get(type2);
	}
	static newCollider(type1, type2, func) {
		if (!baseMap.has(type1)) {
			baseMap.set(type1, new Map());
		}

		baseMap.get(type1).set(type2, func);
	}
}

module.exports = Collision;

(function registerDefaultColliders() {
	Collision.newCollider(Polygon, Polygon, polyToPoly);
	Collision.newCollider(Circle, Circle, circleToCircle);
	Collision.newCollider(Circle, Polygon, circleToPoly);
	Collision.newCollider(Polygon, Circle, polyToCircle);
})();

function findSeparatingAxis(a, b) {
	const btT = b.body.transform.transpose();
	let bestDistance = -Number.MAX_VALUE;
	let bestIndex = 0;
	for (let i = 0; i < a.points.length; i++) {
		const n = btT.mul(a.body.transform.times(a.norms[i]));

		const s = b.getSupport(n.neg());
		let v = a.body.transform.times(a.points[i]).add(a.body.position);
		v = btT.mul(v.sub(b.body.position));

		const d = n.dot(s.minus(v));
		if (d > bestDistance) {
			bestDistance = d;
			bestIndex = i;
		}
	}

	return {index: bestIndex, distance: bestDistance};
}

function findIncidentEdge(ref, inc, index) {
	let refNormal = ref.norms[index].clone();
	refNormal = ref.body.transform.mul(refNormal);
	refNormal = inc.body.transform.transpose().mul(refNormal);
	let edge1 = 0;
	let minDot = Number.MAX_VALUE;
	for (let i = 0; i < inc.points.length; i++) {
		const d = refNormal.dot(inc.norms[i]);
		if (d < minDot) {
			minDot = d;
			edge1 = i;
		}
	}

	const edge2 = edge1 + 1 >= inc.points.length ? 0 : edge1 + 1;

	const r1 = new ManifoldPoint(
		inc.body.transform.times(inc.points[edge1]).add(inc.body.position),
		inc.points[edge1],
		index,
		edge1,
		ManifoldPoint.face,
		ManifoldPoint.vert,
	);
	const r2 = new ManifoldPoint(
		inc.body.transform.times(inc.points[edge2]).add(inc.body.position),
		inc.points[edge2],
		index,
		edge2,
		ManifoldPoint.face,
		ManifoldPoint.vert,
	);

	return [r1, r2];
}

function clipPoints(input, n, c, index) {
	const output = [];
	const dist0 = n.dot(input[0].point) - c;
	const dist1 = n.dot(input[1].point) - c;
	if (dist0 <= 0) {
		output.push(input[0]);
	}
	if (dist1 <= 0) {
		output.push(input[1]);
	}
	if (dist0 * dist1 < 0) {
		const interp = dist0 / (dist0 - dist1);
		output.push(new ManifoldPoint(
			input[1].point.minus(input[0].point).mul(interp).add(input[0].point),
			input[1].lpoint.minus(input[0].lpoint).mul(interp).add(input[0].lpoint),
			index,
			input[0].indexB,
			ManifoldPoint.vert,
			ManifoldPoint.face,
		));
	}
	return output;
}

function polyToPoly(m, a, b) {
	const oldContacts = m.contacts;
	m.contacts = [];
	const {index: edgeA, distance: separationA} = findSeparatingAxis(a, b);
	if (separationA > 0) {
		return;
	}

	const {index: edgeB, distance: separationB} = findSeparatingAxis(b, a);
	if (separationB > 0) {
		return;
	}

	let refIndex;
	let flip;
	let refPoly;
	let incPoly;
	if (separationB > separationA + .0005) {
		refPoly = b;
		incPoly = a;
		refIndex = edgeB;
		m.type = Manifold.faceB;
		flip = true;
	} else {
		refPoly = a;
		incPoly = b;
		refIndex = edgeA;
		m.type = Manifold.faceA;
		flip = false;
	}

	const incidentEdge = findIncidentEdge(refPoly, incPoly, refIndex);
	const iv1 = refIndex;
	const iv2 = iv1 + 1 >= refPoly.points.length ? 0 : iv1 + 1;
	let v1 = refPoly.points[iv1];
	let v2 = refPoly.points[iv2];
	m.ltangent = v2.minus(v1).normalize();
	m.lnormal = Vector2D.cross2x1(m.ltangent, 1);
	m.lpoint = v1.plus(v2).mul(.5);

	const tangent = refPoly.body.transform.times(m.ltangent);
	const normal = Vector2D.cross2x1(tangent, 1);
	v1 = refPoly.body.transform.times(v1).add(refPoly.body.position);
	v2 = refPoly.body.transform.times(v2).add(refPoly.body.position);

	const refC = normal.dot(v1);
	const negSide = -tangent.dot(v1);
	const posSide = tangent.dot(v2);

	let clip = clipPoints(incidentEdge, tangent.neg(), negSide, iv1);
	if (clip.length < 2) {
		return;
	}

	clip = clipPoints(clip, tangent, posSide, iv2);
	if (clip.count < 2) {
		return;
	}

	for (const point of clip) {
		const separation = normal.dot(point.point) - refC;
		if (separation <= 0) {
			if (flip) {
				point.flip();
			}

			for (const old of oldContacts) {
				if (point.equals(old)) {
					point.absorb(old);
				}
			}
			m.contacts.push(point);
		}
	}

	m.normal = flip ? normal.negate() : normal;
	m.tangent = flip ? tangent.negate() : tangent;
}

function circleToCircle(m, a, b) {
	m.contacts = [];
	m.normal = b.body.position.minus(a.body.position);
	const dist = m.normal.lsqr;
	const radius = a.radius + b.radius;
	if (dist > (radius ** 2)) {
		return;
	}

	m.contacts.push(new ManifoldPoint(
		b.body.position.plus(a.body.position).mul(.5),
	));
	m.type = Manifold.circles;
	m.normal.normalize();
	m.tangent = Vector2D.cross2x1(m.normal, 1);
}

function circleToPoly(m, a, b) {
	m.contacts = [];
	const center = b.body.transform.transpose().times(a.body.position.minus(b.body.position));
	let separation = -Number.MAX_VALUE;
	let i1 = 0;
	for (let i = 0; i < b.points.length; i++) {
		const s = b.norms[i].dot(center.minus(b.points[i]));
		if (s > a.radius) {
			return;
		}

		if (s > separation) {
			separation = s;
			i1 = i;
		}
	}
	const i2 = i1 + 1 < b.points.length ? i1 + 1 : 0;
	const v1 = b.points[i1];
	const v2 = b.points[i2];
	if (separation < Number.EPSILON) {
		m.type = Manifold.faceB;
		m.lnormal = b.norms[i1];
		m.lpoint = v1.plus(v2).mul(.5);
		m.normal = b.body.transform.times(b.norms[i1]).negate();
		m.contacts.push(new ManifoldPoint(
			m.normal.times(a.radius).add(a.body.position),
		));
	} else {
		const dot1 = center.minus(v1).dot(v2.minus(v1));
		const dot2 = center.minus(v2).dot(v1.minus(v2));
		if (dot1 <= 0) {
			if (center.minus(v1).lsqr > (a.radius ** 2)) {
				return;
			}

			m.type = Manifold.faceB;
			m.lnormal = center.minus(v1).normalize();
			m.normal = b.body.transform.times(m.lnormal).negate().normalize();
			m.lpoint = v1;
			m.contacts.push(new ManifoldPoint(
				b.body.transform.times(v1).add(b.body.position),
			));
		} else if (dot2 <= 0) {
			if (center.minus(v2).lsqr > (a.radius ** 2)) {
				return;
			}

			m.type = Manifold.faceB;
			m.lnormal = center.minus(v2).normalize();
			m.normal = b.body.transform.times(m.lnormal).negate().normalize();
			m.lpoint = v2;
			m.contacts.push(new ManifoldPoint(
				b.body.transform.times(v2).add(b.body.position),
			));
		} else {
			m.lpoint = v1.plus(v2).mul(.5);
			m.lnormal = b.norms[i1];
			if (center.minus(m.lpoint).dot(m.lnormal) > a.radius) {
				return;
			}

			m.normal = b.body.transform.times(m.lnormal).negate().normalize();
			m.type = Manifold.faceB;
			m.contacts.push(new ManifoldPoint(
				m.normal.times(a.radius).add(a.body.position),
			));
		}
	}
	m.tangent = Vector2D.cross2x1(m.normal, 1);
}

function polyToCircle(m, a, b) {
	circleToPoly(m, b, a);
	m.type = Manifold.faceA;
	m.normal.negate();
	m.tangent.negate();
}
