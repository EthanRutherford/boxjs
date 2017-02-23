const {Vector2D} = require("../framework/math.js");
const {Manifold, ManifoldPoint} = require("./manifold.js");
const Polygon = require("../objects/polygon.js");
const Circle = require("../objects/circle.js");

let baseMap = new Map();

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
	let bestDistance = -Number.MAX_VALUE;
	let bestIndex = 0;
	for (let i = 0; i < a.points.length; i++) {
		let nw = a.body.transform.times(a.norms[i]);
		let btT = b.body.transform.transpose;
		let n = btT.times(nw);

		let s = b.getSupport(n.neg());
		let v = a.body.transform.times(a.points[i]).plus(a.body.position);
		v = btT.times(v.minus(b.body.position));

		let d = n.dot(s.minus(v));
		if (d > bestDistance) {
			bestDistance = d;
			bestIndex = i;
		}
	}
	return {index: bestIndex, distance: bestDistance};
}

function findIncidentEdge(ref, inc, index) {
	let refNormal = ref.norms[index];
	refNormal = ref.body.transform.times(refNormal);
	refNormal = inc.body.transform.transpose.times(refNormal);
	let edge1 = 0;
	let minDot = Number.MAX_VALUE;
	for (let i = 0; i < inc.points.length; i++) {
		let d = refNormal.dot(inc.norms[i]);
		if (d < minDot) {
			minDot = d;
			edge1 = i;
		}
	}
	let edge2 = edge1 + 1 >= inc.points.length ? 0 : edge1 + 1;

	let r1 = new ManifoldPoint(
		inc.body.transform.times(inc.points[edge1]).plus(inc.body.position),
		inc.points[edge1],
		index,
		edge1,
		ManifoldPoint.face,
		ManifoldPoint.vert
	);
	let r2 = new ManifoldPoint(
		inc.body.transform.times(inc.points[edge2]).plus(inc.body.position),
		inc.points[edge2],
		index,
		edge2,
		ManifoldPoint.face,
		ManifoldPoint.vert
	);

	return [r1, r2];
}

function clipPoints(input, n, c, index) {
	let output = [];
	let dist0 = n.dot(input[0].point) - c;
	let dist1 = n.dot(input[1].point) - c;
	if (dist0 <= 0) {
		output.push(input[0]);
	}
	if (dist1 <= 0) {
		output.push(input[1]);
	}
	if (dist0 * dist1 < 0) {
		let interp = dist0 / (dist0 - dist1);
		output.push(new ManifoldPoint(
			input[0].point.plus(input[1].point.minus(input[0].point).times(interp)),
			input[0].lpoint.plus(input[1].lpoint.minus(input[0].lpoint).times(interp)),
			index,
			input[0].indexB,
			ManifoldPoint.vert,
			ManifoldPoint.face
		));
	}
	return output;
}

function polyToPoly(m, a, b) {
	let oldContacts = m.contacts;
	m.contacts = [];
	let {index: edgeA, distance: separationA} = findSeparatingAxis(a, b);
	if (separationA > 0) {
		return;
	}

	let {index: edgeB, distance: separationB} = findSeparatingAxis(b, a);
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

	let incidentEdge = findIncidentEdge(refPoly, incPoly, refIndex);
	let iv1 = refIndex;
	let iv2 = iv1 + 1 >= refPoly.points.length ? 0 : iv1 + 1;
	let v1 = refPoly.points[iv1];
	let v2 = refPoly.points[iv2];
	m.ltangent = v2.minus(v1).normalize();
	m.lnormal = Vector2D.cross2x1(m.ltangent, 1);
	m.lpoint = v1.plus(v2).times(.5);

	let tangent = refPoly.body.transform.times(m.ltangent);
	let normal = Vector2D.cross2x1(tangent, 1);
	v1 = refPoly.body.transform.times(v1).plus(refPoly.body.position);
	v2 = refPoly.body.transform.times(v2).plus(refPoly.body.position);

	let refC = normal.dot(v1);
	let negSide = -tangent.dot(v1);
	let posSide = tangent.dot(v2);

	let clip = clipPoints(incidentEdge, tangent.neg(), negSide, iv1);
	if (clip.length < 2) {
		return;
	}

	clip = clipPoints(clip, tangent, posSide, iv2);
	if (clip.count < 2) {
		return;
	}

	for (let point of clip) {
		let separation = normal.dot(point.point) - refC;
		if (separation <= 0) {
			if (flip) {
				point.flip();
			}

			for (let old of oldContacts) {
				if (point.equals(old)) {
					point.absorb(old);
				}
			}
			m.contacts.push(point);
		}
	}
	m.normal = flip ? normal.neg() : normal;
	m.tangent = flip ? tangent.neg() : tangent;
}

function circleToCircle(m, a, b) {
	m.contacts = [];
	m.normal = a.body.position.minus(b.body.position);
	let dist = m.normal.lsqr;
	let radius = a.radius + b.radius;
	if (dist > Math.sqr(radius)) {
		return;
	}

	m.contacts.push(new ManifoldPoint(
		new Vector2D(b.body.position + a.body.position).mul(.5)
	));
	m.type = Manifold.circles;
	m.normal.normalize();
	m.tangent = Vector2D.cross2x1(m.normal, 1);
}

function circleToPoly(m, a, b) {
	m.contacts = [];
	let center = b.body.transform.transpose.times(a.body.position.minus(b.body.position));
	let separation = -Number.MAX_VALUE;
	let i1 = 0;
	for (let i = 0; i < b.points.length; i++) {
		let s = b.norms[i].dot(center.minus(b.points[i]));
		if (s > a.radius) {
			return;
		}

		if (s > separation) {
			separation = s;
			i1 = i;
		}
	}
	let i2 = i1 + 1 < b.points.length ? i1 + 1 : 0;
	let v1 = b.points[i1];
	let v2 = b.points[i2];
	if (separation < Number.EPSILON) {
		m.type = Manifold.faceB;
		m.lnormal = b.norms[i1];
		m.lpoint = v1.plus(v2).mul(.5);
		m.normal = b.body.transform.times(b.norms[i1]).neg();
		m.contacts.push(new ManifoldPoint(
			m.normal.times(a.radius).plus(a.body.position)
		));
	} else {
		let dot1 = center.minus(v1).dot(v2.minus(v1));
		let dot2 = center.minus(v2).dot(v1.minus(v2));
		if (dot1 <= 0) {
			if (center.minus(v1).lsqr > Math.sqr(a.radius)) {
				return;
			}

			m.type = Manifold.faceB;
			m.lnormal = center.minus(v1).normalize();
			m.normal = b.body.transform.times(m.lnormal).neg().normalize();
			m.lpoint = v1;
			m.contacts.push(new ManifoldPoint(
				b.body.transform.times(v1).plus(b.body.position)
			));
		} else if (dot2 <= 0) {
			if (center.minus(v2).lsqr > Math.sqr(a.radius)) {
				return;
			}

			m.type = Manifold.faceB;
			m.lnormal = center.minus(v2).normalize();
			m.normal = b.body.transform.times(m.lnormal).neg().normalize();
			m.lpoint = v2;
			m.contacts.push(new ManifoldPoint(
				b.body.transform.times(v2).plus(b.body.position)
			));
		} else {
			m.lpoint = v1.plus(v2).mul(.5);
			m.lnormal = b.norms[i1];
			if (center.minus(m.lpoint).dot(m.lnormal) > a.radius) {
				return;
			}

			m.normal = b.body.transform.times(m.lnormal).neg().normalize();
			m.type = Manifold.faceB;
			m.contacts.push(new ManifoldPoint(
				m.normal.times(a.radius).plus(a.body.position)
			));
		}
	}
	m.tangent = Vector2D.cross2x1(m.normal, 1);
}

function polyToCircle(m, a, b) {
	circleToPoly(m, b, a);
	m.type = Manifold.faceA;
	m.normal = m.normal.neg();
	m.tangent = m.tangent.neg();
}
