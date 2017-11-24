const {Vector2D, Matrix2D, clamp} = require("../framework/math");

class ManifoldPoint {
	constructor(point, lpoint, indexA, indexB, typeA, typeB) {
		this.point = point || new Vector2D(0, 0);
		this.lpoint = lpoint || new Vector2D(0, 0);
		this.bias = 0;
		this.normalImpulse = 0;
		this.tangentImpulse = 0;
		this.normalMass = 0;
		this.tangentMass = 0;

		//identification components
		this.indexA = indexA;
		this.indexB = indexB;
		this.typeA = typeA;
		this.typeB = typeB;
	}
	equals(other) {
		return this.indexA != null &&
			this.indexA === other.indexA &&
			this.indexB === other.indexB &&
			this.typeA === other.typeA &&
			this.typeB === other.typeB;
	}
	flip() {
		[this.indexA, this.indexB] = [this.indexB, this.indexA];
		[this.typeA, this.typeB] = [this.typeB, this.typeA];
	}
	absorb(other) {
		this.normalImpulse = other.normalImpulse;
		this.tangentImpulse = other.tangentImpulse;
	}
}

ManifoldPoint.face = 0;
ManifoldPoint.vert = 1;

class Manifold {
	constructor(a, b, key) {
		this.shapeA = a;
		this.shapeB = b;
		this.key = key;
		this.type = -1;
		this.e = Math.max(a.body.restitution, b.body.restitution);
		this.df = Math.sqrt(a.body.friction, b.body.friction);
		this.contacts = [];
		this.normal = new Vector2D(0, 0);
		this.tangent = new Vector2D(0, 0);
		this.lpoint = new Vector2D(0, 0);
		this.lnormal = new Vector2D(0, 0);
		this.ltangent = new Vector2D(0, 0);
		this.k = new Matrix2D(0, 0, 0, 0);
		this.nMass = null;
		this.hasSensor = a.body.sensor || b.body.sensor;
	}
	solve() {
		const a = this.shapeA;
		const b = this.shapeB;
		if (!a.aabb.test(b.aabb)) {
			this.contacts = [];
			return;
		}
		Collision.getCollider(a.constructor, b.constructor)(this, a, b);
	}
	get isCollided() {
		return !this.hasSensor && this.contacts.length > 0;
	}
	initialize() {
		if (!this.isCollided) {
			return;
		}

		const mA = this.shapeA.body.mass.iM;
		const iA = this.shapeA.body.mass.iI;
		const mB = this.shapeB.body.mass.iM;
		const iB = this.shapeB.body.mass.iI;

		const vA = this.shapeA.body.velocity;
		const vB = this.shapeB.body.velocity;

		const wA = this.shapeA.body.angularVelocity;
		const wB = this.shapeB.body.angularVelocity;

		for (const contact of this.contacts) {
			const rA = contact.point.minus(this.shapeA.body.position);
			const rB = contact.point.minus(this.shapeB.body.position);

			const rnA = rA.cross(this.normal);
			const rnB = rB.cross(this.normal);
			const kn = mA + mB + rnA * rnA * iA + rnB * rnB * iB;
			contact.normalMass = kn > 0 ? 1 / kn : 0;

			const rtA = rA.cross(this.tangent);
			const rtB = rB.cross(this.tangent);
			const kt = mA + mB + rtA * rtA * iA + rtB * rtB * iB;
			contact.tangentMass = kt > 0 ? 1 / kt : 0;

			contact.bias = 0;
			const rv = vB.plus(Vector2D.cross1x2(wB, rB)).sub(vA.plus(Vector2D.cross1x2(wA, rA)));
			const vRel = this.normal.dot(rv);
			if (vRel < -1) {
				contact.bias = -this.e * vRel;
			}
		}

		if (this.contacts.length === 2) {
			const rA0 = this.contacts[0].point.minus(this.shapeA.body.position);
			const rB0 = this.contacts[0].point.minus(this.shapeB.body.position);
			const rA1 = this.contacts[1].point.minus(this.shapeA.body.position);
			const rB1 = this.contacts[1].point.minus(this.shapeB.body.position);

			const rn0A = rA0.cross(this.normal);
			const rn0B = rB0.cross(this.normal);
			const rn1A = rA1.cross(this.normal);
			const rn1B = rB1.cross(this.normal);

			const k11 = mA + mB + iA * rn0A * rn0A + iB * rn0B * rn0B;
			const k22 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			const k12 = mA + mB + iA * rn0A * rn1A + iB * rn0B * rn1B;

			const maxCondition = 1000;
			if (k11 * k11 < maxCondition * (k11 * k22 - k12 * k12)) {
				this.k.ii = k11;
				this.k.ij = k12;
				this.k.ji = k12;
				this.k.jj = k22;
				this.nMass = this.k.inverse();
			} else {
				this.contacts.pop();
			}
		}
	}
	warmStart() {
		if (!this.isCollided) {
			return;
		}

		const mA = this.shapeA.body.mass.iM;
		const iA = this.shapeA.body.mass.iI;
		const mB = this.shapeB.body.mass.iM;
		const iB = this.shapeB.body.mass.iI;

		const vA = this.shapeA.body.velocity;
		const vB = this.shapeB.body.velocity;

		let wA = this.shapeA.body.angularVelocity;
		let wB = this.shapeB.body.angularVelocity;

		for (const contact of this.contacts) {
			const rA = contact.point.minus(this.shapeA.body.position);
			const rB = contact.point.minus(this.shapeB.body.position);

			const p = this.normal.times(contact.normalImpulse).add(this.tangent.times(contact.tangentImpulse));
			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * rA.cross(p);
			wB += iB * rB.cross(p);
		}

		this.shapeA.body.velocity.set(vA);
		this.shapeB.body.velocity.set(vB);
		this.shapeA.body.angularVelocity = wA;
		this.shapeB.body.angularVelocity = wB;
	}
	applyImpulse() {
		if (!this.isCollided) {
			return;
		}

		const mA = this.shapeA.body.mass.iM;
		const iA = this.shapeA.body.mass.iI;
		const mB = this.shapeB.body.mass.iM;
		const iB = this.shapeB.body.mass.iI;

		const vA = this.shapeA.body.velocity;
		const vB = this.shapeB.body.velocity;

		let wA = this.shapeA.body.angularVelocity;
		let wB = this.shapeB.body.angularVelocity;

		for (const contact of this.contacts) {
			const rA = contact.point.minus(this.shapeA.body.position);
			const rB = contact.point.minus(this.shapeB.body.position);

			const dv = vB.plus(Vector2D.cross1x2(wB, rB)).sub(vA.plus(Vector2D.cross1x2(wA, rA)));
			const vt = dv.dot(this.tangent);
			let lambda = contact.tangentMass * -vt;
			const mf = contact.normalImpulse * this.df;
			const newImpulse = clamp(contact.tangentImpulse + lambda, -mf, mf);
			lambda = newImpulse - contact.tangentImpulse;
			contact.tangentImpulse = newImpulse;

			const p = this.tangent.times(lambda);
			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * rA.cross(p);
			wB += iB * rB.cross(p);
		}

		if (this.contacts.length === 1) {
			const contact = this.contacts[0];
			const rA = contact.point.minus(this.shapeA.body.position);
			const rB = contact.point.minus(this.shapeB.body.position);

			const dv = vB.plus(Vector2D.cross1x2(wB, rB)).sub(vA.plus(Vector2D.cross1x2(wA, rA)));
			const vn = dv.dot(this.normal);
			let lambda = -contact.normalMass * (vn - contact.bias);
			const newImpulse = Math.max(contact.normalImpulse + lambda, 0);
			lambda = newImpulse - contact.normalImpulse;
			contact.normalImpulse = newImpulse;

			const p = this.normal.times(lambda);
			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * rA.cross(p);
			wB += iB * rB.cross(p);
		} else {
			const contact0 = this.contacts[0];
			const contact1 = this.contacts[1];
			const rA0 = contact0.point.minus(this.shapeA.body.position);
			const rB0 = contact0.point.minus(this.shapeB.body.position);
			const rA1 = contact1.point.minus(this.shapeA.body.position);
			const rB1 = contact1.point.minus(this.shapeB.body.position);

			const a = new Vector2D(contact0.normalImpulse, contact1.normalImpulse);
			const dv0 = vB.plus(Vector2D.cross1x2(wB, rB0)).sub(vA.plus(Vector2D.cross1x2(wA, rA0)));
			const dv1 = vB.plus(Vector2D.cross1x2(wB, rB1)).sub(vA.plus(Vector2D.cross1x2(wA, rA1)));
			let vn0 = dv0.dot(this.normal);
			let vn1 = dv1.dot(this.normal);
			const b = new Vector2D(vn0 - contact0.bias, vn1 - contact1.bias).sub(this.k.times(a));

			//block solver, taken from Box2D
			while (true) {
				const x = this.nMass.times(b).negate();
				if (x.x >= 0 && x.y >= 0) {
					const d = x.minus(a);
					const p0 = this.normal.times(d.x);
					const p1 = this.normal.times(d.y);
					vA.sub(p0.plus(p1).mul(mA));
					vB.add(p0.plus(p1).mul(mB));
					wA -= iA * (rA0.cross(p0) + rA1.cross(p1));
					wB += iB * (rB0.cross(p0) + rB1.cross(p1));
					contact0.normalImpulse = x.x;
					contact1.normalImpulse = x.y;
					break;
				}

				x.x = -contact0.normalMass * b.x;
				x.y = 0;
				vn0 = 0;
				vn1 = this.k.ji * x.x + b.yy;
				if (x.x >= 0 && x.y >= 0) {
					const d = x.minus(a);
					const p0 = this.normal.times(d.x);
					const p1 = this.normal.times(d.y);
					vA.sub(p0.plus(p1).mul(mA));
					vB.add(p0.plus(p1).mul(mB));
					wA -= iA * (rA0.cross(p0) + rA1.cross(p1));
					wB += iB * (rB0.cross(p0) + rB1.cross(p1));
					contact0.normalImpulse = x.x;
					contact1.normalImpulse = x.y;
					break;
				}

				x.x = 0;
				x.y = -contact1.normalMass * b.y;
				vn0 = this.k.ij * x.y + b.x;
				vn1 = 0;
				if (x.y >= 0 && vn0 >= 0) {
					const d = x.minus(a);
					const p0 = this.normal.times(d.x);
					const p1 = this.normal.times(d.y);
					vA.sub(p0.plus(p1).mul(mA));
					vB.add(p0.plus(p1).mul(mB));
					wA -= iA * (rA0.cross(p0) + rA1.cross(p1));
					wB += iB * (rB0.cross(p0) + rB1.cross(p1));
					contact0.normalImpulse = x.x;
					contact1.normalImpulse = x.y;
					break;
				}

				x.x = 0;
				x.y = 0;
				vn0 = b.x;
				vn1 = b.y;
				if (vn0 >= 0 && vn1 >= 0) {
					const d = x.minus(a);
					const p0 = this.normal.times(d.x);
					const p1 = this.normal.times(d.y);
					vA.sub(p0.plus(p1).mul(mA));
					vB.add(p0.plus(p1).mul(mB));
					wA -= iA * (rA0.cross(p0) + rA1.cross(p1));
					wB += iB * (rB0.cross(p0) + rB1.cross(p1));
					contact0.normalImpulse = x.x;
					contact1.normalImpulse = x.y;
					break;
				}

				break;
			}
		}

		this.shapeA.body.velocity.set(vA);
		this.shapeB.body.velocity.set(vB);
		this.shapeA.body.angularVelocity = wA;
		this.shapeB.body.angularVelocity = wB;
	}
	positionalCorrection() {
		if (!this.isCollided) {
			return;
		}

		const percent = .2;
		const kSlop = .005;

		const mA = this.shapeA.body.mass.iM;
		const iA = this.shapeA.body.mass.iI;
		const mB = this.shapeB.body.mass.iM;
		const iB = this.shapeB.body.mass.iI;

		const cA = this.shapeA.body.position.clone();
		const cB = this.shapeB.body.position.clone();
		let aA = this.shapeA.body.transform.radians;
		let aB = this.shapeB.body.transform.radians;

		let r = 0;
		r += this.shapeA.radius ? this.shapeA.radius : 0;
		r += this.shapeB.radius ? this.shapeB.radius : 0;

		for (const contact of this.contacts) {
			let point;
			let separation;
			let normal;
			if (this.type === Manifold.circles) {
				const pA = this.lpoint.plus(this.shapeA.body.position);
				const pB = this.contacts[0].lpoint.plus(this.shapeB.body.position);
				normal = (pB.minus(pA)).normalize();
				point = pA.plus(pB).mul(.5);
				separation = pB.minus(pA).dot(normal) - r;
			} else if (this.type === Manifold.faceA) {
				normal = this.shapeA.body.transform.times(this.lnormal);
				const plane = this.shapeA.body.transform.times(this.lpoint).add(this.shapeA.body.position);
				const clip = this.shapeB.body.transform.times(contact.lpoint).add(this.shapeB.body.position);
				separation = (clip.minus(plane)).dot(normal) - r;
				point = clip;
			} else if (this.type === Manifold.faceB) {
				normal = this.shapeB.body.transform.times(this.lnormal);
				const plane = this.shapeB.body.transform.times(this.lpoint).add(this.shapeB.body.position);
				const clip = this.shapeA.body.transform.times(contact.lpoint).add(this.shapeA.body.position);
				separation = (clip.minus(plane)).dot(normal) - r;
				point = clip;
				normal.negate();
			}

			const rA = point.minus(this.shapeA.body.position);
			const rB = point.minus(this.shapeB.body.position);

			const c = clamp(percent * (separation + kSlop), -percent, 0);
			const rnA = rA.cross(normal);
			const rnB = rB.cross(normal);
			const k = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			const impulse = k > 0 ? -c / k : 0;

			const p = normal.times(impulse);
			cA.sub(p.times(mA));
			cB.add(p.times(mB));
			aA -= iA * rA.cross(p);
			aB += iB * rB.cross(p);
		}

		this.shapeA.body.position.set(cA);
		this.shapeB.body.position.set(cB);
		this.shapeA.body.transform.radians = aA;
		this.shapeB.body.transform.radians = aB;
	}
}

Manifold.circles = 0;
Manifold.faceA = 1;
Manifold.faceB = 2;

class ManifoldMap {
	constructor() {
		this.map = new Map();
	}
	add({a, b}) {
		const key = `${a.id}:${b.id}`;
		if (!this.map.has(key)) {
			this.map.set(key, new Manifold(a, b, key));
		}
	}
	delete(key) {
		this.map.delete(key);
	}
	clear() {
		this.map.clear();
	}
	keys() {
		return this.map.keys();
	}
	[Symbol.iterator]() {
		return this.map.values();
	}
}

module.exports = {
	ManifoldPoint,
	Manifold,
	ManifoldMap,
};

const Collision = require("./collision.js");
