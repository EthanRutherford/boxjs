const {Vector2D} = require("../framework/math");
const Joint = require("./joint");

module.exports = class RopeJoint extends Joint {
	constructor({bodyA, bodyB, anchorA, anchorB, limit}) {
		super({bodyA, bodyB, anchorA, anchorB});
		this.limit = limit;

		this.u = new Vector2D();
		this.mass = 0;
		this.impulse = 0;
		this.distance = 0;
	}
	initialize() {
		const rA = this.bodyA.transform.times(this.anchorA);
		const rB = this.bodyB.transform.times(this.anchorB);

		const cA = this.bodyA.position;
		const cB = this.bodyB.position;

		const mA = this.bodyA.mass.iM;
		const mB = this.bodyB.mass.iM;
		const iA = this.bodyA.mass.iI;
		const iB = this.bodyB.mass.iI;

		this.u = cB.plus(rB).sub(cA.plus(rA));
		this.distance = this.u.length;
		if (this.distance < .005) {
			this.u.set(0, 0);
			this.mass = 0;
			this.impulse = 0;
			return;
		}

		this.u.mul(1 / this.distance);

		const crA = rA.cross(this.u);
		const crB = rB.cross(this.u);
		const invMass = mA + iA * Math.sqr(crA) + mB + iB * Math.sqr(crB);
		this.mass = invMass !== 0 ? 1 / invMass : 0;

		const p = this.u.times(this.impulse);
		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * rA.cross(p);
		this.bodyB.angularVelocity += iB * rB.cross(p);
	}
	applyImpulse(dt) {
		const rA = this.bodyA.transform.times(this.anchorA);
		const rB = this.bodyB.transform.times(this.anchorB);

		const vA = this.bodyA.velocity;
		const vB = this.bodyB.velocity;
		let wA = this.bodyA.angularVelocity;
		let wB = this.bodyB.angularVelocity;

		const mA = this.bodyA.mass.iM;
		const mB = this.bodyB.mass.iM;
		const iA = this.bodyA.mass.iI;
		const iB = this.bodyB.mass.iI;

		const vpA = vA.plus(Vector2D.cross1x2(wA, rA));
		const vpB = vB.plus(Vector2D.cross1x2(wB, rB));
		const c = this.distance - this.limit;
		let cDot = this.u.dot(vpB.minus(vpA));
		if (c < 0) {
			cDot += 1 / dt * c;
		}

		let impulse = -this.mass * cDot;
		const oldImpulse = this.impulse;
		this.impulse = Math.min(0, this.impulse + impulse);
		impulse = this.impulse - oldImpulse;

		const p = this.u.times(impulse);
		vA.sub(p.times(mA));
		vB.add(p.times(mB));
		wA -= iA * rA.cross(p);
		wB += iB * rB.cross(p);

		this.bodyA.velocity.set(vA);
		this.bodyB.velocity.set(vB);
		this.bodyA.angularVelocity = wA;
		this.bodyB.angularVelocity = wB;
	}
	positionalCorrection() {
		const rA = this.bodyA.transform.times(this.anchorA);
		const rB = this.bodyB.transform.times(this.anchorB);

		const cA = this.bodyA.position;
		const cB = this.bodyB.position;

		const mA = this.bodyA.mass.iM;
		const mB = this.bodyB.mass.iM;
		const iA = this.bodyA.mass.iI;
		const iB = this.bodyB.mass.iI;

		const u = cB.plus(rB).sub(cA.plus(rA));
		const length = u.length;
		u.mul(1 / length);
		const c = Math.clamp(length - this.limit, 0, .2);
		const impulse = -this.mass * c;

		const p = u.times(impulse);
		this.bodyA.position.sub(p.times(mA));
		this.bodyB.position.add(p.times(mB));
		this.bodyA.transform.radians -= iA * rA.cross(p);
		this.bodyB.transform.radians += iB * rB.cross(p);
	}
	clone(bodyA, bodyB) {
		const clone = Object.create(RopeJoint.prototype);
		Joint.clone(clone, this, bodyA, bodyB);
		clone.limit = this.limit;
		clone.u = this.u.clone();
		clone.mass = this.mass;
		clone.impulse = this.impulse;
		clone.distance = this.distance;
		return clone;
	}
};
