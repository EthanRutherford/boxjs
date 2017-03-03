const {Vector2D} = require("../framework/math.js");
const Joint = require("./joint.js");

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
		let rA = this.bodyA.transform.times(this.anchorA);
		let rB = this.bodyB.transform.times(this.anchorB);

		let cA = this.bodyA.position;
		let cB = this.bodyB.position;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		this.u = cB.plus(rB).minus(cA.plus(rA));
		this.distance = this.u.length;
		if (this.distance < .005) {
			this.u.set(0, 0);
			this.mass = 0;
			this.impulse = 0;
			return;
		}

		this.u.mul(1 / this.distance);

		let crA = rA.cross(this.u);
		let crB = rB.cross(this.u);
		let invMass = mA + iA * Math.sqr(crA) + mB + iB * Math.sqr(crB);
		this.mass = invMass !== 0 ? 1 / invMass : 0;

		let p = this.u.times(this.impulse);
		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * rA.cross(p);
		this.bodyB.angularVelocity += iB * rB.cross(p);
	}
	applyImpulse(dt) {
		let rA = this.bodyA.transform.times(this.anchorA);
		let rB = this.bodyB.transform.times(this.anchorB);

		let vA = this.bodyA.velocity;
		let vB = this.bodyB.velocity;
		let wA = this.bodyA.angularVelocity;
		let wB = this.bodyB.angularVelocity;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		let vpA = vA.plus(Vector2D.cross1x2(wA, rA));
		let vpB = vB.plus(Vector2D.cross1x2(wB, rB));
		let c = this.distance - this.limit;
		let cDot = this.u.dot(vpB.minus(vpA));
		if (c < 0) {
			cDot += 1 / dt * c;
		}

		let impulse = -this.mass * cDot;
		let oldImpulse = this.impulse;
		this.impulse = Math.min(0, this.impulse + impulse);
		impulse = this.impulse - oldImpulse;

		let p = this.u.times(impulse);
		vA.sub(p.times(mA));
		vB.add(p.times(mB));
		wA -= iA * rA.cross(p);
		wB += iB * rB.cross(p);

		this.bodyA.velocity = vA;
		this.bodyB.velocity = vB;
		this.bodyA.angularVelocity = wA;
		this.bodyB.angularVelocity = wB;
	}
	positionalCorrection() {
		let rA = this.bodyA.transform.times(this.anchorA);
		let rB = this.bodyB.transform.times(this.anchorB);

		let cA = this.bodyA.position;
		let cB = this.bodyB.position;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		let u = cB.plus(rB).minus(cA.plus(rA));
		let length = u.length;
		u.mul(1 / length);
		let c = Math.clamp(length - this.limit, 0, .2);
		let impulse = -this.mass * c;

		let p = u.times(impulse);
		this.bodyA.position.sub(p.times(mA));
		this.bodyB.position.add(p.times(mB));
		this.bodyA.transform.radians -= iA * rA.cross(p);
		this.bodyB.transform.radians += iB * rB.cross(p);
	}
	clone(bodyA, bodyB) {
		let clone = Object.create(RopeJoint.prototype);
		Joint.clone(clone, this, bodyA, bodyB);
		clone.limit = this.limit;
		clone.u = this.u.clone();
		clone.mass = this.mass;
		clone.impulse = this.impulse;
		clone.distance = this.distance;
		return clone;
	}
};
