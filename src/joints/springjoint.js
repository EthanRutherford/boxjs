const {Vector2D} = require("../framework/math");
const Joint = require("./joint");

module.exports = class SpringJoint extends Joint {
	constructor({bodyA, bodyB, anchorA, anchorB, length, frequency, damping}) {
		super({bodyA, bodyB, anchorA, anchorB});
		this.length = length;
		this.frequency = frequency;
		this.damping = damping;

		this.u = new Vector2D(0, 0);
		this.mass = 0;
		this.impulse = 0;
		this.gamma = 0;
		this.bias = 0;
	}
	initialize(dt) {
		const rA = this.bodyA.transform.times(this.anchorA);
		const rB = this.bodyB.transform.times(this.anchorB);

		const cA = this.bodyA.position;
		const cB = this.bodyB.position;

		const mA = this.bodyA.mass.iM;
		const mB = this.bodyB.mass.iM;
		const iA = this.bodyA.mass.iI;
		const iB = this.bodyB.mass.iI;

		this.u = cB.plus(rB).sub(cA.plus(rA));
		const length = this.u.length;

		if (length > .005) {
			this.u.mul(1 / length);
		} else {
			this.u.x = 0;
			this.u.y = 0;
		}

		const crA = rA.cross(this.u);
		const crB = rB.cross(this.u);
		let invMass = mA + iA * Math.sqr(crA) + mB + iB * Math.sqr(crB);
		this.mass = invMass !== 0 ? 1 / invMass : 0;

		if (this.frequency > 0) {
			const c = length - this.length;
			const omega = 2 * Math.PI * this.frequency;
			const d = 2 * this.mass * this.damping * omega;
			const k = this.mass * Math.sqr(omega);

			this.gamma = dt * (d + dt * k);
			this.gamma = this.gamma !== 0 ? 1 / this.gamma : 0;
			this.bias = c * dt * k * this.gamma;

			invMass += this.gamma;
			this.mass = invMass !== 0 ? 1 / invMass : 0;
		} else {
			this.gamma = 0;
			this.bias = 0;
		}

		const p = this.u.times(this.impulse);
		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * rA.cross(p);
		this.bodyB.angularVelocity += iB * rB.cross(p);
	}
	applyImpulse() {
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
		const cDot = this.u.dot(vpB.minus(vpA));

		const impulse = -this.mass * (cDot + this.bias + this.gamma * this.impulse);
		this.impulse += impulse;

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
		if (this.frequency > 0) {
			return;
		}

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
		const c = Math.clamp(length - this.length, -.2, .2);
		const impulse = -this.mass * c;

		const p = u.times(impulse);
		this.bodyA.position.sub(p.times(mA));
		this.bodyB.position.add(p.times(mB));
		this.bodyA.transform.radians -= iA * rA.cross(p);
		this.bodyB.transform.radians += iB * rB.cross(p);
	}
	clone(bodyA, bodyB) {
		const clone = Object.create(SpringJoint.prototype);
		Joint.clone(clone, this, bodyA, bodyB);
		clone.length = this.length;
		clone.frequency = this.frequency;
		clone.damping = this.damping;
		clone.u = this.u.clone();
		clone.mass = this.mass;
		clone.impulse = this.impulse;
		clone.gamma = this.gamma;
		clone.bias = this.bias;
	}
};
