const {Vector2D} = require("../framework/math.js");
const Joint = require("./joint.js");

module.exports = class SpringJoint extends Joint {
	constructor({bodyA, bodyB, anchorA, anchorB, length, frequency, damping}) {
		super({bodyA, bodyB, anchorA, anchorB});
		this.length = length;
		this.frequency = frequency;
		this.damping = damping;

		this.u = new Vector2D();
		this.mass = 0;
		this.impulse = 0;
		this.gamma = 0;
		this.bias = 0;
	}
	initialize(dt) {
		let rA = this.bodyA.transform.times(this.anchorA);
		let rB = this.bodyB.transform.times(this.anchorB);

		let cA = this.bodyA.position;
		let cB = this.bodyB.position;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		this.u = cB.plus(rB).minus(cA.plus(rA));
		let length = this.u.length;

		if (length > .005) {
			this.u.mul(1 / length);
		} else {
			this.u.set({x: 0, y: 0});
		}

		let crA = rA.cross(this.u);
		let crB = rB.cross(this.u);
		let invMass = mA + iA * Math.sqr(crA) + mB + iB * Math.sqr(crB);
		this.mass = invMass !== 0 ? 1 / invMass : 0;

		if (this.frequency > 0) {
			let c = length - this.length;
			let omega = 2 * Math.PI * this.frequency;
			let d = 2 * this.mass * this.damping * omega;
			let k = this.mass * Math.sqr(omega);

			this.gamma = dt * (d + dt * k);
			this.gamma = this.gamma !== 0 ? 1 / this.gamma : 0;
			this.bias = c * dt * k * this.gamma;

			invMass += this.gamma;
			this.mass = invMass !== 0 ? 1 / invMass : 0;
		} else {
			this.gamma = 0;
			this.bias = 0;
		}

		let p = this.u.times(this.impulse);
		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * rA.cross(p);
		this.bodyB.angularVelocity += iB * rB.cross(p);
	}
	applyImpulse() {
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
		let cDot = this.u.dot(vpB.minus(vpA));

		let impulse = -this.mass * (cDot + this.bias + this.gamma * this.impulse);
		this.impulse += impulse;

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
		if (this.frequency > 0) {
			return;
		}

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
		let c = Math.clamp(length - this.length, -.2, .2);
		let impulse = -this.mass * c;

		let p = u.times(impulse);
		this.bodyA.position.sub(p.times(mA));
		this.bodyB.position.add(p.times(mB));
		this.bodyA.transform.radians -= iA * rA.cross(p);
		this.bodyB.transform.radians += iB * rB.cross(p);
	}
};
