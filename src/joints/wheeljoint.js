const {Vector2D} = require("../framework/math");
const Joint = require("./joint");

module.exports = class WheelJoint extends Joint {
	constructor({bodyA, bodyB, anchorA, anchorB, axis, frequency, damping}) {
		super({bodyA, bodyB, anchorA, anchorB});
		this.axis = axis.clone();
		this.frequency = frequency;
		this.damping = damping;

		this.motorOn = false;
		this.motorSpeed = 0;
		this.motorTorqueLimit = 0;

		this.ax = new Vector2D(0, 0);
		this.ay = new Vector2D(0, 0);
		this.sAx = 0;
		this.sAy = 0;
		this.sBx = 0;
		this.sBy = 0;
		this.impulse = 0;
		this.motorImpulse = 0;
		this.springImpulse = 0;
		this.mass = 0;
		this.motorMass = 0;
		this.springMass = 0;
		this.bias = 0;
		this.gamma = 0;
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

		const d = cB.plus(rB).sub(cA.plus(rA));

		this.ay = this.bodyA.transform.times(Vector2D.cross1x2(1, this.axis));
		this.sAy = d.plus(rA).cross(this.ay);
		this.sBy = rB.cross(this.ay);
		this.mass = mA + mB + iA * Math.sqr(this.sAy) + iB * Math.sqr(this.sBy);
		if (this.mass > 0) {
			this.mass = 1 / this.mass;
		}

		this.springMass = 0;
		this.bias = 0;
		this.gamma = 0;
		if (this.frequency > 0) {
			this.ax = this.bodyA.transform.times(this.axis);
			this.sAx = d.plus(rA).cross(this.ax);
			this.sBx = rB.cross(this.ax);
			const invMass = mA + mB + iA * Math.sqr(this.sAx) + iB * Math.sqr(this.sBx);

			if (invMass > 0) {
				this.springMass = 1 / invMass;
				const c = d.dot(this.ax);
				const omega = 2 * Math.PI * this.frequency;
				const damp = 2 * this.springMass * this.damping * omega;
				const k = this.springMass * Math.sqr(omega);

				this.gamma = dt * (damp + dt * k);
				if (this.gamma > 0) {
					this.gamma = 1 / this.gamma;
				}

				this.bias = c * dt * k * this.gamma;
				this.springMass = invMass + this.gamma;
				if (this.springMass > 0) {
					this.springMass = 1 / this.springMass;
				}
			}
		} else {
			this.springImpulse = 0;
		}

		if (this.motorOn) {
			this.motorMass = iA + iB;
			if (this.motorMass > 0) {
				this.motorMass = 1 / this.motorMass;
			}
		} else {
			this.motorMass = 0;
			this.motorImpulse = 0;
		}

		const p = this.ay.times(this.impulse).add(this.ax.times(this.springImpulse));
		const lA = this.sAy * this.impulse + this.sAx * this.springImpulse + this.motorImpulse;
		const lB = this.sBy * this.impulse + this.sBx * this.springImpulse + this.motorImpulse;

		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * lA;
		this.bodyB.angularVelocity += iB * lB;
	}
	applyImpulse(dt) {
		const vA = this.bodyA.velocity;
		const vB = this.bodyB.velocity;
		let wA = this.bodyA.angularVelocity;
		let wB = this.bodyB.angularVelocity;

		const mA = this.bodyA.mass.iM;
		const mB = this.bodyB.mass.iM;
		const iA = this.bodyA.mass.iI;
		const iB = this.bodyB.mass.iI;

		{ //spring constraint
			const cDot = this.ax.dot(vB.minus(vA)) + this.sBx * wB - this.sAx * wA;
			const impulse = -(cDot + this.bias + this.gamma * this.springImpulse) * this.springMass;
			this.springImpulse += impulse;
			const p = this.ax.times(impulse);
			const lA = impulse * this.sAx;
			const lB = impulse * this.sBx;

			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * lA;
			wB += iB * lB;
		}

		{ //motor constraint
			const cDot = wB - wA - this.motorSpeed;
			let impulse = -this.motorMass * cDot;

			const oldImpulse = this.motorImpulse;
			const maxImpulse = dt * this.motorTorqueLimit;
			this.motorImpulse = Math.clamp(this.motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.motorImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iA * impulse;
		}

		{ //axis constraint
			const cDot = this.ay.dot(vB.minus(vA)) + this.sBy * wB - this.sAy * wA;
			const impulse = -this.mass * cDot;
			this.impulse += impulse;
			const p = this.ay.times(impulse);
			const lA = impulse * this.sAy;
			const lB = impulse * this.sBy;

			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * lA;
			wB += iB * lB;
		}

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

		const d = cB.plus(rB).sub(cA.plus(rA));

		const ay = this.bodyA.transform.times(Vector2D.cross1x2(1, this.axis));
		const sAy = d.plus(rA).cross(this.ay);
		const sBy = rB.cross(this.ay);

		const c = d.dot(ay);
		const k = mA + mB + iA * Math.sqr(this.sAy) + iB * Math.sqr(this.sBy);

		let impulse = 0;
		if (k !== 0) {
			impulse = -c / k;
		}

		const p = ay.times(impulse);
		const lA = impulse * sAy;
		const lB = impulse * sBy;

		this.bodyA.position.sub(p.times(mA));
		this.bodyB.position.add(p.times(mB));
		this.bodyA.transform.radians -= iA * lA;
		this.bodyB.transform.radians += iB * lB;
	}
	setMotor(speed, torque = 0) {
		this.motorOn = speed != null;
		this.motorSpeed = speed;
		this.motorTorqueLimit = torque;
	}
	clone(bodyA, bodyB) {
		const clone = Object.create(WheelJoint.prototype);
		Joint.clone(clone, this, bodyA, bodyB);
		clone.axis = this.axis.clone();
		clone.frequency = this.frequency;
		clone.damping = this.damping;
		clone.motorOn = this.motorOn;
		clone.motorSpeed = this.motorSpeed;
		clone.motorTorqueLimit = this.motorTorqueLimit;
		clone.ax = this.ax.clone();
		clone.ay = this.ay.clone();
		clone.sAx = this.sAx;
		clone.sAy = this.sAy;
		clone.sBx = this.sBx;
		clone.sBy = this.sBy;
		clone.impulse = this.impulse;
		clone.motorImpulse = this.motorImpulse;
		clone.springImpulse = this.springImpulse;
		clone.mass = this.mass;
		clone.motorMass = this.motorMass;
		clone.springMass = this.springMass;
		clone.bias = this.bias;
		clone.gamma = this.gamma;
	}
};
