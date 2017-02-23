const {Vector2D} = require("../framework/math.js");
const Joint = require("./joint.js");

module.exports = class WheelJoint extends Joint {
	constructor({bodyA, bodyB, anchorA, anchorB, axis, frequency, damping}) {
		super({bodyA, bodyB, anchorA, anchorB});
		this.axis = axis;
		this.frequency = frequency;
		this.damping = damping;

		this.motorOn = false;
		this.motorSpeed = 0;
		this.motorTorqueLimit = 0;

		this.ax = new Vector2D();
		this.ay = new Vector2D();
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
		let rA = this.bodyA.transform.times(this.anchorA);
		let rB = this.bodyB.transform.times(this.anchorB);

		let cA = this.bodyA.position;
		let cB = this.bodyB.position;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		let d = cB.plus(rB).minus(cA.plus(rA));

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
			let invMass = mA + mB + iA * Math.sqr(this.sAx) + iB * Math.sqr(this.sBx);

			if (invMass > 0) {
				this.springMass = 1 / invMass;
				let c = d.dot(this.ax);
				let omega = 2 * Math.PI * this.frequency;
				let damp = 2 * this.springMass * this.damping * omega;
				let k = this.springMass * Math.sqr(omega);

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

		let p = this.ay.times(this.impulse).plus(this.ax.times(this.springImpulse));
		let lA = this.sAy * this.impulse + this.sAx * this.springImpulse + this.motorImpulse;
		let lB = this.sBy * this.impulse + this.sBx * this.springImpulse + this.motorImpulse;

		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * lA;
		this.bodyB.angularVelocity += iB * lB;
	}
	applyImpulse(dt) {
		let vA = this.bodyA.velocity;
		let vB = this.bodyB.velocity;
		let wA = this.bodyA.angularVelocity;
		let wB = this.bodyB.angularVelocity;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		{ //spring constraint
			let cDot = this.ax.dot(vB.minus(vA)) + this.sBx * wB - this.sAx * wA;
			let impulse = -(cDot + this.bias + this.gamma * this.springImpulse) * this.springMass;
			this.springImpulse += impulse;
			let p = this.ax.times(impulse);
			let lA = impulse * this.sAx;
			let lB = impulse * this.sBx;

			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * lA;
			wB += iB * lB;
		}

		{ //motor constraint
			let cDot = wB - wA - this.motorSpeed;
			let impulse = -this.motorMass * cDot;

			let oldImpulse = this.motorImpulse;
			let maxImpulse = dt * this.motorTorqueLimit;
			this.motorImpulse = Math.clamp(this.motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.motorImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iA * impulse;
		}

		{ //axis constraint
			let cDot = this.ay.dot(vB.minus(vA)) + this.sBy * wB - this.sAy * wA;
			let impulse = -this.mass * cDot;
			this.impulse += impulse;
			let p = this.ay.times(impulse);
			let lA = impulse * this.sAy;
			let lB = impulse * this.sBy;

			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * lA;
			wB += iB * lB;
		}

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

		let d = cB.plus(rB).minus(cA.plus(rA));

		let ay = this.bodyA.transform.times(Vector2D.cross1x2(1, this.axis));
		let sAy = d.plus(rA).cross(this.ay);
		let sBy = rB.cross(this.ay);

		let c = d.dot(ay);
		let k = mA + mB + iA * Math.sqr(this.sAy) + iB * Math.sqr(this.sBy);

		let impulse = 0;
		if (k !== 0) {
			impulse = -c / k;
		}

		let p = ay.times(impulse);
		let lA = impulse * sAy;
		let lB = impulse * sBy;

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
};
