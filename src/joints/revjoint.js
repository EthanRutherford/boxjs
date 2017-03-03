const {Vector2D, Vector3D, Matrix2D, Matrix3D, cleanAngle} = require("../framework/math.js");
const Joint = require("./joint.js");

class RevJoint extends Joint {
	constructor({bodyA, bodyB, anchorA, anchorB, upperLimit, lowerLimit}) {
		super({bodyA, bodyB, anchorA, anchorB});
		this.mass = new Matrix3D();

		if (upperLimit != null && lowerLimit != null) {
			this.setLimit(true, upperLimit, lowerLimit);
		} else {
			this.setLimit(false);
		}

		this.refAngle = cleanAngle(bodyB.transform.radians - bodyA.transform.radians);
		this.cumulativeImpulse = new Vector3D();

		this.state = -1;
	}
	initialize() {
		let rA = this.bodyA.transform.times(this.anchorA);
		let rB = this.bodyB.transform.times(this.anchorB);

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		this.mass.m[0][0] = mA + mB + Math.sqr(rA.y) * iA + Math.sqr(rB.y) * iB;
		this.mass.m[0][1] = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		this.mass.m[0][2] = -rA.y * iA - rB.y * iB;
		this.mass.m[1][0] = this.mass.m[0][1];
		this.mass.m[1][1] = mA + mB + Math.sqr(rA.x) * iA + Math.sqr(rB.x) * iB;
		this.mass.m[1][2] = rA.x * iA + rB.x * iB;
		this.mass.m[2][0] = this.mass.m[0][2];
		this.mass.m[2][1] = this.mass.m[1][2];
		this.mass.m[2][2] = iA + iB;

		if (this.limitEnabled) {
			let curAngle = this.bodyB.transform.radians - this.bodyA.transform.radians;
			curAngle = cleanAngle(curAngle - this.refAngle);
			if (curAngle <= this.lowerLimit) {
				if (this.state !== RevJoint.atLower) {
					this.cumulativeImpulse.z = 0;
				}

				this.state = RevJoint.atLower;
			} else if (curAngle >= this.upperLimit) {
				if (this.state !== RevJoint.atUpper) {
					this.cumulativeImpulse.z = 0;
				}

				this.state = RevJoint.atUpper;
			} else {
				this.state = RevJoint.between;
				this.cumulativeImpulse.z = 0;
			}
		}

		let p = new Vector2D(this.cumulativeImpulse.x, this.cumulativeImpulse.y);
		this.bodyA.velocity.sub(p.times(mA));
		this.bodyB.velocity.add(p.times(mB));
		this.bodyA.angularVelocity -= iA * (rA.cross(p) + this.cumulativeImpulse.z);
		this.bodyB.angularVelocity += iB * (rB.cross(p) + this.cumulativeImpulse.z);
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

		if (this.limitEnabled && this.state !== RevJoint.between) {
			let cDot1 = vB.plus(Vector2D.cross1x2(wB, rB)).minus(vA.plus(Vector2D.cross1x2(wA, rA)));
			let cDot2 = wB - wA;
			let cDot = new Vector3D(cDot1.x, cDot1.y, cDot2);
			let impulse = this.mass.solve3(cDot).neg();

			if (this.state === RevJoint.atLower) {
				let newImpulse = this.cumulativeImpulse.z + impulse.z;
				if (newImpulse < 0) {
					let rhs = cDot1.neg().plus({x: this.mass.m[0][2], y: this.mass.m[1][2]})
						.mul(this.cumulativeImpulse.z);
					let reduced = this.mass.solve2(rhs);
					impulse.set(reduced);
					impulse.z = -this.cumulativeImpulse.z;
					this.cumulativeImpulse.x += reduced.x;
					this.cumulativeImpulse.y += reduced.y;
					this.cumulativeImpulse.z = 0;
				} else {
					this.cumulativeImpulse.add(impulse);
				}
			} else /* RevJoint.atUpper */ {
				let newImpulse = this.cumulativeImpulse.z + impulse.z;
				if (newImpulse > 0) {
					let rhs = cDot1.neg().plus({x: this.mass.m[0][2], y: this.mass.m[1][2]})
						.mul(this.cumulativeImpulse.z);
					let reduced = this.mass.solve2(rhs);
					impulse.set(reduced);
					impulse.z = -this.cumulativeImpulse.z;
					this.cumulativeImpulse.x += reduced.x;
					this.cumulativeImpulse.y += reduced.y;
					this.cumulativeImpulse.z = 0;
				} else {
					this.cumulativeImpulse.add(impulse);
				}
			}

			let p = new Vector2D(impulse.x, impulse.y);
			vA.sub(p.times(mA));
			vB.add(p.times(mB));
			wA -= iA * (rA.cross(p) + impulse.z);
			wB += iB * (rB.cross(p) + impulse.z);
		} else {
			let cDot = vB.plus(Vector2D.cross1x2(wB, rB)).minus(vA.plus(Vector2D.cross1x2(wA, rA)));
			let impulse = this.mass.solve2(cDot.neg());
			this.cumulativeImpulse.x += impulse.x;
			this.cumulativeImpulse.y += impulse.y;
			vA.sub(impulse.times(mA));
			vB.add(impulse.times(mB));
			wA -= iA * rA.cross(impulse);
			wB += iB * rB.cross(impulse);
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
		let aA = this.bodyA.transform.radians;
		let aB = this.bodyB.transform.radians;

		let mA = this.bodyA.mass.iM;
		let mB = this.bodyB.mass.iM;
		let iA = this.bodyA.mass.iI;
		let iB = this.bodyB.mass.iI;

		if (this.limitEnabled && this.state !== RevJoint.between) {
			let angle = cleanAngle(aB - aA - this.refAngle);
			let limitImpulse = 0;
			if (this.state === RevJoint.atLower) {
				let c = angle - this.lowerLimit;
				c = Math.clamp(c + (1 / 90 * Math.PI), -(1 / 22.5 * Math.PI), 0);
				limitImpulse = -1 / (iA + iB) * c;
			} else {
				let c = angle - this.upperLimit;
				c = Math.clamp(c - (1 / 90 * Math.PI), 0, +(1 / 22.5 * Math.PI));
				limitImpulse = -1 / (iA + iB) * c;
			}
			aA -= iA * limitImpulse;
			aB += iB * limitImpulse;
		}

		let c = cB.plus(rB).minus(cA.plus(rA));
		let k = new Matrix2D(
			mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y,
			-iA * rA.x * rA.y - iB * rB.x * rB.y,
			-iA * rA.x * rA.y - iB * rB.x * rB.y,
			mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x
		);

		let impulse = k.solve(c).neg();
		cA.sub(impulse.times(mA));
		cB.add(impulse.times(mB));
		aA -= iA * rA.cross(impulse);
		aB += iB * rB.cross(impulse);

		this.bodyA.position = cA;
		this.bodyB.position = cB;
		this.bodyA.transform.radians = aA;
		this.bodyB.transform.radians = aB;
	}
	setLimit(on, upper, lower) {
		if (!on) {
			this.limitEnabled = false;
			this.upperLimit = null;
			this.lowerLimit = null;
			return;
		}

		if (upper > Math.PI || upper < -Math.PI) {
			throw new Error("upper limit out of bounts (-pi to pi)");
		}

		if (lower > Math.PI || lower < -Math.PI) {
			throw new Error("lower limit out of bounts (-pi to pi)");
		}

		if (lower > upper) {
			throw new Error("upper limit must be greater than lower limit");
		}

		this.limitEnabled = on;
		this.upperLimit = upper;
		this.lowerLimit = lower;
	}
	clone(bodyA, bodyB) {
		let clone = Object.create(RevJoint.prototype);
		Joint.clone(clone, this, bodyA, bodyB);
		clone.mass = this.mass.clone();
		clone.limitEnabled = this.limitEnabled;
		clone.upperLimit = this.upperLimit;
		clone.lowerLimit = this.lowerLimit;
		clone.refAngle = this.refAngle;
		clone.cumulativeImpulse = this.cumulativeImpulse.clone();
		clone.state = this.state;
		return clone;
	}
}

RevJoint.atUpper = 0;
RevJoint.atLower = 1;
RevJoint.between = 2;

module.exports = RevJoint;
