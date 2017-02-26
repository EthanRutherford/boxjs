const {Vector2D} = require("../framework/math.js");
const MassData = require("./mass.js");
const {AABB, Shape} = require("./shape.js");

module.exports = class Circle extends Shape {
	constructor(radius) {
		super();
		this.body = null;
		this.aabb = new AABB();
		this.radius = radius;
	}
	setAABB() {
		this.aabb.max.set({
			x: this.body.position.x + this.radius,
			y: this.body.position.y + this.radius,
		});
		this.aabb.min.set({
			x: this.body.position.x - this.radius,
			y: this.body.position.y - this.radius,
		});
	}
	raycast({p1, p2, maxFraction}) {
		let s = p1.minus(this.body.position);
		let b = s.dot(s) - Math.sqr(this.radius);
		let r = p2.minus(p1);
		let c = s.dot(r);
		let rr = r.dot(r);
		let sigma = c * c - rr * b;

		if (sigma < 0 || rr < Number.EPSILON) {
			return null;
		}

		let fraction = -(c + Math.sqrt(sigma));

		if (fraction >= 0 && fraction <= maxFraction * rr) {
			fraction /= rr;
			let normal = s.plus(r.times(fraction)).normalize();
			return {fraction, normal};
		}

		return null;
	}
	computeMass(density) {
		let mass = new MassData();
		mass.center = new Vector2D(0, 0);
		mass.m = Math.PI * Math.sqr(this.radius) * density;
		mass.iM = mass.m ? 1 / mass.m : 0;
		mass.i = mass.m * Math.sqr(this.radius);
		mass.iI = mass.i ? 1 / mass.i : 0;
		return mass;
	}
	clone() {
		return new Circle(this.radius);
	}
};
