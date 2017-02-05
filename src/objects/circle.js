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
		this.aabb.max.set(
			this.body.position.x + this.radius,
			this.body.position.y + this.radius
		);
		this.aabb.min.set(
			this.body.position.x - this.radius,
			this.body.position.y - this.radius
		);
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
