const {Vector2D} = require("../framework/math.js");
const MassData = require("./mass.js");
const {AABB, Shape} = require("./shape.js");

const inv3 = 1 / 3;

module.exports = class Polygon extends Shape {
	constructor() {
		super();
		this.body = null;
		this.aabb = new AABB();
		this.points = [];
		this.norms = [];
	}
	setAsBox(hw, hh) {
		this.points.push(new Vector2D(-hw, -hh));
		this.points.push(new Vector2D(hw, -hh));
		this.points.push(new Vector2D(hw, hh));
		this.points.push(new Vector2D(-hw, hh));
		this.norms.push(new Vector2D(0, -1));
		this.norms.push(new Vector2D(1, 0));
		this.norms.push(new Vector2D(0, 1));
		this.norms.push(new Vector2D(-1, 0));
		return this;
	}
	set(points) {
		if (points.length < 3) {
			throw new Error("Can't create polygon with fewer than 3 points");
		}

		let rightmost = 0;
		let maxX = points[0].x;
		for (let i = 1; i < points.length; i++) {
			let x = points[i].x;
			if (x > maxX) {
				maxX = x;
				rightmost = i;
			} else if (x === maxX && points[i].y < points[rightmost].y) {
				rightmost = i;
			}
		}
		let hull = [];
		let index = rightmost;
		while (true) {
			hull.push(index);
			let nextIndex = 0;
			for (let i = 1; i < points.length; i++) {
				if (nextIndex === index) {
					nextIndex = i;
					continue;
				}
				let e1 = points[nextIndex].minus(points[index]);
				let e2 = points[i].minus(points[index]);
				let c = e1.cross(e2);
				if (c < 0 || (c === 0 && e2.lsqr > e1.lsqr)) {
					nextIndex = i;
				}
			}
			index = nextIndex;
			if (nextIndex === rightmost) {
				break;
			}
		}
		for (let i of hull) {
			this.points.push(points[i].clone());
		}
		for (let i = 0; i < this.points.length; i++) {
			let j = i + 1 < this.points.length ? i + 1 : 0;
			let edge = this.points[j].minus(this.points[i]);
			this.norms.push(edge.nskew.normalize());
		}
		return this;
	}
	setAABB() {
		this.aabb.min.x = Number.MAX_VALUE;
		this.aabb.min.y = Number.MAX_VALUE;
		this.aabb.max.x = -Number.MAX_VALUE;
		this.aabb.max.y = -Number.MAX_VALUE;
		for (let point of this.points) {
			let v = this.body.position.plus(this.body.transform.times(point));
			if (v.x < this.aabb.min.x) this.aabb.min.x = v.x;
			if (v.y < this.aabb.min.y) this.aabb.min.y = v.y;
			if (v.x > this.aabb.max.x) this.aabb.max.x = v.x;
			if (v.y > this.aabb.max.y) this.aabb.max.y = v.y;
		}
	}
	raycast({p1, p2, maxFraction}) {
		p1 = this.body.transform.transpose.times(p1.minus(this.body.position));
		p2 = this.body.transform.transpose.times(p2.minus(this.body.position));
		let d = p2.minus(p1);

		let low = 0;
		let hi = maxFraction;
		let index = -1;

		for (let i = 0; i < this.points.length; i++) {
			let numer = this.norms[i].dot(this.points[i].minus(p1));
			let denom = this.norms[i].dot(d);

			if (denom === 0 && numer < 0) {
				return null;
			} else if (denom < 0 && numer < low * denom) {
				low = numer / denom;
				index = i;
			} else if (denom > 0 && numer < hi * denom) {
				hi = numer / denom;
			}

			if (hi < low) {
				return null;
			}
		}

		if (index >= 0) {
			let fraction = low;
			let normal = this.body.transform.translate.times(this.norms[index]);
			return {fraction, normal};
		}

		return null;
	}
	computeMass(density) {
		let mass = new MassData();
		let area = 0;
		let inertia = 0;
		for (let i = 0; i < this.points.length; i++) {
			let j = i + 1 < this.points.length ? i + 1 : 0;
			let p1 = this.points[i];
			let p2 = this.points[j];
			let d = p1.cross(p2);
			let triangleArea = d * .5;
			area += triangleArea;
			mass.center.add(p1.plus(p2).times(triangleArea * inv3));
			let intx2 = Math.sqr(p1.x) + Math.sqr(p2.x) + p1.x * p2.x;
			let inty2 = Math.sqr(p1.y) + Math.sqr(p2.y) + p1.y * p2.y;
			inertia += (.25 * inv3 * d) * (intx2 + inty2);
		}
		mass.center.mul(1 / area);
		mass.m = density * area;
		mass.iM = mass.m ? 1 / mass.m : 0;
		mass.i = inertia * density;
		mass.iI = mass.i ? 1 / mass.i : 0;
		return mass;
	}
	getSupport(direction) {
		let bestProjection = -Number.MAX_VALUE;
		let bestVertex;
		for (let vertex of this.points) {
			let projection = vertex.dot(direction);
			if (projection > bestProjection) {
				bestProjection = projection;
				bestVertex = vertex;
			}
		}
		return bestVertex;
	}
	clone() {
		let copy = new Polygon();
		for (let point of this.points) {
			copy.points.push(point.clone());
		}
		for (let norm of this.norms) {
			copy.norms.push(norm.clone());
		}
		return copy;
	}
	recenter(offset) {
		for (let point of this.points) {
			point.sub(offset);
		}
	}
};
