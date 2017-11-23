const {Vector2D} = require("../framework/math");
const MassData = require("./mass");
const {AABB, Shape} = require("./shape");

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

		this.originalPoints = this.points.map((point) => point.clone());
		return this;
	}
	set(points) {
		if (points.length < 3) {
			throw new Error("Can't create polygon with fewer than 3 points");
		}

		let rightmost = 0;
		let maxX = points[0].x;
		for (let i = 1; i < points.length; i++) {
			const x = points[i].x;
			if (x > maxX) {
				maxX = x;
				rightmost = i;
			} else if (x === maxX && points[i].y < points[rightmost].y) {
				rightmost = i;
			}
		}
		const hull = [];
		let index = rightmost;
		while (true) {
			hull.push(index);
			let nextIndex = 0;
			for (let i = 1; i < points.length; i++) {
				if (nextIndex === index) {
					nextIndex = i;
					continue;
				}
				const e1 = points[nextIndex].minus(points[index]);
				const e2 = points[i].minus(points[index]);
				const c = e1.cross(e2);
				if (c < 0 || (c === 0 && e2.lsqr > e1.lsqr)) {
					nextIndex = i;
				}
			}
			index = nextIndex;
			if (nextIndex === rightmost) {
				break;
			}
		}
		for (const i of hull) {
			this.points.push(points[i].clone());
		}
		for (let i = 0; i < this.points.length; i++) {
			const j = i + 1 < this.points.length ? i + 1 : 0;
			const edge = this.points[j].minus(this.points[i]);
			this.norms.push(edge.nskew.normalize());
		}

		this.originalPoints = this.points.map((point) => point.clone());
		return this;
	}
	setAABB() {
		this.aabb.min.x = Number.MAX_VALUE;
		this.aabb.min.y = Number.MAX_VALUE;
		this.aabb.max.x = -Number.MAX_VALUE;
		this.aabb.max.y = -Number.MAX_VALUE;
		for (const point of this.points) {
			const v = this.body.position.plus(this.body.transform.times(point));
			if (v.x < this.aabb.min.x) this.aabb.min.x = v.x;
			if (v.y < this.aabb.min.y) this.aabb.min.y = v.y;
			if (v.x > this.aabb.max.x) this.aabb.max.x = v.x;
			if (v.y > this.aabb.max.y) this.aabb.max.y = v.y;
		}
	}
	raycast({p1, p2, maxFraction}) {
		p1 = this.body.transform.transpose.times(p1.minus(this.body.position));
		p2 = this.body.transform.transpose.times(p2.minus(this.body.position));
		const d = p2.minus(p1);

		let low = 0;
		let hi = maxFraction;
		let index = -1;

		for (let i = 0; i < this.points.length; i++) {
			const numer = this.norms[i].dot(this.points[i].minus(p1));
			const denom = this.norms[i].dot(d);

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
			const fraction = low;
			const normal = this.body.transform.transpose.times(this.norms[index]);
			return {fraction, normal};
		}

		return null;
	}
	computeMass(density) {
		const mass = new MassData();
		let area = 0;
		let inertia = 0;
		for (let i = 0; i < this.points.length; i++) {
			const j = i + 1 < this.points.length ? i + 1 : 0;
			const p1 = this.points[i];
			const p2 = this.points[j];
			const d = p1.cross(p2);
			const triangleArea = d * .5;
			area += triangleArea;
			mass.center.add(p1.plus(p2).mul(triangleArea * inv3));
			const intx2 = (p1.x ** 2) + (p2.x ** 2) + p1.x * p2.x;
			const inty2 = (p1.y ** 2) + (p2.y ** 2) + p1.y * p2.y;
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
		for (const vertex of this.points) {
			const projection = vertex.dot(direction);
			if (projection > bestProjection) {
				bestProjection = projection;
				bestVertex = vertex;
			}
		}
		return bestVertex;
	}
	recenter(offset) {
		for (const point of this.points) {
			point.sub(offset);
		}
	}
	clone() {
		const clone = Object.create(Polygon.prototype);
		clone.id = this.id;
		clone.body = null;
		clone.aabb = new AABB();
		clone.points = [];
		for (const point of this.points) {
			clone.points.push(point.clone());
		}
		clone.norms = [];
		for (const norm of this.norms) {
			clone.norms.push(norm.clone());
		}
		clone.originalPoints = this.originalPoints.map(
			(point) => point.clone(),
		);
		return clone;
	}
};
