const {Vector2D} = require("../framework/math");
let next = 0;

class AABB {
	constructor(mx = 0, my = 0, Mx = 0, My = 0) {
		this.min = new Vector2D(mx, my);
		this.max = new Vector2D(Mx, My);
	}
	test(other) {
		if (this.max.x < other.min.x || this.min.x > other.max.x ||
			this.max.y < other.min.y || this.min.y > other.max.y) {
			return false;
		}

		return true;
	}
	contains(other) {
		if (this.max.x < other.max.x || this.min.x > other.min.x ||
			this.max.y < other.max.y || this.min.y > other.min.y) {
			return false;
		}
		return true;
	}
	get perimeter() {
		return (this.max.x - this.min.x + this.max.y - this.min.y) * 2;
	}
	combine(other) {
		return new AABB(
			Math.min(this.min.x, other.min.x),
			Math.min(this.min.y, other.min.y),
			Math.max(this.max.x, other.max.x),
			Math.max(this.max.y, other.max.y)
		);
	}
	clone() {
		return new AABB(this.min.x, this.min.y, this.max.x, this.max.y);
	}
}

class Shape {
	constructor() {
		this.id = next++;
	}
	static order(a, b) {
		if (a.id < b.id) {
			return [a, b];
		}

		return [b, a];
	}
}

module.exports = {
	AABB,
	Shape,
};
