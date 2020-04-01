class Vector2D {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}
	set(other) {
		this.x = other.x;
		this.y = other.y;
		return this;
	}
	neg() {
		return new Vector2D(-this.x, -this.y);
	}
	negate() {
		this.x = -this.x;
		this.y = -this.y;
		return this;
	}
	add(other) {
		this.x += other.x;
		this.y += other.y;
		return this;
	}
	sub(other) {
		this.x -= other.x;
		this.y -= other.y;
		return this;
	}
	mul(scale) {
		this.x *= scale;
		this.y *= scale;
		return this;
	}
	plus(other) {
		return new Vector2D(this.x + other.x, this.y + other.y);
	}
	minus(other) {
		return new Vector2D(this.x - other.x, this.y - other.y);
	}
	times(scale) {
		return new Vector2D(this.x * scale, this.y * scale);
	}
	dot(other) {
		return this.x * other.x + this.y * other.y;
	}
	cross(other) {
		return this.x * other.y - this.y * other.x;
	}
	eq(other) {
		return this.x === other.x && this.y === other.y;
	}
	get length() {
		return Math.sqrt(this.lsqr);
	}
	get lsqr() {
		return (this.x ** 2) + (this.y ** 2);
	}
	normalize() {
		const length = this.length;
		if (length < Number.EPSILON) {
			return this;
		}

		return this.mul(1 / length);
	}
	clone() {
		return new Vector2D(this.x, this.y);
	}
	static clone(other) {
		return new Vector2D(other.x, other.y);
	}
	static cross2x1(a, s) {
		return new Vector2D(s * a.y, -s * a.x);
	}
	static cross1x2(s, b) {
		return new Vector2D(-s * b.y, s * b.x);
	}
	static diff(a, b) {
		return a.minus(b).length;
	}
	static diffsqr(a, b) {
		return a.minus(b).lsqr;
	}
}

Vector2D.zero = new Vector2D(0, 0);

class Vector3D {
	constructor(x, y, z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	set(other) {
		this.x = other.x;
		this.y = other.y;
		this.z = other.z;
		return this;
	}
	neg() {
		return new Vector3D(-this.x, -this.y, -this.z);
	}
	negate() {
		this.x = -this.x;
		this.y = -this.y;
		this.z = -this.z;
		return this;
	}
	add(other) {
		this.x += other.x;
		this.y += other.y;
		this.z += other.z;
		return this;
	}
	sub(other) {
		this.x -= other.x;
		this.y -= other.y;
		this.z -= other.z;
		return this;
	}
	plus(other) {
		return new Vector3D(this.x + other.x, this.y + other.y, this.z + other.z);
	}
	minus(other) {
		return new Vector3D(this.x - other.x, this.y - other.y, this.z - other.z);
	}
	dot(other) {
		return this.x * other.x + this.y * other.y + this.z * other.z;
	}
	cross(other) {
		return new Vector3D(
			this.y * other.z - this.z * other.y,
			this.z * other.x - this.x * other.z,
			this.x * other.y - this.y * other.x,
		);
	}
	clone() {
		return new Vector3D(this.x, this.y, this.z);
	}
}

class Matrix2D {
	constructor(a, b, c, d) {
		this.ii = a;
		this.ij = b;
		this.ji = c;
		this.jj = d;
	}
	set(other) {
		this.ii = other.ii;
		this.ij = other.ij;
		this.ji = other.ji;
		this.jj = other.jj;
		return this;
	}
	plus(other) {
		return new Matrix2D(this.ii + other.ii, this.ij + other.ij, this.ji + other.ji, this.jj + other.jj);
	}
	times(vector) {
		return new Vector2D(this.ii * vector.x + this.ij * vector.y, this.ji * vector.x + this.jj * vector.y);
	}
	get determinant() {
		let det = this.ii * this.jj - this.ij * this.ji;
		if (det !== 0) {
			det = 1 / det;
		}

		return det;
	}
	inverse() {
		const d = this.determinant;
		return new Matrix2D(d * this.jj, -d * this.ij, -d * this.ji, d * this.ii);
	}
	transpose() {
		return new Matrix2D(this.ii, this.ji, this.ij, this.jj);
	}
	solve(vector) {
		const d = this.determinant;
		return new Vector2D(d * (this.jj * vector.x - this.ij * vector.y),
			d * (this.ii * vector.y - this.ji * vector.x));
	}
	clone() {
		return new Matrix2D(this.ii, this.ij, this.ji, this.jj);
	}
}

Matrix2D.identity = new Matrix2D(1, 0, 0, 1);

class Rotation {
	constructor(radians) {
		this.radians = radians;
	}
	get radians() {
		return this.r;
	}
	set radians(value) {
		value = cleanAngle(value);
		this.r = value;
		this.c = Math.cos(value);
		this.s = Math.sin(value);
	}
	times(v) {
		const {s, c} = this;
		return new Vector2D(c * v.x - s * v.y, s * v.x + c * v.y);
	}
	mul(v) {
		const {s, c} = this;
		const x = c * v.x - s * v.y;
		const y = s * v.x + c * v.y;
		v.x = x;
		v.y = y;
		return v;
	}
	transpose() {
		return Rotation.from(-this.r, this.c, -this.s);
	}
	clone() {
		return Rotation.from(this.r, this.c, this.s);
	}
	static from(r, c, s) {
		const rot = Object.create(Rotation.prototype);
		rot.r = r;
		rot.c = c;
		rot.s = s;
		return rot;
	}
}

class Matrix3D {
	constructor(a, b, c, d, e, f, g, h, i) {
		this.m = [[a, b, c], [d, e, f], [g, h, i]];
	}
	solve2(vec2) {
		const m = this.m;
		let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
		if (det !== 0) {
			det = 1 / det;
		}

		return new Vector2D(det * (m[1][1] * vec2.x - m[0][1] * vec2.y),
			det * (m[0][0] * vec2.y - m[1][0] * vec2.x));
	}
	solve3(vec3) {
		const m = this.m;
		const ex = new Vector3D(m[0][0], m[1][0], m[2][0]);
		const ey = new Vector3D(m[0][1], m[1][1], m[2][1]);
		const ez = new Vector3D(m[0][2], m[1][2], m[2][2]);

		let det = ex.dot(ey.cross(ez));
		if (det !== 0) {
			det = 1 / det;
		}

		return new Vector3D(
			det * vec3.dot(ey.cross(ez)),
			det * ex.dot(vec3.cross(ez)),
			det * ex.dot(ey.cross(vec3)),
		);
	}
	clone() {
		return new Matrix3D(...this.m[0], ...this.m[1], ...this.m[2]);
	}
}

// inverse of 2pi
const i2pi = 1 / (2 * Math.PI);

// wrap angle between -pi and pi
function cleanAngle(angle) {
	return angle - 2 * Math.PI * Math.trunc(angle * i2pi + Math.sign(angle) * .5);
}

module.exports = {
	Vector2D,
	Vector3D,
	Matrix2D,
	Rotation,
	Matrix3D,
	cleanAngle,
	bigG: 6.674 * Math.pow(10, -11),
	clamp: (value, min, max) => Math.max(min, Math.min(value, max)),
};
