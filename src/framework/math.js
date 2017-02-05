//extend Math with some convenience functions
Math.sqr = (x) => x * x;
Math.clamp = (value, min, max) => Math.max(min, Math.min(value, max));

class Vector2D {
	constructor(x, y) {
		this.x = x || 0;
		this.y = y || 0;
	}
	set({x, y}) {
		if (x != null) this.x = x;
		if (y != null) this.y = y;
	}
	neg() {
		return new Vector2D(-this.x, -this.y);
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
		return Math.sqr(this.x) + Math.sqr(this.y);
	}
	normalize() {
		if (this.length < Number.EPSILON)
			return this;
		let invLength = 1 / this.length;
		this.mul(invLength);
		return this;
	}
	get skew() {
		return new Vector2D(-this.y, this.x);
	}
	get nskew() {
		return new Vector2D(this.y, -this.x);
	}
	static clone({x, y} = {x: 0, y: 0}) {
		return new Vector2D(x, y);
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

class Vector3D {
	constructor(x, y, z) {
		this.x = x || 0;
		this.y = y || 0;
		this.z = z || 0;
	}
	set({x, y, z}) {
		if (x != null) this.x = x;
		if (y != null) this.y = y;
		if (z != null) this.z = z;
	}
	neg() {
		return new Vector3D(-this.x, -this.y, -this.z);
	}
	add(other) {
		this.x += other.x;
		this.y += other.y;
		this.z += other.z;
	}
	sub(other) {
		this.x -= other.x;
		this.y -= other.y;
		this.z -= other.z;
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
			this.x * other.y - this.y * other.x
		);
	}
}

class Matrix2D {
	constructor(a, b, c, d) {
		if (a == null) {
			this.ii = 0;
			this.ij = 0;
			this.ji = 0;
			this.jj = 0;
		} else if (c == null) {
			this.ii = a.x;
			this.ij = a.y;
			this.ji = b.x;
			this.jj = b.y;
		} else {
			this.ii = a;
			this.ij = b;
			this.ji = c;
			this.jj = d;
		}
	}
	set({ii, ij, ji, jj}) {
		if (ii != null) this.ii = ii;
		if (ij != null) this.ij = ij;
		if (ji != null) this.ji = ji;
		if (jj != null) this.jj = jj;
	}
	setRotation(radians) {
		let c = Math.cos(radians);
		let s = Math.sin(radians);
		this.ii = c;
		this.ij = -s;
		this.ji = s;
		this.jj = c;
	}
	plus(other) {
		return new Matrix2D(this.ii + other.ii, this.ij + other.ij, this.ji + other.ji, this.jj + other.jj);
	}
	times(vector) {
		return new Vector2D(this.ii * vector.x + this.ij * vector.y, this.ji * vector.x + this.jj * vector.y);
	}
	get determinant() {
		let det = this.ii * this.jj - this.ij * this.ji;
		if (det !== 0)
			det = 1 / det;
		return det;
	}
	get inverse() {
		let d = this.determinant;
		return new Matrix2D(d * this.jj, -d * this.ij, -d * this.ji, d * this.ii);
	}
	get transpose() {
		return new Matrix2D(this.ii, this.ji, this.ij, this.jj);
	}
	solve(vector) {
		let d = this.determinant;
		return new Vector2D(d * (this.jj * vector.x - this.ij * vector.y),
			d * (this.ii * vector.y - this.ji * vector.x));
	}
	static get Identity() {
		return identity;
	}
}

//specialization of a Matrix used for rotations
class Rotation extends Matrix2D {
	constructor(radians = 0) {
		super();
		this.radians = radians;
	}
	set(value) {
		this.radians = value;
	}
	get radians() {
		return this._r;
	}
	set radians(value) {
		value = cleanAngle(value);
		this.setRotation(value);
		this._r = value;
	}
}

class Matrix3D {
	constructor(a, b, c, d, e, f, g, h, i) {
		if (a == null)
			this.m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
		else
			this.m = [[a, b, c], [d, e, f], [g, h, i]];
	}
	solve2(vec2) {
		let m = this.m;
		let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
		if (det !== 0)
			det = 1 / det;
		return new Vector2D(det * (m[1][1] * vec2.x - m[0][1] * vec2.y),
			det * (m[0][0] * vec2.y - m[1][0] * vec2.x));
	}
	solve3(vec3) {
		let m = this.m;
		let ex = new Vector3D(m[0][0], m[1][0], m[2][0]);
		let ey = new Vector3D(m[0][1], m[1][1], m[2][1]);
		let ez = new Vector3D(m[0][2], m[1][2], m[2][2]);

		let det = ex.dot(ey.cross(ez));
		if (det !== 0)
			det = 1 / det;

		return new Vector3D(
			det * vec3.dot(ey.cross(ez)),
			det * ex.dot(vec3.cross(ez)),
			det * ex.dot(ey.cross(vec3))
		);
	}
}

//matrix identity
const identity = new Matrix2D(1, 0, 0, 1);

//inverse of 2pi
const i2pi = 1 / (2 * Math.PI);

//wrap angle between -pi and pi
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
};
