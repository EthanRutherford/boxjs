const {
	rgba,
	builtIn: {
		Shape,
		VectorMaterial,
	},
} = require("2d-gl");
const {
	Math: {Vector2D, clamp},
	Shapes: {Polygon},
	Joints: {WheelJoint},
} = require("../../src/box");

class PID {
	constructor(kP, kI, kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.integral = 0;
		this.prevErr = 0;
	}
	calc(target, current, dt) {
		const err = target - current;
		this.integral += err * dt;
		const errDiff = (err - this.prevErr) / dt;
		this.prevErr = err;

		const P = err * this.kP;
		const I = this.integral * this.kI;
		const D = errDiff * this.kD;

		return P + I + D;
	}
	clone() {
		const clone = Object.create(PID.prototype);
		clone.kP = this.kP;
		clone.kI = this.kI;
		clone.kD = this.kD;
		clone.integral = this.integral;
		clone.prevErr = this.prevErr;
		return clone;
	}
}

const yellow = rgba(1, 1, 0, 1);
const cyan = rgba(0, 1, 1, 1);

// shapes/materials
const groundShape = new Shape(
	new Polygon().setAsBox(1000, .5).originalPoints,
);
const groundMaterial = new VectorMaterial(
	[yellow, yellow, yellow, yellow],
);
const rectShape = new Shape(new Polygon().setAsBox(.25, 1).originalPoints);
const rectMaterial = new VectorMaterial(
	[cyan, cyan, cyan, cyan],
);

// variables
const pid = new PID(25, 16, 1);
let body = null;
let wheel = null;
let joint = null;
let dir = 0;

const onKeyDown = (event) => {
	if (event.key === "a" || event.key === "A") {
		dir = -1;
	} else if (event.key === "d" || event.key === "D") {
		dir = 1;
	}
};

const onKeyUp = (event) => {
	if (event.key === "a" || event.key === "A") {
		dir = 0;
	}
	if (event.key === "d" || event.key === "D") {
		dir = 0;
	}
};

// initialization method
function create({getSolver, createBody, createBall}) {
	const solver = getSolver();

	body = createBody({
		position: new Vector2D(0, 1.9),
		shapes: [new Polygon().setAsBox(.25, 1)],
		filterGroup: 3,
		exclusionList: [3],
	}, rectShape, rectMaterial);
	wheel = createBall(0, 1, {
		filterGroup: 3,
		exclusionList: [3],
	});
	createBody({
		position: new Vector2D(0, 0),
		shapes: [new Polygon().setAsBox(1000, .5)],
		static: true,
	}, groundShape, groundMaterial);

	joint = new WheelJoint({
		bodyA: body,
		bodyB: wheel,
		anchorA: new Vector2D(0, -.9),
		anchorB: new Vector2D(0, 0),
		axis: new Vector2D(0, 1),
		frequency: 4,
		damping: .7,
	});

	solver.addJoint(joint);

	registerCallbacks();
}

// register some callbacks
function registerCallbacks() {
	joint.setMotor(0, 10);
	dir = 0;

	window.addEventListener("keydown", onKeyDown);
	window.addEventListener("keyup", onKeyUp);
}

// on step method
function step({dt, prevCam, curCam}) {
	// keep body awake
	body.setAsleep(false);

	const vx = (body.velocity.x + wheel.velocity.x) / 2;
	const angle = dir === 0 ? clamp(vx / 2, -1, 1) * Math.PI / 8 : -dir * Math.PI / 8;
	const speed = -pid.calc(angle, body.transform.radians, dt);
	joint.setMotor(speed, 10);

	// do camera thing
	prevCam.set(curCam);
	const error = body.position.minus(curCam);
	const correction = error.times(.1);
	curCam.add(correction);
}

// clean up the test
function cleanUp() {
	window.removeEventListener("keydown", onKeyDown);
	window.removeEventListener("keyup", onKeyUp);
}

module.exports = {
	create,
	step,
	cleanUp,
};
