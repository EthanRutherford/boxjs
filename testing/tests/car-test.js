const {
	rgba,
	builtIn: {
		Shape,
		VectorMaterial,
	},
} = require("2d-gl");
const {
	Math: {Vector2D},
	Shapes: {Polygon},
	Joints: {RevJoint, WheelJoint},
} = require("../../src/box");

const carVerts = [
	new Vector2D(-2, -.8),
	new Vector2D(2, -.8),
	new Vector2D(2, -.2),
	new Vector2D(0, .8),
	new Vector2D(-1.5, .8),
	new Vector2D(-2, 0),
];

const blue = rgba(0, 0, 1, 1);
const yellow = rgba(1, 1, 0, 1);
const orange = rgba(1, .5, 0, 1);
const cyan = rgba(0, 1, 1, 1);

// shapes/materials
const carShape = new Shape(carVerts);
const carMaterial = new VectorMaterial(
	[blue, blue, blue, blue, blue, blue],
);

const groundShape1 = new Shape(
	new Polygon().setAsBox(50, .5).originalPoints,
);
const groundShape2 = new Shape(
	new Polygon().setAsBox(25, .5).originalPoints,
);
const groundShape3 = new Shape(
	new Polygon().setAsBox(10.4, .5).originalPoints,
);
const groundShape4 = new Shape(
	new Polygon().setAsBox(10, .5).originalPoints,
);
const groundShape5 = new Shape(
	new Polygon().setAsBox(.5, 2.5).originalPoints,
);
const groundShape6 = groundShape4;
const groundShape7 = new Shape(
	new Polygon().setAsBox(.5, 5).originalPoints,
);
const groundMaterial = new VectorMaterial(
	[yellow, yellow, orange, yellow],
);

const bridgeShape = new Shape(
	new Polygon().setAsBox(1, .2).originalPoints,
);
const bridgeMaterial = new VectorMaterial(
	[cyan, cyan, cyan, cyan],
);

// variables
let body;
let motor;
let a = false;
let s = false;
let d = false;

const onKeyDown = (event) => {
	if (event.key === "a" || event.key === "A") {
		a = true;
	}
	if (event.key === "s" || event.key === "S") {
		s = true;
	}
	if (event.key === "d" || event.key === "D") {
		d = true;
	}
	if (s) {
		motor.setMotor(0, 20);
	} else if (d) {
		motor.setMotor(-50, 25);
	} else if (a) {
		motor.setMotor(10, 20);
	} else {
		motor.setMotor(0, 1);
	}
};

const onKeyUp = (event) => {
	if (event.key === "a" || event.key === "A") {
		a = false;
	}
	if (event.key === "s" || event.key === "S") {
		s = false;
	}
	if (event.key === "d" || event.key === "D") {
		d = false;
	}
	if (s) {
		motor.setMotor(0, 20);
	} else if (d) {
		motor.setMotor(-50, 25);
	} else if (a) {
		motor.setMotor(10, 20);
	} else {
		motor.setMotor(0, 1);
	}
};

// initialization method
function create({getSolver, createBody, createCrate, createBall}) {
	const solver = getSolver();
	{	// create the car
		const frame = createBody({
			position: new Vector2D(0, 2),
			shapes: [new Polygon().set(carVerts)],
			filterGroup: 3,
			exclusionList: [3],
		}, carShape, carMaterial);
		const wheel1 = createBall(-1.4, 1.1, {
			filterGroup: 3,
			exclusionList: [3],
		});
		const wheel2 = createBall(1.3, 1.1, {
			filterGroup: 3,
			exclusionList: [3],
		});

		const joint1 = new WheelJoint({
			bodyA: frame,
			bodyB: wheel1,
			anchorA: new Vector2D(-1.4, -.9),
			anchorB: new Vector2D(0, 0),
			axis: new Vector2D(0, 1),
			frequency: 4,
			damping: .7,
		});
		const joint2 = new WheelJoint({
			bodyA: frame,
			bodyB: wheel2,
			anchorA: new Vector2D(1.3, -.9),
			anchorB: new Vector2D(0, 0),
			axis: new Vector2D(0, 1),
			frequency: 4,
			damping: .7,
		});

		solver.addJoint(joint1);
		solver.addJoint(joint2);

		body = frame;
		motor = joint1;
	}
	{	// create some terrain
		const ground1 = createBody({
			position: new Vector2D(40, 0),
			shapes: [new Polygon().setAsBox(50, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape1, groundMaterial);
		const ground2 = createBody({
			position: new Vector2D(135, 0),
			shapes: [new Polygon().setAsBox(25, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape2, groundMaterial);
		createBody({
			position: new Vector2D(170, 2),
			angle: Math.PI / 16,
			shapes: [new Polygon().setAsBox(10.4, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape3, groundMaterial);
		createBody({
			position: new Vector2D(190, 4),
			shapes: [new Polygon().setAsBox(10, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape4, groundMaterial);
		createBody({
			position: new Vector2D(200, 1.5),
			shapes: [new Polygon().setAsBox(.5, 2.5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape5, groundMaterial);
		createBody({
			position: new Vector2D(210, -1),
			shapes: [new Polygon().setAsBox(10, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape6, groundMaterial);
		createBody({
			position: new Vector2D(220, 4),
			shapes: [new Polygon().setAsBox(.5, 5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		}, groundShape7, groundMaterial);

		const position = new Vector2D(91, .3);
		const firstBox = createBody({
			position,
			shapes: [new Polygon().setAsBox(1, .2)],
			filterGroup: 2,
			exclusionList: [2],
		}, bridgeShape, bridgeMaterial);

		position.x += 1;
		let lastBox = firstBox;
		for (let i = 0; i < 9; i++) {
			const box = createBody({
				position,
				shapes: [new Polygon().setAsBox(1, .2)],
				filterGroup: 2,
				exclusionList: [2],
			}, bridgeShape, bridgeMaterial);

			position.x += 2;
			const joint = new RevJoint({
				bodyA: lastBox,
				bodyB: box,
				anchorA: new Vector2D(1, 0),
				anchorB: new Vector2D(-1, 0),
			});

			solver.addJoint(joint);
			lastBox = box;
		}

		for (let i = 0; i < 10; i++) {
			createCrate(190, 4 + i);
		}

		for (let i = 0; i < 100; i++) {
			createBall(210 + (i % 4) / 8, -6 + i);
		}

		const firstJoint = new RevJoint({
			bodyA: ground1,
			bodyB: firstBox,
			anchorA: new Vector2D(50, .3),
			anchorB: new Vector2D(-1, 0),
		});

		const lastJoint = new RevJoint({
			bodyA: lastBox,
			bodyB: ground2,
			anchorA: new Vector2D(1, 0),
			anchorB: new Vector2D(-25, .3),
		});
		solver.addJoint(firstJoint);
		solver.addJoint(lastJoint);
	}
	registerCallbacks();
}

// register some callbacks
function registerCallbacks() {
	motor.setMotor(0, 1);

	a = false;
	s = false;
	d = false;

	window.addEventListener("keydown", onKeyDown);
	window.addEventListener("keyup", onKeyUp);
}

// on step method
function step({prevCam, curCam}) {
	body.setAsleep(false);
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
