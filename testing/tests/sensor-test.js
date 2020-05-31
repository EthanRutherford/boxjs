const {
	rgba,
	builtIn: {
		Shape,
		VectorMaterial,
	},
} = require("2d-gl");
const {
	Math: {Vector2D},
	Shapes: {Polygon, Circle},
	Joints: {RevJoint},
} = require("../../src/box");
const {generateCirclePoints} = require("../utils");

const red = rgba(1, 0, 0, 1);
const green = rgba(0, 1, 0, 1);
const blue = rgba(0, 0, 1, 1);
const yellow = rgba(1, 1, 0, 1);

// shapes/materials
const groundShape = new Shape(
	new Polygon().setAsBox(10, .5).originalPoints,
);
const groundMaterial = new VectorMaterial(
	[yellow, yellow, yellow, yellow],
);

const triVerts = [new Vector2D(0, 1), new Vector2D(1, 0), new Vector2D(0, -1)];
const triShape = new Shape(triVerts);
const triMaterial = new VectorMaterial([blue, blue, blue]);

const circleVerts = generateCirclePoints(4, 50);
const circleShape = new Shape(circleVerts, Shape.lineLoop);
const circleMaterial = new VectorMaterial(new Array(50).fill(red));

const rectVerts = [new Vector2D(0, 1), new Vector2D(3, 1), new Vector2D(3, -1), new Vector2D(0, -1)];
const rectShape = new Shape(rectVerts, Shape.lineLoop);
const rectMaterial = new VectorMaterial(new Array(4).fill(red));

// variables
let body = null;
let circleRen = null;
let rectRen = null;
const onKeyDown = (event) => {
	if (event.key.toLowerCase() === "w") {
		body.position.y += .1;
	} else if (event.key.toLowerCase() === "a") {
		body.position.x -= .1;
	} else if (event.key.toLowerCase() === "s") {
		body.position.y -= .1;
	} else if (event.key.toLowerCase() === "d") {
		body.position.x += .1;
	}
};

// initialization method
function create({getSolver, createBody, createCrate, createBall, createRenderObj}) {
	function onCollide(contactData) {
		if (contactData.shape === body.shapes[0]) {
			circleRen.contact = true;
		} else if (contactData.shape === body.shapes[1]) {
			rectRen.contact = true;
		}
	}

	const solver = getSolver();
	{	// create some objects
		const box = createCrate(0, 5);

		const box2 = createCrate(1, 6);

		solver.addJoint(new RevJoint({
			bodyA: box,
			bodyB: box2,
			anchorA: new Vector2D(.5, .5),
			anchorB: new Vector2D(-.5, -.5),
			lowerLimit: -Math.PI / 8,
			upperLimit: Math.PI / 8,
		}));

		createBall(-2, 10);

		createBody({
			position: new Vector2D(0, 0),
			shapes: [new Polygon().setAsBox(10, .5)],
			static: true,
		}, groundShape, groundMaterial);

		body = createBody({
			position: new Vector2D(0, 2),
			shapes: [
				new Circle(4, true),
				new Polygon(true).set(rectVerts),
			],
			static: true,
			onCollide,
		}, triShape, triMaterial);

		circleRen = createRenderObj(circleShape, circleMaterial);
		rectRen = createRenderObj(rectShape, rectMaterial);
	}
	registerCallbacks();
}

// register some callbacks
function registerCallbacks() {
	window.addEventListener("keydown", onKeyDown);
}

function step() {
	body.setAsleep(false);
	circleRen.prevPos.set(circleRen.position);
	rectRen.prevPos.set(rectRen.position);
	circleRen.position.set(body.position);
	rectRen.position.set(body.position).sub(body.mass.center);

	if (circleRen.contact) {
		circleRen.material.update(new Array(50).fill(green));
	} else {
		circleRen.material.update(new Array(50).fill(red));
	}

	if (rectRen.contact) {
		rectRen.material.update(new Array(4).fill(green));
	} else {
		rectRen.material.update(new Array(4).fill(red));
	}

	circleRen.contact = false;
	rectRen.contact = false;
}

// clean up the test
function cleanUp({removeRenderObj}) {
	window.removeEventListener("keydown", onKeyDown);
	removeRenderObj(circleRen);
	removeRenderObj(rectRen);
}


module.exports = {
	create,
	step,
	cleanUp,
};
