const {
	rgba,
	builtIn: {
		Shape,
		VectorMaterial,
	},
} = require("2d-gl");
const {
	fork,
	Math: {Vector2D},
	Shapes: {Polygon},
	Joints: {RevJoint},
} = require("../../src/box");

const yellow = rgba(1, 1, 0, 1);

// shapes/materials
const groundShape = new Shape(
	new Polygon().setAsBox(10, .5).originalPoints,
);
const groundMaterial = new VectorMaterial(
	[yellow, yellow, yellow, yellow],
);

// variables
let restorePoint;
let getSolver;
let setSolver;

const forkAndCopyRenderables = (source) => {
	const target = fork(source);
	for (const body of source.bodies) {
		for (const shape of body.shapes) {
			target.shapeMap[shape.id].renderable = shape.renderable;
		}
	}
	return target;
};

const onKeyDown = (event) => {
	if (event.key === "f" || event.key === "F") {
		restorePoint = forkAndCopyRenderables(getSolver());
	} else if (event.key === "r" || event.key === "R") {
		if (restorePoint != null) {
			setSolver(forkAndCopyRenderables(restorePoint));
		}
	}
};

// initialization method
function create({getSolver, setSolver, createBody, createCrate, createBall}) {
	const solver = getSolver();
	{	// create some objects
		const box = createCrate(0, 5);

		const box2 = createCrate(1, 6);

		const joint = new RevJoint({
			bodyA: box,
			bodyB: box2,
			anchorA: new Vector2D(.5, .5),
			anchorB: new Vector2D(-.5, -.5),
			lowerLimit: -Math.PI / 8,
			upperLimit: Math.PI / 8,
		});

		createBall(0, 10);

		createBody({
			position: new Vector2D(0, 0),
			shapes: [new Polygon().setAsBox(10, .5)],
			static: true,
		}, groundShape, groundMaterial);

		solver.addJoint(joint);
	}
	registerCallbacks(getSolver, setSolver);
}

// register some callbacks
function registerCallbacks(getS, setS) {
	getSolver = getS;
	setSolver = setS;
	window.addEventListener("keydown", onKeyDown);
}

// clean up the test
function cleanUp() {
	window.removeEventListener("keydown", onKeyDown);
}

module.exports = {
	create,
	step: () => {},
	cleanUp,
};
