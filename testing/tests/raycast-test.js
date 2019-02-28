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
} = require("../../src/box");

const triangleVerts = [
	new Vector2D(Math.cos(0), Math.sin(0)),
	new Vector2D(Math.cos(2 * Math.PI / 3), Math.sin(2 * Math.PI / 3)),
	new Vector2D(Math.cos(-2 * Math.PI / 3), Math.sin(-2 * Math.PI / 3)),
];

const red = rgba(1, 0, 0, 1);
const green = rgba(0, 1, 0, 1);
const white = rgba(1, 1, 1, 1);

// shapes/materials
const boxShape = new Shape(
	new Polygon().setAsBox(.5, .5).originalPoints,
);
const boxMaterial = new VectorMaterial(
	[green, green, green, green],
);

const triangleShape = new Shape(triangleVerts);
const triangleMaterial = new VectorMaterial(
	[white, white, white, white],
);

const rayMaterial = new VectorMaterial(
	[red, red],
	VectorMaterial.lineStrip,
);

// variables
let ray;
const p1 = new Vector2D(0, 0);
let p2 = new Vector2D(50, 0);
let angle = 0;

// initialization method
function create({createBody, createBall, createRenderObj}) {
	createBody({
		position: new Vector2D(4, 0),
		shapes: [new Polygon().setAsBox(.5, .5)],
		static: true,
	}, boxShape, boxMaterial);

	createBall(3, 2, {static: true});

	createBody({
		position: new Vector2D(-2, -1),
		shapes: [new Polygon().set(triangleVerts)],
		static: true,
	}, triangleShape, triangleMaterial);

	createBody({
		position: new Vector2D(-1.2, -2),
		shapes: [new Polygon().setAsBox(.5, .5)],
		static: true,
	}, boxShape, boxMaterial);

	initRay(createRenderObj);
}

// create the ray
function initRay(createRenderObj) {
	ray = createRenderObj(
		new Shape([{x: 0, y: 0}, {x: 0, y: 0}]),
		rayMaterial,
	);

	p2 = new Vector2D(50, 0);
	angle = 0;
}

function step({solver}) {
	let fraction = 1;
	angle += .01;
	p2 = new Vector2D(50 * Math.cos(angle), 50 * Math.sin(angle));
	solver.raycast({p1, p2, callback: (data) => fraction = data.fraction});

	ray.shape.update([{x: 0, y: 0}, {x: fraction * 50, y: 0}]);

	ray.prevAngle = ray.angle;
	ray.angle = angle;
}

function cleanUp({removeRenderObj}) {
	removeRenderObj(ray);
}

module.exports = {
	create,
	step,
	cleanUp,
};
