const {
	rgba,
	builtIn: {
		Shape,
		VectorMaterial,
	},
} = require("2d-gl");
const {
	Math: {Vector2D},
	Body,
	Shapes: {Polygon},
} = require("../../src/box");

const yellow = rgba(1, 1, 0, 1);

const groundShape = new Shape(
	new Polygon().setAsBox(50, .5).originalPoints,
);

const groundMaterial = new VectorMaterial(
	[yellow, yellow, yellow, yellow],
);

function create({getSolver, createBody, createCrate}) {
	console.log("measuring perf...");
	const solver = getSolver();
	createBody({
		position: new Vector2D(0, 0),
		shapes: [new Polygon().setAsBox(50, .5)],
		static: true,
	}, groundShape, groundMaterial);

	for (let i = 0; i < 10; i++) {
		for (let j = 0; j < 10; j++) {
			createCrate(-5 + i, 1 + j);
		}
	}

	const start = performance.now();
	for (let i = 0; i < 1000; i++) {
		solver.solve(1 / 45);
	}

	console.log("1000 steps in " + (performance.now() - start) + "ms");
}

module.exports = {
	create,
	step() {},
	cleanUp() {},
};
