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
} = require("../../src/box");

const yellow = rgba(1, 1, 0, 1);
const red = rgba(1, .1, .1, 1);

// shapes/materials
const groundShape = new Shape(
	new Polygon().setAsBox(10, .5).originalPoints,
);
const groundMaterial = new VectorMaterial(
	[yellow, yellow, yellow, yellow],
);

const particleShape = new Shape(
	new Polygon().setAsBox(.1, .1).originalPoints,
);
const particleMaterial = new VectorMaterial(
	[red, red, red, red],
);

// initialization method
function create({createBody}) {
	createBody({
		position: new Vector2D(0, 0),
		shapes: [new Polygon().setAsBox(10, .5)],
		static: true,
	}, groundShape, groundMaterial);
}

const particles = [];
module.exports = {
	create,
	step: ({solver, scene, createBody}) => {
		particles.push(createBody({
			shapes: [new Circle(.1)],
			position: new Vector2D(0, 5),
			velocity: new Vector2D(
				Math.random() * 16 - 8,
				Math.random() * 5,
			),
			restitution: .5,
			particle: true,
			filterGroup: 32,
			exclusionList: [32],
		}, particleShape, particleMaterial));

		if (particles.length > 100) {
			const removed = particles.shift();
			solver.removeBody(removed);
			scene.delete(removed.shapes[0].renderable);
		}
	},
	cleanUp: () => {
		particles.splice(0, Infinity);
	},
};
