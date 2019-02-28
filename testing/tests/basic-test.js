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
	Joints: {RevJoint, RopeJoint, SpringJoint},
} = require("../../src/box");

const yellow = rgba(1, 1, 0, 1);
const cyan = rgba(0, 1, 1, 1);

// shapes/materials
const groundShape = new Shape(
	new Polygon().setAsBox(10, .5).originalPoints,
);
const groundMaterial = new VectorMaterial(
	[yellow, yellow, yellow, yellow],
);

const chainShape = new Shape(
	new Polygon().setAsBox(.275, .1).originalPoints,
);
const chainMaterial = new VectorMaterial(
	[cyan, cyan, cyan, cyan],
);

const denseShape = new Shape(
	new Polygon().setAsBox(.5, .5).originalPoints,
);
const denseMaterial = new VectorMaterial(
	[cyan, cyan, cyan, cyan],
);

// initialization method
function create({getSolver, createBody, createCrate, createBall}) {
	const solver = getSolver();
	{	// ground, ball, joint with angle limits
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
	{	// rope with dense mass on the bottom
		const position = new Vector2D(10, 5);
		const anchorPoint = new Body({
			position,
			shapes: [new Polygon().setAsBox(.25, .1)],
			static: true,
			filterGroup: 0,
		});

		position.x += .25;
		let lastBox = anchorPoint;
		for (let i = 0; i < 20; i++) {
			const box = createBody({
				position,
				shapes: [new Polygon().setAsBox(.275, .1)],
				filterGroup: 2,
				exclusionList: [2],
			}, chainShape, chainMaterial);

			position.x += .5;
			const joint = new RevJoint({
				bodyA: lastBox,
				bodyB: box,
				anchorA: new Vector2D(.25, 0),
				anchorB: new Vector2D(-.25, 0),
			});
			solver.addJoint(joint);

			lastBox = box;
		}

		const superDense = createBody({
			position,
			shapes: [new Polygon().setAsBox(.5, .5)],
			density: 10,
			filterGroup: 2,
			exclusionList: [2],
		}, denseShape, denseMaterial);

		const joint = new RevJoint({
			bodyA: lastBox,
			bodyB: superDense,
			anchorA: new Vector2D(.25, 0),
			anchorB: new Vector2D(-.5, 0),
		});
		const rope = new RopeJoint({
			bodyA: anchorPoint,
			bodyB: superDense,
			anchorA: new Vector2D(.5, 0),
			anchorB: new Vector2D(-.5, 0),
			limit: 10.25,
		});

		solver.addJoint(joint);
		solver.addJoint(rope);
	}
	{	// spring joint test
		const box = createCrate(5, 1);

		const box2 = createCrate(5, 4);

		const joint = new SpringJoint({
			bodyA: box,
			bodyB: box2,
			anchorA: new Vector2D(0, 0),
			anchorB: new Vector2D(0, 0),
			length: 3,
			frequency: 1,
			damping: .1,
		});

		solver.addJoint(joint);
	}
}

module.exports = {
	create,
	step: () => {},
	cleanUp: () => {},
};
