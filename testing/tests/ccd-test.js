const {rgba, builtIn: {Shape, VectorMaterial}} = require("2d-gl");
const {Math: {Vector2D}, Shapes: {Polygon}} = require("../../src/box");

const blue = rgba(0, 0, 1, 1);
const square = new Shape(new Polygon().setAsBox(.1, 5).originalPoints);
const squareMaterial = new VectorMaterial([blue, blue, blue, blue]);

let count = 0;
let toi = false;
const onKeyDown = (event) => {
	if (event.key.toLowerCase() === "t") {
		toi = !toi;
	}
};

function create({createBody}) {
	createBody({
		position: new Vector2D(0, 0),
		shapes: [new Polygon().setAsBox(.1, 5)],
		static: true,
	}, square, squareMaterial);

	count = 0;
	toi = false;
	window.addEventListener("keydown", onKeyDown);
}

function step({createCrate}) {
	if (count++ === 100) {
		createCrate(-10, 0, {
			velocity: new Vector2D(100, Math.random() * 2 - 1),
			toi,
		});

		count = 0;
	}
}

function cleanUp() {
	window.removeEventListener("keydown", onKeyDown);
}

module.exports = {
	create,
	step,
	cleanUp,
};
