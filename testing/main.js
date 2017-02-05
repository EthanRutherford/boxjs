//# preload ../src/collision/broadphase.js
//# preload ../src/collision/collision.js
//# preload ../src/collision/manifold.js
//# preload ../src/framework/math.js
//# preload ../src/framework/solver.js
//# preload ../src/joints/joint.js
//# preload ../src/joints/revjoint.js
//# preload ../src/joints/ropejoint.js
//# preload ../src/joints/springjoint.js
//# preload ../src/objects/body.js
//# preload ../src/objects/mass.js
//# preload ../src/objects/polygon.js
//# preload ../src/objects/circle.js
//# preload ../src/objects/shape.js
//# preload ./gl-matrix.min.js
//# preload ./render.js

const {initGL, SimpleRenderable, setOrtho, viewportToWorld} = require("./render.js");
const {Vector2D} = require("../src/framework/math.js");
const Body = require("../src/objects/body.js");
const RevJoint = require("../src/joints/revjoint.js");
const RopeJoint = require("../src/joints/ropejoint.js");
const SpringJoint = require("../src/joints/springjoint.js");
const Polygon = require("../src/objects/polygon.js");
const Circle = require("../src/objects/circle.js");
const Solver = require("../src/framework/solver.js");

const gl = initGL(document.getElementById("canvas"), 20);

function generateCircle(radius, count) {
	let points = [];
	let sector = 2 * Math.PI / count;
	for (let i = 0; i < count; i++) {
		let angle = sector * i;
		points.push(new Vector2D(Math.cos(angle) * radius, Math.sin(angle) * radius));
	}
	return points;
}

function generateCircleColors(count) {
	let colors = [];
	for (let i = 0; i < count; i++)
		colors.push(i / 40 + .5, i / 40 + .5, i / 40 + .5, 1);
	return colors;
}

function serializePoints(array) {
	return array.reduce((a, p) => {a.push(p.x, p.y); return a;}, []);
}

let solver = new Solver();
const gravityAcceleration = new Vector2D(0, -9.8);
solver.applyG = (bodies) => {
	for (let body of bodies)
		body.applyForce(gravityAcceleration.times(body.mass.m));
};

let renderables = [];

function render() {
	gl.clear(gl.COLOR_BUFFER_BIT);
	setOrtho(0, 0, 1);
	for (let body of renderables)
		body.renderable.render(body.position, body.transform.radians);
}

function step() {
	solver.solve(1 / 60);
	render();
}

function createBasicTest() {
	let box = new Body({
		position: new Vector2D(0, 5),
		shapes: [new Polygon().setAsBox(.5, .5)],
	});

	let box2 = new Body({
		position: new Vector2D(1, 6),
		shapes: [new Polygon().setAsBox(.5, .5)],
	});

	let joint = new RevJoint({
		bodyA: box,
		bodyB: box2,
		anchorA: new Vector2D(.5, .5),
		anchorB: new Vector2D(-.5, -.5),
		lowerLimit: -Math.PI / 8,
		upperLimit: Math.PI / 8,
	});

	let ball = new Body({
		position: new Vector2D(0, 10),
		shapes: [new Circle(.5)],
	});

	let ground = new Body({
		position: new Vector2D(0, 0),
		shapes: [new Polygon().setAsBox(10, .5)],
		static: true,
	});

	solver.addBody(ground);
	solver.addBody(box);
	solver.addBody(box2);
	solver.addJoint(joint);
	solver.addBody(ball);

	box.renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points),
		[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
	);

	box2.renderable = new SimpleRenderable(
		serializePoints(box2.shapes[0].points),
		[1, 1, 0, 1, 0, 1, 0, 1, 0, 1, .5, 1, 0, .2, 1, 1]
	);

	ground.renderable = new SimpleRenderable(
		serializePoints(ground.shapes[0].points),
		[1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1]
	);

	ball.renderable = new SimpleRenderable(
		serializePoints(generateCircle(ball.shapes[0].radius, 20)),
		generateCircleColors(20)
	);

	renderables.push(box, box2, ground, ball);
}

function createRopeTest() {
	let position = new Vector2D(10, 5);
	let anchorPoint = new Body({
		position,
		shapes: [new Polygon().setAsBox(.5, .1)],
		static: true,
		filterGroup: 0,
	});
	solver.addBody(anchorPoint);
	position.x += .5;
	let lastBox = anchorPoint;
	for (let i = 0; i < 10; i++) {
		let box = new Body({
			position,
			shapes: [new Polygon().setAsBox(.5, .1)],
			filterGroup: 2,
			exclusionList: [2],
		});
		position.x += 1;
		solver.addBody(box);
		let joint = new RevJoint({
			bodyA: lastBox,
			bodyB: box,
			anchorA: new Vector2D(.5, 0),
			anchorB: new Vector2D(-.5, 0),
		});
		solver.addJoint(joint);
		box.renderable = new SimpleRenderable(
			serializePoints(box.shapes[0].points),
			[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
		);
		renderables.push(box);
		lastBox = box;
	}
	let superDense = new Body({
		position,
		shapes: [new Polygon().setAsBox(.5, .5)],
		density: 10,
		filterGroup: 2,
		exclusionList: [2],
	});
	solver.addBody(superDense);
	superDense.renderable = new SimpleRenderable(
		serializePoints(superDense.shapes[0].points),
		[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
	);
	let joint = new RevJoint({
		bodyA: lastBox,
		bodyB: superDense,
		anchorA: new Vector2D(.5, 0),
		anchorB: new Vector2D(-.5, 0),
	});
	solver.addJoint(joint);
	renderables.push(superDense);
	let rope = new RopeJoint({
		bodyA: anchorPoint,
		bodyB: superDense,
		anchorA: new Vector2D(.5, 0),
		anchorB: new Vector2D(-.5, 0),
		limit: 10,
	});
	solver.addJoint(rope);
}

function createSpringTest() {
	let box = new Body({
		position: new Vector2D(5, 1),
		shapes: [new Polygon().setAsBox(.5, .5)],
	});

	let box2 = new Body({
		position: new Vector2D(5, 4),
		shapes: [new Polygon().setAsBox(.5, .5)],
	});

	let joint = new SpringJoint({
		bodyA: box,
		bodyB: box2,
		anchorA: new Vector2D(),
		anchorB: new Vector2D(),
		length: 3,
		frequency: 1,
		damping: .1,
	});

	solver.addBody(box);
	solver.addBody(box2);
	solver.addJoint(joint);

	box.renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points),
		[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
	);

	box2.renderable = new SimpleRenderable(
		serializePoints(box2.shapes[0].points),
		[1, 1, 0, 1, 0, 1, 0, 1, 0, 1, .5, 1, 0, .2, 1, 1]
	);

	renderables.push(box, box2);
}

createBasicTest();
createRopeTest();
createSpringTest();
window.setInterval(step, 1 / 60 * 1000);
window.onclick = (event) => {
	let p = viewportToWorld({x: event.x, y: event.y});
	let box = new Body({
		position: new Vector2D(p.x, p.y),
		shapes: [new Polygon().setAsBox(.5, .5)],
	});
	solver.addBody(box);
	let colors = [];
	for (let i = 0; i < 4; i++)
		colors.push(Math.random(), Math.random(), Math.random(), 1);
	box.renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points), colors
	);
	renderables.push(box);
	console.log(solver.bodies.size);
};
