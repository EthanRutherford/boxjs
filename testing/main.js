//# preload ../src/box.js
//# preload ./gl-matrix.min.js
//# preload ./render.js

const {
	initGL,
	SimpleRenderable,
	setOrtho,
	viewportToWorld,
	getBounds,
} = require("./render.js");
const {
	Math: {Vector2D},
	Solver,
	Body,
	AABB,
	Shapes: {Polygon, Circle},
	Joints: {RevJoint, RopeJoint, SpringJoint},

} = require("../src/box.js");

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

let debugBoxRenderable = new SimpleRenderable(
	[0, 0, 0, 0, 0, 0, 0, 0],
	[1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1],
	gl.LINE_LOOP
);

function render() {
	gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
	setOrtho(0, 0, 1);

	let {x0, x1, y0, y1} = getBounds();

	solver.query(new AABB(x0, y0, x1, y1), (shape) => {
		let body = shape.body;
		if (shape.renderable)
			shape.renderable.render(body.position, body.transform.radians);
	});

	for (let item of renderables)
		item.renderable.render(item.position, item.radians);

	if (!window.debugDraw)
		return;

	let nodes = solver.debugGetNodes();

	for (let node of nodes) {
		debugBoxRenderable.updateBuffers({
			verts: [
				node.min.x, node.min.y,
				node.min.x, node.max.y,
				node.max.x, node.max.y,
				node.max.x, node.min.y,
			],
		});
		debugBoxRenderable.render({x: 0, y: 0}, 0);
	}
}

function startLoop() {
	const minPhysicsElapsedTime = 1 / 30;
	let timestamp;

	(function step(stamp) {
		window.requestAnimationFrame(step);
		let elapsed = (stamp - timestamp) / 1000 || 0;
		timestamp = stamp;

		render();
		solver.solve(Math.min(elapsed, minPhysicsElapsedTime));
	})();
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

	box.shapes[0].renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points),
		[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
	);

	box2.shapes[0].renderable = new SimpleRenderable(
		serializePoints(box2.shapes[0].points),
		[1, 1, 0, 1, 0, 1, 0, 1, 0, 1, .5, 1, 0, .2, 1, 1]
	);

	ground.shapes[0].renderable = new SimpleRenderable(
		serializePoints(ground.shapes[0].points),
		[1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1]
	);

	ball.shapes[0].renderable = new SimpleRenderable(
		serializePoints(generateCircle(ball.shapes[0].radius, 20)),
		generateCircleColors(20)
	);
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
		box.shapes[0].renderable = new SimpleRenderable(
			serializePoints(box.shapes[0].points),
			[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
		);
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
	superDense.shapes[0].renderable = new SimpleRenderable(
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

	box.shapes[0].renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points),
		[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
	);

	box2.shapes[0].renderable = new SimpleRenderable(
		serializePoints(box2.shapes[0].points),
		[1, 1, 0, 1, 0, 1, 0, 1, 0, 1, .5, 1, 0, .2, 1, 1]
	);
}

createBasicTest();
createRopeTest();
createSpringTest();
startLoop();

function startEvent(eventItem) {
	let origin = viewportToWorld({x: eventItem.clientX, y: eventItem.clientY});
	return {origin, endPoint: origin, v: new Vector2D(0, 0), index: null};
}

function moveEvent(data, eventItem) {
	data.endPoint = viewportToWorld({x: eventItem.clientX, y: eventItem.clientY});
	data.v = Vector2D.clone(data.endPoint).minus(data.origin);
	let length = data.v.length;
	data.v.mul(1 / length);
	length = Math.min(length, 10);
	data.v.mul(length);

	let width = .01 + length / 100;
	let verts = [0, -width, 0, width, length, 0];
	let angle = Math.atan2(data.v.y, data.v.x);
	let redness = length / 10;
	let colors = [0, 0, .5, 1, 0, 0, .5, 1, redness, 1 - redness, 0, 1];
	if (data.index == null) {
		renderables.push({
			renderable: new SimpleRenderable(verts, colors),
			position: data.origin,
			radians: angle,
		});
		data.index = renderables.length - 1;
	} else {
		renderables[data.index].renderable.updateBuffers({verts, colors});
		renderables[data.index].radians = angle;
	}
}

function removeRenderable(data) {
	if (data.index != null)
		renderables.splice(data.index, 1)[0].renderable.deleteBuffers();
}

function endEvent(data) {
	let box = new Body({
		position: new Vector2D(data.origin.x, data.origin.y),
		shapes: [new Polygon().setAsBox(.5, .5)],
		velocity: data.v.times(5),
	});
	solver.addBody(box);
	let colors = [];
	for (let i = 0; i < 4; i++)
		colors.push(Math.random(), Math.random(), Math.random(), 1);
	box.shapes[0].renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points), colors
	);
}

window.addEventListener("touchstart", (event) => {
	//prevent mousedown handler from firing
	event.preventDefault();
	//only track one touch
	if (event.touches.length !== 1)
		return;

	let touch = event.touches[0];
	let data = startEvent(touch);

	function touchMove(innerEvent) {
		let innerTouch = [...innerEvent.changedTouches].find((x) => x.identifier === touch.identifier);
		if (innerTouch)
			moveEvent(data, innerTouch);
	}

	function touchEnd(innerEvent) {
		let innerTouch = [...innerEvent.changedTouches].find((x) => x.identifier === touch.identifier);
		if (!innerTouch)
			return;

		removeRenderable(data);
		endEvent(data);

		window.removeEventListener("touchmove", touchMove);
		window.removeEventListener("touchend", touchEnd);
		window.removeEventListener("touchcancel", touchCancel);
	}

	function touchCancel() {
		let innerTouch = [...innerEvent.changedTouches].find((x) => x.identifier === touch.identifier);
		if (!innerTouch)
			return;

		removeRenderable(data);

		window.removeEventListener("touchmove", touchMove);
		window.removeEventListener("touchend", touchEnd);
		window.removeEventListener("touchcancel", touchCancel);
	}

	window.addEventListener("touchmove", touchMove);
	window.addEventListener("touchend", touchEnd);
	window.addEventListener("touchcancel", touchCancel);
}, {passive: false});

window.addEventListener("mousedown", (event) => {
	if (event.button !== 0)
		return;

	let data = startEvent(event);

	function mouseMove(innerEvent) {
		moveEvent(data, innerEvent);
	}

	function mouseUp(innerEvent) {
		if (innerEvent.button !== 0)
			return;

		removeRenderable(data);
		endEvent(data);

		window.removeEventListener("mousemove", mouseMove);
		window.removeEventListener("mouseup", mouseUp);
	}

	window.addEventListener("mousemove", mouseMove);
	window.addEventListener("mouseup", mouseUp);
});

//external debug flags
window.debugDraw = false;
