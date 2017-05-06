//# preload ../src/box.js
//# preload ./gl-matrix.min.js
//# preload ./render.js

const {
	initGL,
	SimpleRenderable,
	setOrtho,
	viewportToWorld,
	getBounds,
} = require("./render");
const {
	fork,
	Math: {Vector2D},
	Solver,
	Body,
	AABB,
	Shapes: {Polygon, Circle},
	Joints: {RevJoint, RopeJoint, SpringJoint, WheelJoint},
} = require("../src/box");

const gl = initGL(document.getElementById("canvas"), 20);

const camera = {
	position: new Vector2D(),
	prevPos: new Vector2D(),
	zoom: 1,
};

function generateCircle(radius, count) {
	const points = [];
	const sector = 2 * Math.PI / count;
	for (let i = 0; i < count; i++) {
		const angle = sector * i;
		points.push(new Vector2D(Math.cos(angle) * radius, Math.sin(angle) * radius));
	}
	return points;
}

function generateCircleColors(count) {
	const colors = [];
	for (let i = 0; i < count; i++) {
		colors.push(i / 40 + .5, i / 40 + .5, i / 40 + .5, 1);
	}

	return colors;
}

function serializePoints(array) {
	return array.reduce((a, p) => {a.push(p.x, p.y); return a;}, []);
}

let solver = new Solver();
const gravityAcceleration = new Vector2D(0, -9.8);
solver.applyG = (bodies) => {
	for (const body of bodies) {
		body.applyForce(gravityAcceleration.times(body.mass.m));
	}
};

const renderables = new Set();

const debugBoxRenderable = new SimpleRenderable(
	[0, 0, 0, 0, 0, 0, 0, 0],
	[1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1],
	gl.LINE_LOOP,
	0
);

const logicSteps = new Set();

function lerp(a, b, ratio) {
	return (a * (1 - ratio)) + (b * ratio);
}
function vlerp(a, b, ratio) {
	return new Vector2D(lerp(a.x, b.x, ratio), lerp(a.y, b.y, ratio));
}
function alerp(a, b, ratio) {
	const diff = Math.abs(a - b);
	if (diff > Math.PI) {
		if (a > b) {
			b += 2 * Math.PI;
		} else {
			a += 2 * Math.PI;
		}
	}
	return lerp(a, b, ratio);
}

function render(lerpRatio) {
	gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
	const camPos = vlerp(camera.prevPos, camera.position, lerpRatio);
	setOrtho(camPos.x, camPos.y, camera.zoom);

	const {x0, x1, y0, y1} = getBounds();

	solver.query(new AABB(x0, y0, x1, y1), (shape) => {
		const body = shape.body;
		if (shape.renderable) {
			const pos = vlerp(body.prevPos, body.position, lerpRatio);
			const angle = alerp(body.prevAngle, body.transform.radians, lerpRatio);
			shape.renderable.render(pos, angle);
		}
	});

	for (const item of renderables) {
		const pos = vlerp(item.prevPos, item.position, lerpRatio);
		const angle = alerp(item.prevAngle, item.radians, lerpRatio);
		item.renderable.render(pos, angle);
	}

	if (!window.debugDraw) {
		return;
	}

	const nodes = solver.debugGetNodes();

	for (const node of nodes) {
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
	const physTarget = 1 / 45;
	const maxSteps = 5;
	let timestamp;
	let acc = 0;

	function step(stamp) {
		window.requestAnimationFrame(step);
		acc += (stamp - timestamp) / 1000 || 0;
		timestamp = stamp;

		let steps = Math.floor(acc / physTarget);
		if (steps > 0) acc -= steps * physTarget;
		steps = Math.min(steps, maxSteps);

		for (let i = 0; i < steps; i++) {
			solver.solve(physTarget);
			logicSteps.forEach((step) => step());
		}

		render(acc / physTarget);
	}

	window.requestAnimationFrame(step);
}

const onCleanup = [];

function createBasicTest() {
	{	//ground, ball, joint with angle limits
		const box = new Body({
			position: new Vector2D(0, 5),
			shapes: [new Polygon().setAsBox(.5, .5)],
		});

		const box2 = new Body({
			position: new Vector2D(1, 6),
			shapes: [new Polygon().setAsBox(.5, .5)],
		});

		const joint = new RevJoint({
			bodyA: box,
			bodyB: box2,
			anchorA: new Vector2D(.5, .5),
			anchorB: new Vector2D(-.5, -.5),
			lowerLimit: -Math.PI / 8,
			upperLimit: Math.PI / 8,
		});

		const ball = new Body({
			position: new Vector2D(0, 10),
			shapes: [new Circle(.5)],
		});

		const ground = new Body({
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
	{	//rope with dense mass on the bottom
		const position = new Vector2D(10, 5);
		const anchorPoint = new Body({
			position,
			shapes: [new Polygon().setAsBox(.5, .1)],
			static: true,
			filterGroup: 0,
		});

		solver.addBody(anchorPoint);
		position.x += .5;
		let lastBox = anchorPoint;
		for (let i = 0; i < 10; i++) {
			const box = new Body({
				position,
				shapes: [new Polygon().setAsBox(.5, .1)],
				filterGroup: 2,
				exclusionList: [2],
			});
			position.x += 1;
			solver.addBody(box);
			const joint = new RevJoint({
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

		const superDense = new Body({
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

		const joint = new RevJoint({
			bodyA: lastBox,
			bodyB: superDense,
			anchorA: new Vector2D(.5, 0),
			anchorB: new Vector2D(-.5, 0),
		});

		solver.addJoint(joint);
		const rope = new RopeJoint({
			bodyA: anchorPoint,
			bodyB: superDense,
			anchorA: new Vector2D(.5, 0),
			anchorB: new Vector2D(-.5, 0),
			limit: 10,
		});

		solver.addJoint(rope);
	}
	{	//spring joint test
		const box = new Body({
			position: new Vector2D(5, 1),
			shapes: [new Polygon().setAsBox(.5, .5)],
		});

		const box2 = new Body({
			position: new Vector2D(5, 4),
			shapes: [new Polygon().setAsBox(.5, .5)],
		});

		const joint = new SpringJoint({
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
}

function createCarTest() {
	let body;
	let motor;
	{	//create the car
		const verts = [
			new Vector2D(-2, -.8),
			new Vector2D(2, -.8),
			new Vector2D(2, -.2),
			new Vector2D(0, .8),
			new Vector2D(-1.5, .8),
			new Vector2D(-2, 0),
		];
		const frame = new Body({
			position: new Vector2D(0, 2),
			shapes: [new Polygon().set(verts)],
			filterGroup: 3,
			exclusionList: [3],
		});
		const wheel1 = new Body({
			position: new Vector2D(-1.4, 1.1),
			shapes: [new Circle(.5)],
			filterGroup: 3,
			exclusionList: [3],
		});
		const wheel2 = new Body({
			position: new Vector2D(1.3, 1.1),
			shapes: [new Circle(.5)],
			filterGroup: 3,
			exclusionList: [3],
		});

		const joint1 = new WheelJoint({
			bodyA: frame,
			bodyB: wheel1,
			anchorA: new Vector2D(-1.4, -.9),
			anchorB: new Vector2D(),
			axis: new Vector2D(0, 1),
			frequency: 4,
			damping: .7,
		});
		const joint2 = new WheelJoint({
			bodyA: frame,
			bodyB: wheel2,
			anchorA: new Vector2D(1.3, -.9),
			anchorB: new Vector2D(),
			axis: new Vector2D(0, 1),
			frequency: 4,
			damping: .7,
		});

		solver.addBody(frame);
		solver.addBody(wheel1);
		solver.addBody(wheel2);
		solver.addJoint(joint1);
		solver.addJoint(joint2);

		frame.shapes[0].renderable = new SimpleRenderable(
			serializePoints(frame.shapes[0].points),
			[0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1]
		);

		wheel1.shapes[0].renderable = new SimpleRenderable(
			serializePoints(generateCircle(wheel1.shapes[0].radius, 20)),
			generateCircleColors(20)
		);

		wheel2.shapes[0].renderable = new SimpleRenderable(
			serializePoints(generateCircle(wheel2.shapes[0].radius, 20)),
			generateCircleColors(20)
		);

		body = frame;
		motor = joint1;
	}
	{	//create some terrain
		const ground1 = new Body({
			position: new Vector2D(40, 0),
			shapes: [new Polygon().setAsBox(50, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});
		const ground2 = new Body({
			position: new Vector2D(135, 0),
			shapes: [new Polygon().setAsBox(25, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});
		const ground3 = new Body({
			position: new Vector2D(170, 2),
			angle: Math.PI / 16,
			shapes: [new Polygon().setAsBox(10.4, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});
		const ground4 = new Body({
			position: new Vector2D(190, 4),
			shapes: [new Polygon().setAsBox(10, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});
		const ground5 = new Body({
			position: new Vector2D(200, 1.5),
			shapes: [new Polygon().setAsBox(.5, 2.5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});
		const ground6 = new Body({
			position: new Vector2D(210, -1),
			shapes: [new Polygon().setAsBox(10, .5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});
		const ground7 = new Body({
			position: new Vector2D(220, 4),
			shapes: [new Polygon().setAsBox(.5, 5)],
			static: true,
			filterGroup: 2,
			exclusionList: [2],
		});

		const position = new Vector2D(91, .3);
		const firstBox = new Body({
			position,
			shapes: [new Polygon().setAsBox(1, .2)],
			filterGroup: 2,
			exclusionList: [2],
		});

		position.x += 1;
		let lastBox = firstBox;
		for (let i = 0; i < 9; i++) {
			const box = new Body({
				position,
				shapes: [new Polygon().setAsBox(1, .2)],
				filterGroup: 2,
				exclusionList: [2],
			});
			position.x += 2;
			solver.addBody(box);
			const joint = new RevJoint({
				bodyA: lastBox,
				bodyB: box,
				anchorA: new Vector2D(1, 0),
				anchorB: new Vector2D(-1, 0),
			});
			solver.addJoint(joint);
			box.shapes[0].renderable = new SimpleRenderable(
				serializePoints(box.shapes[0].points),
				[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
			);
			lastBox = box;
		}

		for (let i = 0; i < 10; i++) {
			const box = new Body({
				position: new Vector2D(190, 4 + i),
				shapes: [new Polygon().setAsBox(.5, .5)],
			});
			solver.addBody(box);
			box.shapes[0].renderable = new SimpleRenderable(
				serializePoints(box.shapes[0].points),
				[.2, .2, .1, 1, .2, .2, .1, 1, .3, .3, .2, 1, .3, .3, .2, 1]
			);
		}

		for (let i = 0; i < 100; i++) {
			const ball = new Body({
				position: new Vector2D(210 + (i % 4) / 8, -6 + i),
				shapes: [new Circle(.5)],
			});
			solver.addBody(ball);
			ball.shapes[0].renderable = new SimpleRenderable(
				serializePoints(generateCircle(ball.shapes[0].radius, 20)),
				generateCircleColors(20)
			);
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

		solver.addBody(ground1);
		solver.addBody(ground2);
		solver.addBody(ground3);
		solver.addBody(ground4);
		solver.addBody(ground5);
		solver.addBody(ground6);
		solver.addBody(ground7);
		solver.addBody(firstBox);
		solver.addJoint(firstJoint);
		solver.addJoint(lastJoint);

		ground1.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground1.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		ground2.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground2.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		ground3.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground3.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		ground4.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground4.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		ground5.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground5.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		ground6.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground6.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		ground7.shapes[0].renderable = new SimpleRenderable(
			serializePoints(ground7.shapes[0].points),
			[1, 1, 0, 1, 1, 1, 0, 1, 1, .5, 0, 1, 1, 0, 0, 1]
		);

		firstBox.shapes[0].renderable = new SimpleRenderable(
			serializePoints(firstBox.shapes[0].points),
			[0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
		);
	}
	{	//register some callbacks
		const logicStep = () => {
			camera.prevPos.set(camera.position);
			const error = body.position.minus(camera.position);
			const correction = error.times(.1);
			camera.position.add(correction);
		};
		logicSteps.add(logicStep);

		motor.setMotor(0, 1);

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

		window.addEventListener("keydown", onKeyDown);
		window.addEventListener("keyup", onKeyUp);

		onCleanup.push(() => {
			logicSteps.delete(logicStep);
			window.removeEventListener("keydown", onKeyDown);
			window.removeEventListener("keyup", onKeyUp);
		});
	}
}

function createRaycastTest() {
	{	//create some objects
		const points = [
			new Vector2D(Math.cos(0), Math.sin(0)),
			new Vector2D(Math.cos(2 * Math.PI / 3), Math.sin(2 * Math.PI / 3)),
			new Vector2D(Math.cos(-2 * Math.PI / 3), Math.sin(-2 * Math.PI / 3)),
		];

		const box = new Body({
			position: new Vector2D(4, 0),
			shapes: [new Polygon().setAsBox(.5, .5)],
			static: true,
		});

		const ball = new Body({
			position: new Vector2D(3, 2),
			shapes: [new Circle(.5)],
			static: true,
		});

		const triangle = new Body({
			position: new Vector2D(-2, -1),
			shapes: [new Polygon().set(points)],
			static: true,
		});

		const box2 = new Body({
			position: new Vector2D(-1.2, -2),
			shapes: [new Polygon().setAsBox(.5, .5)],
			static: true,
		});

		solver.addBody(box);
		solver.addBody(ball);
		solver.addBody(triangle);
		solver.addBody(box2);

		box.shapes[0].renderable = new SimpleRenderable(
			serializePoints(box.shapes[0].points),
			[0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
		);

		ball.shapes[0].renderable = new SimpleRenderable(
			serializePoints(generateCircle(ball.shapes[0].radius, 20)),
			generateCircleColors(20)
		);

		triangle.shapes[0].renderable = new SimpleRenderable(
			serializePoints(triangle.shapes[0].points),
			[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
		);

		box2.shapes[0].renderable = new SimpleRenderable(
			serializePoints(box2.shapes[0].points),
			[0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1]
		);
	}
	{	//register the raycast method
		const ray = {
			renderable: new SimpleRenderable(
				[0, 0, 0, 0, 0, 0, 0, 0],
				[1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1]
			),
			position: new Vector2D(),
			prevPos: new Vector2D(),
			radians: 0,
		};
		renderables.add(ray);

		const p1 = new Vector2D();
		let p2 = new Vector2D(50, 0);
		let angle = 0;
		const logicStep = () => {
			let fraction = 1;
			angle += .01;
			p2 = new Vector2D(50 * Math.cos(angle), 50 * Math.sin(angle));
			solver.raycast({p1, p2, callback: (data) => fraction = data.fraction});
			ray.renderable.updateBuffers({
				verts: [0, .01, 0, -.01, fraction * 50, -.01, fraction * 50, .01],
			});
			ray.prevAngle = ray.radians;
			ray.radians = angle;
		};
		logicSteps.add(logicStep);

		onCleanup.push(() => {
			ray.renderable.deleteBuffers();
			renderables.delete(ray);
			logicSteps.delete(logicStep);
		});
	}
}

function createForkTest() {
	{	//create some objects
		const box = new Body({
			position: new Vector2D(0, 5),
			shapes: [new Polygon().setAsBox(.5, .5)],
		});

		const box2 = new Body({
			position: new Vector2D(1, 6),
			shapes: [new Polygon().setAsBox(.5, .5)],
		});

		const joint = new RevJoint({
			bodyA: box,
			bodyB: box2,
			anchorA: new Vector2D(.5, .5),
			anchorB: new Vector2D(-.5, -.5),
			lowerLimit: -Math.PI / 8,
			upperLimit: Math.PI / 8,
		});

		const ball = new Body({
			position: new Vector2D(0, 10),
			shapes: [new Circle(.5)],
		});

		const ground = new Body({
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
	{	//register callbacks
		let restorePoint = null;
		const forkAndCopyRenderables = (source) => {
			const target = fork(source);
			for (const body of source.bodies) {
				for (const shape of body.shapes) {
					target.shapeMap.get(shape).renderable = shape.renderable;
				}
			}
			return target.solver;
		};

		const onKeyDown = (event) => {
			if (event.key === "f" || event.key === "F") {
				restorePoint = forkAndCopyRenderables(solver);
			} else if (event.key === "r" || event.key === "R") {
				if (restorePoint != null) {
					solver = forkAndCopyRenderables(restorePoint);
				}
			}
		};

		window.addEventListener("keydown", onKeyDown);

		onCleanup.push(() => {
			window.removeEventListener("keydown", onKeyDown);
		});
	}
}

function cleanupTests() {
	const bodies = solver.flush();
	for (const body of bodies) {
		for (const shape of body.shapes) {
			if (shape.renderable) {
				shape.renderable.deleteBuffers();
			}
		}
	}

	let callback;
	while ((callback = onCleanup.pop())) {
		callback();
	}

	camera.position.set({x: 0, y: 0});
	camera.prevPos.set({x: 0, y: 0});
	camera.zoom = 1;
}

createBasicTest();
startLoop();

//fun box throwing code
function startEvent(eventItem) {
	const origin = viewportToWorld({x: eventItem.clientX, y: eventItem.clientY});
	return {origin, endPoint: origin, v: new Vector2D(0, 0), index: null};
}

function moveEvent(data, eventItem) {
	data.endPoint = viewportToWorld({x: eventItem.clientX, y: eventItem.clientY});
	data.v = Vector2D.clone(data.endPoint).minus(data.origin);
	let length = data.v.length;
	data.v.mul(1 / length);
	length = Math.min(length, 10);
	data.v.mul(length);

	const width = .01 + length / 100;
	const verts = [0, -width, 0, width, length, 0];
	const angle = Math.atan2(data.v.y, data.v.x);
	const redness = length / 10;
	const colors = [0, 0, .5, 1, 0, 0, .5, 1, redness, 1 - redness, 0, 1];
	if (data.arrow == null) {
		data.arrow = {
			renderable: new SimpleRenderable(verts, colors),
			position: data.origin,
			prevPos: data.origin,
			radians: angle,
		};
		renderables.add(data.arrow);
	} else {
		data.arrow.renderable.updateBuffers({verts, colors});
		data.arrow.radians = angle;
		data.arrow.prevAngle = angle;
	}
}

function removeRenderable(data) {
	if (data.arrow != null) {
		data.arrow.renderable.deleteBuffers();
		renderables.delete(data.arrow);
	}
}

function endEvent(data) {
	const box = new Body({
		position: new Vector2D(data.origin.x, data.origin.y),
		shapes: [new Polygon().setAsBox(.5, .5)],
		velocity: data.v.times(5),
	});
	solver.addBody(box);
	const colors = [];
	for (let i = 0; i < 4; i++) {
		colors.push(Math.random(), Math.random(), Math.random(), 1);
	}

	box.shapes[0].renderable = new SimpleRenderable(
		serializePoints(box.shapes[0].points), colors
	);
}

window.addEventListener("touchstart", (event) => {
	//prevent mousedown handler from firing
	event.preventDefault();
	//only track one touch
	if (event.touches.length !== 1) {
		return;
	}

	const touch = event.touches[0];
	const data = startEvent(touch);

	function touchMove(innerEvent) {
		const innerTouch = [...innerEvent.changedTouches].find((x) => x.identifier === touch.identifier);
		if (innerTouch) {
			moveEvent(data, innerTouch);
		}
	}

	function touchEnd(innerEvent) {
		const innerTouch = [...innerEvent.changedTouches].find((x) => x.identifier === touch.identifier);
		if (!innerTouch) {
			return;
		}

		removeRenderable(data);
		endEvent(data);

		window.removeEventListener("touchmove", touchMove);
		window.removeEventListener("touchend", touchEnd);
		window.removeEventListener("touchcancel", touchCancel);
	}

	function touchCancel(innerEvent) {
		const innerTouch = [...innerEvent.changedTouches].find((x) => x.identifier === touch.identifier);
		if (!innerTouch) {
			return;
		}

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
	if (event.button !== 0) {
		return;
	}

	const data = startEvent(event);

	function mouseMove(innerEvent) {
		moveEvent(data, innerEvent);
	}

	function mouseUp(innerEvent) {
		if (innerEvent.button !== 0) {
			return;
		}

		removeRenderable(data);
		endEvent(data);

		window.removeEventListener("mousemove", mouseMove);
		window.removeEventListener("mouseup", mouseUp);
	}

	window.addEventListener("mousemove", mouseMove);
	window.addEventListener("mouseup", mouseUp);
});

//use number keys (for now) to switch tests
window.addEventListener("keydown", (event) => {
	if (event.key === "1") {
		cleanupTests();
		createBasicTest();
	} else if (event.key === "2") {
		cleanupTests();
		createCarTest();
	} else if (event.key === "3") {
		cleanupTests();
		createRaycastTest();
	} else if (event.key === "4") {
		cleanupTests();
		createForkTest();
	}
});

//external debug flags
window.debugDraw = false;
