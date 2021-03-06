const {
	Renderer,
	Scene,
	ImageLoader,
	rgba,
	builtIn: {
		Shape,
		OrthoCamera,
		VectorMaterial,
		SpriteMaterial,
	},
	shaders: {
		MotionBlur,
	},
} = require("2d-gl");
const {
	Math: {Vector2D, Rotation},
	Solver,
	Body,
	AABB,
	Shapes: {Polygon, Circle},
} = require("../src/box");
const basicTest = require("./tests/basic-test");
const carTest = require("./tests/car-test");
const raycastTest = require("./tests/raycast-test");
const forkTest = require("./tests/fork-test");
const perfTest = require("./tests/perf-test");
const particleTest = require("./tests/particle-test");
const selfRightingTest = require("./tests/self-righting-test");
const sensorTest = require("./tests/sensor-test");
const ccdTest = require("./tests/ccd-test");
const {generateCirclePoints, generateCircleColors} = require("./utils");

// initialize solver
let solver = new Solver();
const gravityAcceleration = new Vector2D(0, -9.8);
solver.applyG = (bodies) => {
	for (const body of bodies) {
		body.applyForce(gravityAcceleration.times(body.mass.m));
	}
};

// initialize renderer
const canvas = document.querySelector("canvas");
const renderer = new Renderer(canvas);

// set up scene and camera
const renderObjects = new Set();
const getVisibleFunc = ({x0, y0, x1, y1}) => {
	const visible = [];

	for (const item of renderObjects) {
		if (scene.has(item.renderable)) {
			visible.push(item.renderable);
		}
	}

	solver.query(new AABB(x0, y0, x1, y1), (shape) => {
		if (scene.has(shape.renderable)) {
			visible.push(shape.renderable);
		}
	});

	if (window.debugDraw) {
		const nodes = solver.debugGetNodes();

		for (const node of nodes) {
			const points = [
				{x: node.aabb.min.x, y: node.aabb.min.y},
				{x: node.aabb.min.x, y: node.aabb.max.y},
				{x: node.aabb.max.x, y: node.aabb.max.y},
				{x: node.aabb.max.x, y: node.aabb.min.y},
			];

			if (node.shape.debug == null) {
				node.shape.debug = {shape: new Shape(points, Shape.lineLoop)};
				node.shape.debug.renderable = renderer.getInstance(
					node.shape.debug.shape,
					debugMaterial,
				);
				node.shape.debug.renderable.zIndex = 10;
			} else {
				node.shape.debug.shape.update(points);
			}

			visible.push(node.shape.debug.renderable);
		}
	}

	return visible;
};

const scene = new Scene({bgColor: rgba(.1, .1, .1, 1), getVisibleFunc});
scene.addPostProcShader(renderer.createShader(MotionBlur));
const prevCam = new Vector2D(0, 0);
const curCam = new Vector2D(0, 0);
const camera = new OrthoCamera(0, 0, 20);

// initialize global shapes/materials
const red = rgba(1, 0, 0, 1);

const crateShape = new Shape(
	new Polygon().setAsBox(.5, .5).originalPoints,
);
let crateMaterial;

const ballShape = new Shape(
	generateCirclePoints(.5, 20),
);
const ballMaterial = new VectorMaterial(
	generateCircleColors(20),
);

const loader = new ImageLoader();
loader.get("crate", "/boxjs/testing/images/crate.png").then((result) => {
	crateMaterial = new SpriteMaterial(
		[{x: 0, y: 1}, {x: 0, y: 0}, {x: 1, y: 0}, {x: 1, y: 1}],
		result,
	);
});

const debugMaterial = new VectorMaterial([red, red, red, red]);

// rendering linear interpolation helpers
function render(lerpRatio) {
	const camPos = prevCam.lerp(curCam, lerpRatio);
	camera.set(camPos);

	for (const item of renderObjects) {
		const pos = item.prevPos.lerp(item.position, lerpRatio);
		const angle = Rotation.lerp(item.prevAngle, item.angle, lerpRatio);

		item.renderable.x = pos.x;
		item.renderable.y = pos.y;
		item.renderable.r = angle;
	}

	for (const body of solver.bodies) {
		const pos = body.originalPrevPos.lerp(body.originalPosition, lerpRatio);
		const angle = Rotation.lerp(body.prevTrans.radians, body.transform.radians, lerpRatio);
		for (const shape of body.shapes) {
			if (shape.renderable) {
				shape.renderable.x = pos.x;
				shape.renderable.y = pos.y;
				shape.renderable.r = angle;
			}
		}
	}

	renderer.render(camera, scene);
}

let postStep = null;
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
			curTest.step({
				dt: physTarget,
				solver,
				scene,
				prevCam,
				curCam,
				createBody,
				createCrate,
				createBall,
				createRenderObj,
			});
			if (postStep) postStep();
		}

		render(window.debugDraw ? 1 : acc / physTarget);
	}

	window.requestAnimationFrame(step);
}

// create object helpers
function createBody(params, shape, material) {
	const renderable = renderer.getInstance(shape, material);
	const body = new Body(params);

	solver.addBody(body);
	body.shapes[0].renderable = renderable;
	scene.add(renderable);

	return body;
}

function createCrate(x, y, params = {}) {
	return createBody(Object.assign(params, {
		position: new Vector2D(x, y),
		shapes: [new Polygon().setAsBox(.5, .5)],
	}), crateShape, crateMaterial);
}

function createBall(x, y, params = {}) {
	return createBody(Object.assign(params, {
		position: new Vector2D(x, y),
		shapes: [new Circle(.5)],
	}), ballShape, ballMaterial);
}

function createRenderObj(shape, material, x = 0, y = 0, r = 0) {
	const renderable = renderer.getInstance(shape, material);

	const renderObject = {
		shape,
		material,
		renderable,
		prevPos: new Vector2D(x, y),
		position: new Vector2D(x, y),
		prevAngle: r,
		angle: r,
	};

	scene.add(renderable);
	renderObjects.add(renderObject);

	return renderObject;
}

function removeRenderObj(item) {
	scene.delete(item.renderable);
	renderObjects.delete(item);
}

// tests
let curTest;

const testParams = {
	getSolver() {return solver;},
	setSolver(value) {solver = value;},
	createBody,
	createCrate,
	createBall,
	createRenderObj,
};

function cleanUpTests() {
	for (const body of solver.bodies) {
		for (const shape of body.shapes) {
			scene.delete(shape.renderable);
		}
	}

	solver.flush();
	curTest.cleanUp({removeRenderObj});

	curCam.set({x: 0, y: 0});
	prevCam.set({x: 0, y: 0});
}

// wait till the images are loaded, then load basic test
loader.all().then(() => {
	basicTest.create(testParams);
	curTest = basicTest;
	startLoop();
});

// test switching
function setTest(test) {
	cleanUpTests();
	test.create(testParams);
	curTest = test;
}

// fun box throwing code
function startEvent(eventItem) {
	const origin = renderer.viewportToWorld(
		eventItem.clientX,
		eventItem.clientY,
		camera,
	);
	return {origin, endPoint: origin, v: new Vector2D(0, 0), index: null};
}

function moveEvent(data, eventItem) {
	data.endPoint = renderer.viewportToWorld(
		eventItem.clientX,
		eventItem.clientY,
		camera,
	);
	data.v = Vector2D.clone(data.endPoint).sub(data.origin);
	let length = data.v.length;
	if (length > 10) {
		data.v.mul(10 / length);
		length = 10;
	}

	const width = .01 + length / 100;
	const verts = [
		{x: 0, y: -width},
		{x: 0, y: width},
		{x: length, y: 0},
	];
	const angle = Math.atan2(data.v.y, data.v.x);
	const redness = length / 10;
	const colors = [
		rgba(0, 0, .5, 1),
		rgba(0, 0, .5, 1),
		rgba(redness, 1 - redness, 0, 1),
	];
	if (data.arrow == null) {
		data.arrow = createRenderObj(
			new Shape(verts),
			new VectorMaterial(colors),
			data.origin.x,
			data.origin.y,
			angle,
		);
	} else {
		data.arrow.shape.update(verts);
		data.arrow.material.update(colors);
		data.arrow.prevAngle = angle;
		data.arrow.angle = angle;
	}
}

function endEvent(data) {
	const box = new Body({
		position: new Vector2D(data.origin.x, data.origin.y),
		shapes: [new Polygon().setAsBox(.5, .5)],
		velocity: data.v.times(5),
		toi: true,
	});
	solver.addBody(box);

	box.shapes[0].renderable = renderer.getInstance(crateShape, crateMaterial);
	scene.add(box.shapes[0].renderable);
}

// body grabbing code
function grabEvent(startEvent) {
	let position = Vector2D.clone(renderer.viewportToWorld(
		startEvent.clientX,
		startEvent.clientY,
		camera,
	));

	let body = null;
	solver.query(new AABB(position.x, position.y, position.x, position.y), (shape) => {
		body = shape.body;
	});

	if (body == null) {
		return;
	}

	function mouseMove(event) {
		position = Vector2D.clone(renderer.viewportToWorld(
			event.clientX,
			event.clientY,
			camera,
		));
	}

	function mouseUp() {
		postStep = null;
		window.removeEventListener("mousemove", mouseMove);
		window.removeEventListener("mouseup", mouseUp);
	}

	postStep = () => {
		// keep body awake
		body.setAsleep(false);

		// drive body toward mouse
		const toPos = position.minus(body.position);
		const distance = toPos.length;
		toPos.mul(1 / distance);

		const force = toPos.mul(Math.min(distance, 10) * body.mass.m * 100);
		body.applyForce(force);

		// dampen velocity
		body.velocity.mul(.8);
	};

	window.addEventListener("mousemove", mouseMove);
	window.addEventListener("mouseup", mouseUp);
}

// event handlers
canvas.addEventListener("touchstart", (event) => {
	// prevent mousedown handler from firing
	event.preventDefault();
	// only track one touch
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

		if (data.arrow != null) {
			removeRenderObj(data.arrow);
		}
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

		if (data.arrow != null) {
			removeRenderObj(data.arrow);
		}

		window.removeEventListener("touchmove", touchMove);
		window.removeEventListener("touchend", touchEnd);
		window.removeEventListener("touchcancel", touchCancel);
	}

	window.addEventListener("touchmove", touchMove);
	window.addEventListener("touchend", touchEnd);
	window.addEventListener("touchcancel", touchCancel);
}, {passive: false});

canvas.addEventListener("mousedown", (event) => {
	if (event.button !== 0) {
		return;
	}

	if (event.shiftKey) {
		grabEvent(event);
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

		if (data.arrow != null) {
			removeRenderObj(data.arrow);
		}
		endEvent(data);

		window.removeEventListener("mousemove", mouseMove);
		window.removeEventListener("mouseup", mouseUp);
	}

	window.addEventListener("mousemove", mouseMove);
	window.addEventListener("mouseup", mouseUp);
});

// use number keys (for now) to switch tests
window.addEventListener("keydown", (event) => {
	if (event.key === "1") {
		setTest(basicTest);
	} else if (event.key === "2") {
		setTest(carTest);
	} else if (event.key === "3") {
		setTest(raycastTest);
	} else if (event.key === "4") {
		setTest(forkTest);
	} else if (event.key === "5") {
		setTest(perfTest);
	} else if (event.key === "6") {
		setTest(particleTest);
	} else if (event.key === "7") {
		setTest(selfRightingTest);
	} else if (event.key === "8") {
		setTest(sensorTest);
	} else if (event.key === "9") {
		setTest(ccdTest);
	} else if (event.key === "0") {
		window.debugDraw = !window.debugDraw;
	}
});

// external debug stuff
window.debugDraw = false;
window.getSolver = () => solver;
