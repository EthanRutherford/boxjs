//# preload ../src/collision/broadphase.js
//# preload ../src/collision/collision.js
//# preload ../src/collision/manifold.js
//# preload ../src/framework/math.js
//# preload ../src/framework/solver.js
//# preload ../src/joints/joint.js
//# preload ../src/joints/revjoint.js
//# preload ../src/joints/ropejoint.js
//# preload ../src/joints/springjoint.js
//# preload ../src/joints/wheeljoint.js
//# preload ../src/objects/body.js
//# preload ../src/objects/mass.js
//# preload ../src/objects/polygon.js
//# preload ../src/objects/circle.js
//# preload ../src/objects/shape.js
const Math = require("./framework/math.js");
const Solver = require("./framework/solver.js");
const Joint = require("./joints/joint.js");
const RevJoint = require("./joints/revjoint.js");
const RopeJoint = require("./joints/ropejoint.js");
const SpringJoint = require("./joints/springjoint.js");
const WheelJoint = require("./joints/wheeljoint.js");
const Body = require("./objects/body.js");
const Circle = require("./objects/circle.js");
const Polygon = require("./objects/polygon.js");
const {Shape, AABB} = require("./objects/shape.js");

module.exports = {
	Math,
	Solver,
	Body,
	AABB,
	Shapes: {
		Shape,
		Circle,
		Polygon,
	},
	Joints: {
		Joint,
		RevJoint,
		RopeJoint,
		SpringJoint,
		WheelJoint,
	},
};
