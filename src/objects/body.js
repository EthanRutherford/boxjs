const {Vector2D, Rotation, bigG} = require("../framework/math");
const MassData = require("./mass");

module.exports = class Body {
	constructor({
		position,
		angle,
		velocity,
		angularVelocity,
		shapes,
		friction,
		restitution,
		density,
		sensor,
		static: isStatic,
		filterGroup,
		exclusionList,
		onCollide,
	}) {
		this.position = Vector2D.clone(position);
		this.prevPos = Vector2D.clone(position);
		this.transform = new Rotation(angle || 0);
		this.velocity = Vector2D.clone(velocity);
		this.angularVelocity = angularVelocity || 0;
		this.force = new Vector2D(0, 0);
		this.torque = 0;
		this.shapes = shapes;
		this.friction = friction != null ? friction : .6;
		this.restitution = restitution != null ? restitution : .2;
		//set mass data
		density = density != null ? density : 1;
		this.mass = new MassData();
		for (let shape of this.shapes) {
			//set body of shape
			shape.body = this;
			//sum mass data
			let mass = shape.computeMass(density);
			this.mass.m += mass.m;
			this.mass.center.add(mass.center.times(mass.m));
			this.mass.i += mass.i;
		}
		//inverse values
		this.mass.iM = this.mass.m ? 1 / this.mass.m : 0;
		this.mass.iI = this.mass.i ? 1 / this.mass.i : 0;
		this.mass.center.mul(this.mass.iM);
		for (let shape of this.shapes) {
			//translate each shape
			if (shape.recenter instanceof Function) {
				shape.recenter(this.mass.center);
			}
			//initialize aabb
			shape.setAABB();
		}
		this.position.add(this.mass.center);
		//set to static?
		if (isStatic) {
			this.setStatic();
		}
		//set to sensor
		this.sensor = sensor || false;
		//set filter parameters
		this.setFilter(filterGroup != null ? filterGroup : 1, exclusionList || []);
		//set collision callback
		this.onCollide = onCollide;
	}
	applyForce(force) {
		this.force.add(force);
	}
	applyTorque(torque) {
		this.torque += torque;
	}
	applyGravity(mass, direction, distance) {
		let g = bigG * mass * this.mass.m / Math.sqr(distance);
		let f = direction.times(g);
		this.applyForce(f);
	}
	applyImpules(impulse, contact) {
		this.velocity.add(impulse.times(this.mass.iM));
		this.angularVelocity += this.mass.iI * contact.cross(impulse);
	}
	setStatic() {
		this.mass.m = 0;
		this.mass.iM = 0;
		this.mass.i = 0;
		this.mass.iI = 0;
	}
	setFilter(filterGroup, exclusionList) {
		if (filterGroup > 32 || filterGroup < 0) {
			throw new Error("filter group is out of bounds (0 - 32)");
		}

		this.filterGroup = filterGroup ? 1 << (filterGroup - 1) : 0;
		let exclusionMask = 0;
		for (let exclusion of exclusionList) {
			if (exclusion > 32 || exclusion < 2) {
				throw new Error("exclusion group is out of bounds (2 - 32)");
			}

			exclusionMask |= 1 << (exclusion - 1);
		}

		this.exclusionMask = ~exclusionMask;
	}
};
