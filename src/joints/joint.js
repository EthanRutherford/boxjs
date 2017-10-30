let next = 0;

module.exports = class Joint {
	constructor({bodyA, bodyB, anchorA, anchorB}) {
		this.id = next++;
		this.bodyA = bodyA;
		this.bodyB = bodyB;
		this.anchorA = anchorA.minus(bodyA.mass.center);
		this.anchorB = anchorB.minus(bodyB.mass.center);
	}
	static clone(target, source, bodyA, bodyB) {
		target.id = source.id;
		target.bodyA = bodyA;
		target.bodyB = bodyB;
		target.anchorA = source.anchorA.clone();
		target.anchorB = source.anchorB.clone();
	}
};
