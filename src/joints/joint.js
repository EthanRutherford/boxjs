module.exports = class Joint {
	constructor({bodyA, bodyB, anchorA, anchorB}) {
		this.bodyA = bodyA;
		this.bodyB = bodyB;
		this.anchorA = anchorA.minus(bodyA.mass.center);
		this.anchorB = anchorB.minus(bodyB.mass.center);
	}
};
