module.exports = class ContactData {
	constructor(manifold, flipped, dt) {
		this.shape = flipped ? manifold.shapeB : manifold.shapeA;
		this.otherShape = flipped ? manifold.shapeA : manifold.shapeB;
		this.normalForces = [];
		this.tangentForces = [];
		if (!manifold.hasSensor) {
			for (const contact of manifold.contacts) {
				const normalForce = manifold.normal.times(contact.normalImpulse / dt);
				const tangentForce = manifold.tangent.times(contact.tangentImpulse / dt);
				if (flipped) {
					normalForce.negate();
					tangentForce.negate();
				}

				this.normalForces.push(normalForce);
				this.tangentForces.push(tangentForce);
			}
		}
	}
};
