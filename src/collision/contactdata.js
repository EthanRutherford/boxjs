module.exports = class ContactData {
	constructor(manifold, flipped, dt) {
		this.shape = flipped ? manifold.shapeB : manifold.shapeA;
		this.otherShape = flipped ? manifold.shapeA : manifold.shapeB;
		this.normalForces = [];
		this.tangentForces = [];
		if (!manifold.hasSensor) {
			for (let contact of manifold.contacts) {
				let normalForce = manifold.normal.times(contact.normalImpulse / dt);
				let tangentForce = manifold.tangent.times(contact.tangentImpulse / dt);
				if (flipped) {
					normalForce = normalForce.neg();
					tangentForce = tangentForce.neg();
				}

				this.normalForces.push(normalForce);
				this.tangentForces.push(tangentForce);
			}
		}
	}
};