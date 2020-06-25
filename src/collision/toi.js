const {Vector2D} = require("../framework/math");
const computeGjk = require("./gjk");

const lerpPositions = (shape, t) => [
	shape.body.prevTrans.lerp(shape.body.transform, t),
	shape.body.prevPos.lerp(shape.body.position, t),
];

function getSeparationFuncs(simplex, shapeA, tA, pA, shapeB, tB, pB) {
	if (simplex.length === 1) {
		const axis = simplex[0].support.clone().normalize();
		return {
			getSupports: (tA, tB) => [
				shapeA.getSupport(tA.transpose().times(axis)),
				shapeB.getSupport(tB.transpose().times(axis.neg())),
			],
			getSeparation(tA, pA, sA, tB, pB, sB) {
				const bToA = tB.times(sB).add(pB).sub(tA.times(sA).add(pA));
				return bToA.dot(axis);
			},
		};
	}

	if (simplex[0].vA === simplex[1].vA) {
		const axis = Vector2D.cross2x1(simplex[1].vB.minus(simplex[0].vB), 1).normalize();
		const localB = simplex[0].vB.plus(simplex[1].vB).mul(.5);
		const n = tB.times(axis);
		const ab = simplex[0].sA.minus(tB.times(localB).add(pB));
		const dot = ab.dot(n);
		if (dot < 0) {
			axis.negate();
		}

		return {
			getSupports: (tA, tB) => [
				shapeA.getSupport(tA.transpose().mul(tB.times(axis).neg())),
				null,
			],
			getSeparation(tA, pA, sA, tB, pB) {
				const normal = tB.times(axis);
				const aToB = tA.times(sA).add(pA).sub(tB.times(localB).add(pB));
				return aToB.dot(normal);
			},
		};
	}

	const axis = Vector2D.cross2x1(simplex[1].vA.minus(simplex[0].vA), 1).normalize();
	const localA = simplex[0].vA.plus(simplex[1].vA).mul(.5);
	const n = tA.times(axis);
	const ba = simplex[0].sB.minus(tA.times(localA).add(pA));
	const dot = ba.dot(n);
	if (dot < 0) {
		axis.negate();
	}

	return {
		getSupports: (tA, tB) => [
			null,
			shapeB.getSupport(tB.transpose().mul(tA.times(axis).neg())),
		],
		getSeparation(tA, pA, _, tB, pB, sB) {
			const normal = tA.times(axis);
			const bToA = tB.times(sB).add(pB).sub(tA.times(localA).add(pA));
			return bToA.dot(normal);
		},
	};
}

function findTimeOfImpact(t0, t1, shapeA, shapeB) {
	const rA = shapeA.radius || 0;
	const rB = shapeA.radius || 0;
	const target = rA + rB;
	const tol = .001;

	for (let outermost = 0; outermost < 20; outermost++) {
		const [tA, pA] = lerpPositions(shapeA, t0);
		const [tB, pB] = lerpPositions(shapeB, t0);

		const result = computeGjk(shapeA, tA, pA, shapeB, tB, pB);

		if (result.distance === target) {
			return outermost === 0 ? null : t0;
		}

		const {getSupports, getSeparation} = getSeparationFuncs(
			result.simplex,
			shapeA, tA, pA,
			shapeB, tB, pB,
		);
		for (let middle = 0; middle < 10; middle++) {
			const [tA1, pA1] = lerpPositions(shapeA, t1);
			const [tB1, pB1] = lerpPositions(shapeB, t1);
			const [sA, sB] = getSupports(tA1, tB1);
			let sep1 = getSeparation(tA1, pA1, sA, tB1, pB1, sB);

			if (sep1 > target) {
				return null;
			}

			if (sep1 > target - tol) {
				t0 = t1;
				break;
			}

			const [tA0, pA0] = lerpPositions(shapeA, t0);
			const [tB0, pB0] = lerpPositions(shapeB, t0);
			let sep0 = getSeparation(tA0, pA0, sA, tB0, pB0, sB);

			if (sep0 < target - tol) {
				return null;
			}

			if (sep0 <= target) {
				return t0;
			}

			let a0 = t0, a1 = t1;
			for (let i = 0; i < 50; i++) {
				const t = i & 1 ? a0 + (target - sep0) * (a1 - a0) / (sep1 - sep0) : .5 * (a0 + a1);

				const [tA, pA] = lerpPositions(shapeA, t);
				const [tB, pB] = lerpPositions(shapeB, t);
				const sep = getSeparation(tA, pA, sA, tB, pB, sB);

				if (sep > target - tol && sep <= target) {
					t1 = t;
					break;
				}

				if (sep > target) {
					a0 = t;
					sep0 = sep;
				} else {
					a1 = t;
					sep1 = sep;
				}
			}
		}
	}

	return null;
}

module.exports = findTimeOfImpact;
