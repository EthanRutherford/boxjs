const {Vector2D} = require("../framework/math");

function makeSimplexVert(vA, tA, pA, vB, tB, pB) {
	const sA = tA.times(vA).add(pA);
	const sB = tB.times(vB).add(pB);

	return {
		vA, vB, sA, sB,
		support: sB.minus(sA),
		bary: 1,
	};
}

const simplexSolverByDim = {
	1(simplex) {
		return [simplex[0]];
	},
	2(simplex) {
		const [v1, v2] = simplex;
		const s1 = v1.support;
		const s2 = v2.support;
		const edge12 = s2.minus(s1);

		const dotS1 = -s1.dot(edge12);
		if (dotS1 <= 0) {
			v1.bary = 1;
			return [v1];
		}

		const dotS2 = s2.dot(edge12);
		if (dotS2 <= 0) {
			v2.bary = 1;
			return [v2];
		}

		const invDot = 1 / (dotS1 + dotS2);
		v1.bary = dotS2 * invDot;
		v2.bary = dotS1 * invDot;
		return [v1, v2];
	},
	3(simplex) {
		const [v1, v2, v3] = simplex;
		const s1 = v1.support;
		const s2 = v2.support;
		const s3 = v3.support;

		const edge12 = s2.minus(s1);
		const dot12S1 = -s1.dot(edge12);
		const dot12S2 = s2.dot(edge12);

		const edge13 = s3.minus(s1);
		const dot13S1 = -s1.dot(edge13);
		const dot13S3 = s3.dot(edge13);

		const edge23 = s3.minus(s2);
		const dot23S2 = -s2.dot(edge23);
		const dot23S3 = s3.dot(edge23);

		const n123 = edge12.cross(edge13);
		const d123a = n123 * s2.cross(s3);
		const d123b = n123 * s3.cross(s1);
		const d123c = n123 * s1.cross(s2);

		if (dot12S1 <= 0 && dot13S1 <= 0) {
			v1.bary = 1;
			return [v1];
		}

		if (dot12S1 > 0 && dot12S2 > 0 && d123c <= 0) {
			const invDot = 1 / (dot12S1 + dot12S2);
			v1.bary = dot12S2 * invDot;
			v2.bary = dot12S1 * invDot;
			return [v1, v2];
		}

		if (dot13S1 > 0 && dot13S3 > 0 && d123b <= 0) {
			const invDot = 1 / (dot13S1 + dot13S3);
			v1.bary = dot13S3 * invDot;
			v3.bary = dot13S1 * invDot;
			return [v1, v3];
		}

		if (dot12S2 <= 0 && dot23S2 <= 0) {
			v2.bary = 1;
			return [v2];
		}

		if (dot13S3 <= 0 && dot23S3 <= 0) {
			v3.bary = 1;
			return [v3];
		}

		if (dot23S2 > 0 && dot23S3 > 0 && d123a <= 0) {
			const invDot = 1 / (dot23S2 + dot23S3);
			v2.bary = dot23S3 * invDot;
			v3.bary = dot23S2 * invDot;
			return [v2, v3];
		}

		const invDot = 1 / (d123a + d123b + d123c);
		v1.bary = d123a * invDot;
		v2.bary = d123b * invDot;
		v3.bary = d123c * invDot;
		return [v1, v2, v3];
	},
};

const searchDirByDim = {
	1(simplex) {
		return simplex[0].support.neg();
	},
	2(simplex) {
		const [v1, v2] = simplex;
		const s1 = v1.support;
		const s2 = v2.support;
		const edge12 = s2.minus(s1);
		const sign = edge12.cross(s1.neg());
		return sign > 0 ? Vector2D.cross1x2(1, edge12) : Vector2D.cross2x1(edge12, 1);
	},
};

const getPointsByDim = {
	1(simplex) {
		return [simplex[0].sA, simplex[0].sB];
	},
	2(simplex) {
		const [v1, v2] = simplex;
		return [
			v1.sA.times(v1.bary).add(v2.sA.times(v2.bary)),
			v1.sB.times(v1.bary).add(v2.sB.times(v2.bary)),
		];
	},
	3(simplex) {
		const [v1, v2, v3] = simplex;
		const out = v1.sA.times(v1.bary).add(v2.sA.times(v2.bary)).add(v3.sA.times(v3.bary));
		return [out, out.clone()];
	},
};

function computeGjk(shapeA, tA, pA, shapeB, tB, pB) {
	const initialVert = makeSimplexVert(shapeA.points[0], tA, pA, shapeB.points[0], tB, pB);

	let simplex = [initialVert];
	while (true) {
		const prevSim = simplex;
		simplex = simplexSolverByDim[simplex.length](simplex);
		if (simplex.length === 3) {
			break;
		}

		const dir = searchDirByDim[simplex.length](simplex);
		if (dir.lsqr < Number.EPSILON) {
			break;
		}

		const newVert = makeSimplexVert(
			shapeA.getSupport(tA.transpose().times(dir.neg())), tA, pA,
			shapeB.getSupport(tB.transpose().times(dir)), tB, pB,
		);

		if (prevSim.some((v) => v.vA === newVert.vA && v.vB === newVert.vB)) {
			break;
		}

		simplex.push(newVert);
	}

	const [pointA, pointB] = getPointsByDim[simplex.length](simplex);
	return {pointA, pointB, distance: pointA.minus(pointB).length, simplex};
}

module.exports = computeGjk;
