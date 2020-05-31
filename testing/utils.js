const {rgba} = require("2d-gl");

// circle helpers
function generateCirclePoints(radius, count) {
	const points = [];
	const sector = 2 * Math.PI / count;
	for (let i = 0; i < count; i++) {
		const angle = sector * i;
		points.push({
			x: Math.cos(angle) * radius,
			y: Math.sin(angle) * radius,
		});
	}

	return points;
}

function generateCircleColors(count) {
	const colors = [];
	for (let i = 0; i < count; i++) {
		colors.push(rgba(i / 40 + .5, i / 40 + .5, i / 40 + .5, 1));
	}

	return colors;
}

module.exports = {generateCirclePoints, generateCircleColors};
