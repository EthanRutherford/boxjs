const {Vector2D} = require("../framework/math.js");

module.exports = class MassData {
	constructor() {
		this.m = 0;
		this.iM = 0;
		this.i = 0;
		this.iI = 0;
		this.center = new Vector2D();
	}
};
