const {mat4: Mat4} = require("./gl-matrix.min");

const fragmentShader = `
varying highp vec2 vTextureCoord;
uniform sampler2D uSampler;
void main(void) {
	gl_FragColor = texture2D(uSampler, vTextureCoord.st);
}`;

const vertexShader = `
attribute vec3 aVertexPosition;
attribute vec2 aTextureCoord;
uniform mat4 uMVMatrix;
uniform mat4 uPMatrix;
varying highp vec2 vTextureCoord;
void main(void) {
	gl_Position = uPMatrix * uMVMatrix * vec4(aVertexPosition, 1);
	vTextureCoord = aTextureCoord;
}`;

const simpleFragmentShader = `
varying mediump vec4 vColor;
void main(void) {
	gl_FragColor = vColor;
}`;

const simpleVertexShader = `
attribute vec3 aVertexPosition;
attribute vec4 aVertexColor;
uniform mat4 uMVMatrix;
uniform mat4 uPMatrix;
varying vec4 vColor;
void main(void) {
	gl_Position = uPMatrix * uMVMatrix * vec4(aVertexPosition, 1);
	vColor = aVertexColor;
}`;

const gl = {
	context: null,
	canvas: null,
	height: 0,
	aspect: 0,
	bounds: {x0: 0, x1: 0, y0: 0, y1: 0},
	mvMatrix: Mat4.create(),
	pMatrix: Mat4.create(),
};

function initGL(canvas, height) {
	gl.context = canvas.getContext("webgl");
	if (!gl.context) {
		throw new Error("could not initialize WebGL");
	}

	gl.height = height;
	gl.canvas = canvas;
	gl.context.clearColor(0, 0, 0, 1);
	gl.context.enable(gl.context.BLEND);
	gl.context.enable(gl.context.DEPTH_TEST);
	gl.context.blendFunc(gl.context.SRC_ALPHA, gl.context.ONE_MINUS_SRC_ALPHA);
	resize();
	initShaders();
	window.addEventListener("resize", resize);
	return gl.context;
}

function resize() {
	const style = window.getComputedStyle(gl.canvas);
	gl.canvas.width = parseInt(style.width, 10);
	gl.canvas.height = parseInt(style.height, 10);
	gl.context.viewport(0, 0, gl.canvas.width, gl.canvas.height);
	gl.aspect = gl.canvas.width / gl.canvas.height;
}

function getShader(src, isFrag) {
	const shader = gl.context.createShader(isFrag ? gl.context.FRAGMENT_SHADER : gl.context.VERTEX_SHADER);
	gl.context.shaderSource(shader, src);
	gl.context.compileShader(shader);
	if (!gl.context.getShaderParameter(shader, gl.context.COMPILE_STATUS)) {
		throw new Error(gl.context.getShaderInfoLog(shader));
	}

	return shader;
}

function initShaders() {
	//create the texture shader
	let frag = getShader(fragmentShader, true);
	let vert = getShader(vertexShader, false);
	gl.textureShader = gl.context.createProgram();
	gl.context.attachShader(gl.textureShader, vert);
	gl.context.attachShader(gl.textureShader, frag);
	gl.context.linkProgram(gl.textureShader);

	if (!gl.context.getProgramParameter(gl.textureShader, gl.context.LINK_STATUS)) {
		throw new Error("Could not initialize texture shader");
	}

	gl.context.useProgram(gl.textureShader);
	gl.textureShader.vertextPositionAttribute =
		gl.context.getAttribLocation(gl.textureShader, "aVertexPosition");
	gl.textureShader.textureCoordAttribute =
		gl.context.getAttribLocation(gl.textureShader, "aTextureCoord");
	gl.context.enableVertexAttribArray(gl.textureShader.textureCoordAttribute);
	gl.context.enableVertexAttribArray(gl.textureShader.vertexPositionAttribute);
	gl.textureShader.pMatrixUniform = gl.context.getUniformLocation(
		gl.textureShader, "uPMatrix");
	gl.textureShader.mvMatrixUniform = gl.context.getUniformLocation(
		gl.textureShader, "uMVMatrix");
	gl.textureShader.samplerUniform = gl.context.getUniformLocation(
		gl.textureShader, "uSampler");

	//create the simple shader
	frag = getShader(simpleFragmentShader, true);
	vert = getShader(simpleVertexShader, false);
	gl.simpleShader = gl.context.createProgram();
	gl.context.attachShader(gl.simpleShader, vert);
	gl.context.attachShader(gl.simpleShader, frag);
	gl.context.linkProgram(gl.simpleShader);

	if (!gl.context.getProgramParameter(gl.simpleShader, gl.context.LINK_STATUS)) {
		throw new Error("Could not initialize simple shader");
	}

	gl.context.useProgram(gl.simpleShader);
	gl.simpleShader.vertextPositionAttribute =
		gl.context.getAttribLocation(gl.simpleShader, "aVertexPosition");
	gl.simpleShader.vertextColorAttribute =
		gl.context.getAttribLocation(gl.simpleShader, "aVertexColor");
	gl.context.enableVertexAttribArray(gl.simpleShader.vertexPositionAttribute);
	gl.context.enableVertexAttribArray(gl.simpleShader.textureColorAttribute);
	gl.simpleShader.pMatrixUniform = gl.context.getUniformLocation(
		gl.simpleShader, "uPMatrix");
	gl.simpleShader.mvMatrixUniform = gl.context.getUniformLocation(
		gl.simpleShader, "uMVMatrix");
}

function setCurShader(shader) {
	if (gl.curShader !== shader) {
		gl.curShader = shader;
		gl.context.useProgram(shader);
	}
	if (gl.curShader.projectionDirty) {
		gl.context.uniformMatrix4fv(gl.curShader.pMatrixUniform, false, gl.pMatrix);
		gl.curShader.projectionDirty = false;
	}
}

function setProjectionDirty() {
	gl.textureShader.projectionDirty = true;
	gl.simpleShader.projectionDirty = true;
}

function setModelView() {
	gl.context.uniformMatrix4fv(gl.curShader.mvMatrixUniform, false, gl.mvMatrix);
}

function setOrtho(x, y, zoom) {
	zoom = 1 / zoom;
	const w = (gl.height * gl.aspect / 2) * zoom;
	const h = (gl.height / 2) * zoom;
	Mat4.ortho(gl.pMatrix, x - w, x + w, y - h, y + h, 0, -1);
	setProjectionDirty();
	gl.bounds.x0 = x - w;
	gl.bounds.x1 = x + w;
	gl.bounds.y0 = y - h;
	gl.bounds.y1 = y + h;
}

function viewportToWorld(pos) {
	const x = pos.x / gl.canvas.width - .5;
	const y = .5 - pos.y / gl.canvas.height;

	const ans = {};
	ans.x = x / gl.pMatrix[0] * 2 - gl.pMatrix[12] / gl.pMatrix[0];
	ans.y = y / gl.pMatrix[5] * 2 - gl.pMatrix[13] / gl.pMatrix[5];

	return ans;
}

function getBounds() {
	return Object.assign({}, gl.bounds);
}

const getZOrder = (() => {
	const zMaxValue = 8000;
	let zOrder = zMaxValue;
	return function() {
		if (--zOrder < 0) {
			zOrder = zMaxValue;
		}

		return zOrder;
	};
})();

function calcZValue(zIndex, zOrder) {
	return (((1000 - zIndex) * 2000) + zOrder) / 16000000;
}

function arrayToZArray(array, z) {
	const zArray = [];
	for (let i = 0; i < array.length; i += 2) {
		zArray.push(array[i], array[i + 1], z);
	}

	return zArray;
}

class SimpleRenderable {
	constructor(verts, colors, zIndex = 0) {
		//create and fill buffers
		if (verts.length % 2) {
			throw new Error("Vertex buffer must have even length");
		}

		if (verts.length < 6) {
			throw new Error("Vertex buffer must have at least 3 points");
		}

		if (verts.length / 2 !== colors.length / 4) {
			throw new Error("Vertex and color count must match");
		}

		this.zIndex = Math.round(zIndex);
		this.zOrder = getZOrder();
		this.z = calcZValue(this.zIndex, this.zOrder);
		this.vertCount = verts.length / 2;
		verts = arrayToZArray(verts, this.z);
		this.vertBuf = gl.context.createBuffer();
		this.colorBuf = gl.context.createBuffer();
		this.drawMode = gl.context.TRIANGLE_FAN;
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.vertBuf);
		gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array(verts), gl.context.STATIC_DRAW);
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.colorBuf);
		gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array(colors), gl.context.STATIC_DRAW);
	}
	updateBuffers({verts, colors}) {
		//if the buffers have been deleted, we shouldn't be updating
		if (this.vertBuf == null) {
			throw new Error("Cannot update deleted buffers");
		}

		if (verts) {
			if (verts.length / 2 !== this.vertCount) {
				throw new Error("Updated vertex buffer length does not match original length");
			}

			verts = arrayToZArray(verts, this.z);
			gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.vertBuf);
			gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array(verts), gl.context.STATIC_DRAW);
		}
		if (colors) {
			if (colors.length / 4 !== this.vertCount) {
				throw new Error("Updated color buffer length does not match original length");
			}

			gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.colorBuf);
			gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array(colors), gl.context.STATIC_DRAW);
		}
	}
	deleteBuffers() {
		if (this.vertBuf == null) {
			return;
		}

		gl.context.deleteBuffer(this.vertBuf);
		gl.context.deleteBuffer(this.colorBuf);
		this.vertBuf = null;
		this.colorBuf = null;
	}
	render(pos, r) {
		//if the buffers have been deleted, we can't draw
		if (this.vertBuf == null) {
			throw new Error("Attempt to render deleted buffers");
		}

		//use the simple program
		setCurShader(gl.simpleShader);
		//set model matrix
		Mat4.identity(gl.mvMatrix);
		Mat4.fromTranslation(gl.mvMatrix, [pos.x, pos.y, 0]);
		Mat4.rotateZ(gl.mvMatrix, gl.mvMatrix, r);
		setModelView();
		//update active buffers
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.vertBuf);
		gl.context.vertexAttribPointer(gl.simpleShader.vertextPositionAttribute,
			3, gl.context.FLOAT, false, 0, 0);
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.colorBuf);
		gl.context.vertexAttribPointer(gl.simpleShader.vertextColorAttribute,
			4, gl.context.FLOAT, false, 0, 0);
		gl.context.drawArrays(this.drawMode, 0, this.vertCount);
	}
}

class TextureRenderable {
	constructor(imageSrc, width, height, zIndex = 0) {
		width /= 2;
		height /= 2;
		let verts = [width, height, -width, height, width, -height, -width, -height];
		const tex = [1, 1, 0, 1, 1, 0, 0, 0];

		this.zIndex = Math.round(zIndex);
		this.zOrder = getZOrder();
		this.z = calcZValue(this.zIndex, this.zOrder);
		verts = arrayToZArray(verts, this.z);
		this.vertBuf = gl.context.createBuffer();
		this.texBuf = gl.context.createBuffer();
		this.texture = gl.context.createTexture();
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.vertBuf);
		gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array(verts), gl.context.STATIC_DRAW);
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.texBuf);
		gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array(tex), gl.context.STATIC_DRAW);

		//set up fallback shader logic
		this.fallbackColorBuf = gl.context.createBuffer();
		gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.fallbackColorBuf);
		gl.context.bufferData(gl.context.ARRAY_BUFFER, new Float32Array([
			1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		]), gl.context.STATIC_DRAW);

		this.promise = new Promise((resolve, reject) => {
			const img = new Image();
			img.src = imageSrc;
			img.onload = () => {
				gl.context.bindTexture(gl.context.TEXTURE_2D, this.texture);
				gl.context.pixelStorei(gl.context.UNPACK_FLIP_Y_WEBGL, true);
				gl.context.texImage2D(gl.context.TEXTURE_2D, 0, gl.context.RGBA,
					gl.context.RGBA, gl.context.UNSIGNED_BYTE, img);
				gl.context.texParameteri(gl.context.TEXTURE_2D,
					gl.context.TEXTURE_MAG_FILTER, gl.context.LINEAR);
				gl.context.texParameteri(gl.context.TEXTURE_2D, gl.context.TEXTURE_MIN_FILTER,
					gl.context.NEAREST_MIPMAP_LINEAR);
				gl.context.generateMipmap(gl.context.TEXTURE_2D);
				gl.context.bindTexture(gl.context.TEXTURE_2D, null);
				resolve(this);
			};
			img.onerror = () => reject("image failed to load");
		});
		this.promise.then(() => this.onLoaded());
	}
	onLoaded() {
		this.resolved = true;
		gl.context.deleteBuffer(this.fallbackColorBuf);
		this.fallbackColorBuf = null;
	}
	deleteBuffers() {
		if (this.vertBuf == null) {
			return;
		}

		gl.context.deleteBuffer(this.vertBuf);
		gl.context.deleteBuffer(this.texBuf);
		gl.context.deleteTexture(this.texture);
		this.vertBuf = null;
		this.texBuf = null;
		this.texture = null;
	}
	render(pos, r) {
		//if the buffers have been deleted, we can't draw
		if (this.vertBuf == null) {
			throw new Error("Attempt to render deleted buffers");
		}

		if (this.resolved) {
			//use the texture program
			setCurShader(gl.textureShader);
			//set up model view matrix using input values
			Mat4.identity(gl.mvMatrix);
			Mat4.fromTranslation(gl.mvMatrix, [pos.x, pos.y, 0]);
			Mat4.rotateZ(gl.mvMatrix, gl.mvMatrix, r);
			setModelView();
			//update the active buffers and texture
			gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.vertBuf);
			gl.context.vertexAttribPointer(gl.textureShader.vertexPositionAttribute,
				3, gl.context.FLOAT, false, 0, 0);
			gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.texBuf);
			gl.context.vertexAttribPointer(gl.textureShader.textureCoordAttribute,
				2, gl.context.FLOAT, false, 0, 0);
			gl.context.activeTexture(gl.context.TEXTURE0);
			gl.context.bindTexture(gl.context.TEXTURE_2D, this.texture);
			gl.context.uniform1i(gl.textureShader.samplerUniform, 0);
			//finally, actually draw the sprite
			gl.context.drawArrays(gl.context.TRIANGLE_STRIP, 0, 4);
		} else {
			//use the texture program
			setCurShader(gl.simpleShader);
			//set up model view matrix using input values
			Mat4.identity(gl.mvMatrix);
			Mat4.fromTranslation(gl.mvMatrix, [pos.x, pos.y, 0]);
			Mat4.rotateZ(gl.mvMatrix, gl.mvMatrix, r);
			setModelView();
			//update active buffers
			gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.vertBuf);
			gl.context.vertexAttribPointer(gl.simpleShader.vertextPositionAttribute,
				3, gl.context.FLOAT, false, 0, 0);
			gl.context.bindBuffer(gl.context.ARRAY_BUFFER, this.fallbackColorBuf);
			gl.context.vertexAttribPointer(gl.simpleShader.vertextColorAttribute,
				4, gl.context.FLOAT, false, 0, 0);
			gl.context.drawArrays(gl.context.TRIANGLE_STRIP, 0, 4);
		}
	}
}

module.exports = {
	SimpleRenderable,
	TextureRenderable,
	initGL,
	setOrtho,
	viewportToWorld,
	getBounds,
};
