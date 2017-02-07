const {Vector2D} = require("../framework/math.js");
const {AABB, Shape} = require("../objects/shape.js");

function makeFatAABB(aabb) {
	return new AABB(
		aabb.min.x - .2,
		aabb.min.y - .2,
		aabb.max.x + .2,
		aabb.max.y + .2
	);
}

class Node {
	constructor(aabb, parent = null, height = 0) {
		this.aabb = makeFatAABB(aabb);
		this.parent = parent;
		this.children = [];
		this.height = height;
	}
	get isLeaf() {
		return this.children.length === 0;
	}
}

class AABBTree {
	constructor() {
		this.count = 0;
		this.root = null;
	}
	insert(aabb) {
		let node = new Node(aabb);
		this.insertLeaf(node);
		return node;
	}
	remove(node) {
		removeLeaf(node);
	}
	checkMove(node, aabb) {
		if (node.aabb.contains(aabb))
			return false;
		node.aabb = makeFatAABB(aabb);
		this.removeLeaf(node);
		this.insertLeaf(node);
		return true;
	}
	query(node, callback) {
		let stack = [this.root];
		while (stack.length > 0) {
			let testNode = stack.pop();
			if (testNode == null)
				continue;
			if (testNode.aabb.test(node.aabb)) {
				if (testNode.isLeaf) {
					if (!callback(node, testNode))
						return;
				}
				stack.push(testNode.children[0], testNode.children[1]);
			}
		}
	}
	rayCast({p1, p2, maxFraction}, callback) {
		let r = p2.minus(p1).normalize();
		let v = Vector2D.cross1x2(1, r);
		let absV = new Vector2D(Math.abs(v.x), Math.abs(v.y));

		let t = p1.plus(p2.minus(p1).mul(maxFraction));
		let aabb = new AABB(
			Math.min(p1.x, t.x),
			Math.min(p1.y, t.y),
			Math.max(p1.x, t.x),
			Math.max(p1.y, t.y)
		);

		let stack = [this.root];
		while (stack.length > 0) {
			let node = stack.pop();
			if (!node.aabb.test(aabb))
				continue;
			let center = node.aabb.min.plus(node.aabb.max).mul(.5);
			let halfDims = node.aabb.max.minus(node.aabb.min).mul(.5);
			let separation = Math.abs(v.dot(p1.minus(center)) - absV.dot(halfDims));
			if (separation > 0)
				continue;
			if (node.isLeaf) {
				let value = callback({p1, p2, maxFraction}, node);
				if (value === 0)
					return;
				if (value > 0) {
					maxFraction = value;
					t = p1.plus(p2.minus(p1).mul(maxFraction));
					aabb = new AABB(
						Math.min(p1.x, t.x),
						Math.min(p1.y, t.y),
						Math.max(p1.x, t.x),
						Math.max(p1.y, t.y)
					);
				}
			} else {
				stack.push(node.children[0], node.children[1]);
			}
		}
	}
	insertLeaf(leaf) {
		this.count++;
		if (this.root === null) {
			this.root = leaf;
			this.root.parent = null;
			return;
		}

		let walk = this.root;
		while (!walk.isLeaf) {
			let perimeter = walk.aabb.perimeter;
			let combinedPerimeter = leaf.aabb.combine(walk.aabb).perimeter;
			let cost = combinedPerimeter * 2;
			let inheritanceCost = 2 * (combinedPerimeter - perimeter);

			let cost0;
			if (walk.children[0].isLeaf) {
				cost0 = leaf.aabb.combine(walk.children[0].aabb).perimeter + inheritanceCost;
			} else {
				let oldArea = walk.children[0].aabb.perimeter;
				let newArea = leaf.aabb.combine(walk.children[0].aabb).perimeter;
				cost0 = (newArea - oldArea) + inheritanceCost;
			}

			let cost1;
			if (walk.children[1].isLeaf) {
				cost1 = leaf.aabb.combine(walk.children[1].aabb).perimeter + inheritanceCost;
			} else {
				let oldArea = walk.children[1].aabb.perimeter;
				let newArea = leaf.aabb.combine(walk.children[1].aabb).perimeter;
				cost1 = (newArea - oldArea) + inheritanceCost;
			}

			if (cost < cost0 && cost < cost1)
				break;
			if (cost0 < cost1)
				walk = walk.children[0];
			else
				walk = walk.children[1];
		}

		let oldParent = walk.parent;
		let newParent = new Node(leaf.aabb.combine(walk.aabb), oldParent, walk.height + 1);
		if (oldParent != null) {
			if (oldParent.children[0] === walk)
				oldParent.children[0] = newParent;
			else
				oldParent.children[1] = newParent;
			newParent.children.push(walk, leaf);
			walk.parent = newParent;
			leaf.parent = newParent;
		} else {
			newParent.children.push(walk, leaf);
			walk.parent = newParent;
			leaf.parent = newParent;
			this.root = newParent;
		}

		while ((walk = walk.parent)) {
			walk = this.balance(walk);
			walk.height = Math.max(walk.children[0].height, walk.children[1].height) + 1;
			walk.aabb = walk.children[0].aabb.combine(walk.children[1].aabb);
		}
	}
	removeLeaf(leaf) {
		this.count--;
		if (leaf === this.root) {
			this.root = null;
			return;
		}

		let parent = leaf.parent;
		let grandParent = parent.parent;
		let sibling = parent.children[0] === leaf ? parent.children[1] : parent.children[0];

		if (grandParent != null) {
			if (grandParent.children[0] === parent)
				grandParent.children[0] = sibling;
			else
				grandParent.children[1] = sibling;
			sibling.parent = grandParent;

			let walk = parent;
			while ((walk = walk.parent)) {
				walk = this.balance(walk);
				walk.aabb = walk.children[0].aabb.combine(walk.children[1].aabb);
				walk.height = Math.max(walk.children[0].height, walk.children[1].height) + 1;
			}
		} else {
			this.root = sibling;
			sibling.parent = null;
		}
	}
	balance(node) {
		if (node.isLeaf || node.height < 2)
			return node;
		let a = node;
		let b = node.children[0];
		let c = node.children[1];
		let balance = c.height - b.height;
		if (balance > 1) {
			let f = c.children[0];
			let g = c.children[1];

			c.children[0] = a;
			c.parent = a.parent;
			a.parent = c;

			if (c.parent != null) {
				if (c.parent.children[0] === a)
					c.parent.children[0] = c;
				else
					c.parent.children[1] = c;
			} else {
				this.root = c;
			}

			if (f.height > g.height) {
				c.children[1] = f;
				a.children[1] = g;
				g.parent = a;
				a.aabb = b.aabb.combine(g.aabb);
				c.aabb = a.aabb.combine(f.aabb);
				a.height = Math.max(b.height, g.height) + 1;
				c.height = Math.max(a.height, f.height) + 1;
			} else {
				c.children[1] = g;
				a.children[1] = f;
				f.parent = a;
				a.aabb = b.aabb.combine(f.aabb);
				c.aabb = a.aabb.combine(g.aabb);
				a.height = Math.max(b.height, f.height) + 1;
				c.height = Math.max(a.height, g.height) + 1;
			}
			return c;
		}
		if (balance < -1) {
			let d = b.children[0];
			let e = b.children[1];

			b.children[0] = a;
			b.parent = a.parent;
			a.parent = b;

			if (b.parent != null) {
				if (b.parent.children[0] === a)
					b.parent.children[0] = b;
				else
					b.parent.children[1] = b;
			} else {
				this.root = b;
			}

			if (d.height > e.height) {
				b.children[1] = d;
				a.children[0] = e;
				e.parent = a;
				a.aabb = c.aabb.combine(e.aabb);
				b.aabb = a.aabb.combine(d.aabb);
				a.height = Math.max(c.height, d.height) + 1;
				b.height = Math.max(a.height, e.height) + 1;
			} else {
				b.children[1] = e;
				a.children[0] = d;
				d.parent = a;
				a.aabb = c.aabb.combine(d.aabb);
				b.aabb = a.aabb.combine(e.aabb);
				a.height = Math.max(c.height, d.height) + 1;
				b.height = Math.max(a.height, e.height) + 1;
			}
			return b;
		}
		return a;
	}
	get height() {
		if (this.root == null)
			return 0;
		return root.height;
	}
}

class PairSet {
	constructor() {
		this.map = new Map();
	}
	has({a, b}) {
		let key = `${a.id}:${b.id}`;
		return this.map.has(key);
	}
	get({a, b}) {
		let key = `${a.id}:${b.id}`;
		return this.map.get(key);
	}
	add({a, b}) {
		let key = `${a.id}:${b.id}`;
		if (!this.map.has(key))
			this.map.set(key, {a, b});
		return this.map.get(key);
	}
	delete({a, b}) {
		let key = `${a.id}:${b.id}`;
		this.map.delete(key);
	}
	*[Symbol.iterator]() {
		for (let kv of this.map)
			yield kv[1];
	}
	get size() {
		return this.map.size;
	}
}

const pairs = new PairSet();

function queryCallback(nodeA, nodeB) {
	//don't collide if on same body
	if (nodeA.shape.body === nodeB.shape.body)
		return true;

	//perform collision filtering
	let groupA = nodeA.shape.body.filterGroup;
	let groupB = nodeB.shape.body.filterGroup;
	let maskA = nodeA.shape.body.exclusionMask;
	let maskB = nodeB.shape.body.exclusionMask;
	if (!(groupA & maskB && groupB & maskA))
		return true;

	//use standard order and add to pairs
	[a, b] = Shape.order(nodeA.shape, nodeB.shape);
	pairs.add({a, b});
	return true;
}

module.exports = class BroadPhase {
	constructor() {
		this.tree = new AABBTree();
		this.shapeToNode = new Map();
		this.pairs = new PairSet();
	}
	insert(shape) {
		let node = this.tree.insert(shape.aabb);
		node.shape = shape;
		this.shapeToNode.set(shape, node);
		this.tree.query(node, queryCallback);
	}
	remove(shape) {
		this.tree.remove(this.shapeToNode.get(shape));
		this.shapeToNode.delete(shape);
	}
	collectMovedNodes() {
		let moved = [];
		for (let kv of this.shapeToNode) {
			let shape = kv[0];
			let node = kv[1];
			if (this.tree.checkMove(node, shape.aabb))
				moved.push(node);
		}
		return moved;
	}
	getPairs() {
		let movedNodes = this.collectMovedNodes();
		for (let node of movedNodes)
			this.tree.query(node, queryCallback);

		let oldPairs = [...pairs];
		for (let pair of oldPairs) {
			let fatA = this.shapeToNode.get(pair.a).aabb;
			let fatB = this.shapeToNode.get(pair.b).aabb;
			if (!fatA.test(fatB))
				pairs.delete(pair);
		}

		return pairs;
	}
	query(shape, callback) {
		this.tree.query(this.shapeToNode(shape), callback);
	}
	raycast({p1, p2, maxFraction}, callback) {
		this.tree.rayCast({p1, p2, maxFraction}, callback);
	}
};
