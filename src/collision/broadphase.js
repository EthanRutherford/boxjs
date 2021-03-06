const {Vector2D} = require("../framework/math");
const {AABB, Shape} = require("../objects/shape");

function makeFatAABB(aabb) {
	return new AABB(
		aabb.min.x - .2,
		aabb.min.y - .2,
		aabb.max.x + .2,
		aabb.max.y + .2,
	);
}

class Node {
	constructor(aabb, parent = null, height = 0) {
		this.aabb = aabb;
		this.parent = parent;
		this.children = [];
		this.height = height;
		this.shape = null;
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
		const node = new Node(makeFatAABB(aabb));
		this.insertLeaf(node);
		return node;
	}
	remove(node) {
		this.removeLeaf(node);
	}
	clear() {
		this.count = 0;
		this.root = null;
	}
	checkMove(node, aabb, displacement) {
		const stretchAABB = aabb.clone();
		(displacement.x < 0 ? stretchAABB.min : stretchAABB.max).x += displacement.x;
		(displacement.y < 0 ? stretchAABB.min : stretchAABB.max).y += displacement.y;

		if (node.aabb.contains(stretchAABB)) {
			return false;
		}

		node.aabb = makeFatAABB(aabb);

		const d = displacement.times(2);

		(d.x < 0 ? node.aabb.min : node.aabb.max).x += d.x;
		(d.y < 0 ? node.aabb.min : node.aabb.max).y += d.y;

		this.removeLeaf(node);
		this.insertLeaf(node);
		return true;
	}
	query(node, callback) {
		const stack = [this.root];
		while (stack.length > 0) {
			const testNode = stack.pop();
			if (testNode == null) {
				continue;
			}
			if (testNode.aabb.test(node.aabb)) {
				if (testNode.isLeaf) {
					if (!callback(node, testNode)) {
						return;
					}
				}
				stack.push(testNode.children[0], testNode.children[1]);
			}
		}
	}
	rayCast({p1, p2, maxFraction}, callback) {
		const r = p2.minus(p1).normalize();
		const v = Vector2D.cross1x2(1, r);
		const absV = new Vector2D(Math.abs(v.x), Math.abs(v.y));

		let t = p1.plus(p2.minus(p1).mul(maxFraction));
		let aabb = new AABB(
			Math.min(p1.x, t.x),
			Math.min(p1.y, t.y),
			Math.max(p1.x, t.x),
			Math.max(p1.y, t.y),
		);

		const stack = [this.root];
		while (stack.length > 0) {
			const node = stack.pop();
			if (node == null || !node.aabb.test(aabb)) {
				continue;
			}

			const center = node.aabb.min.plus(node.aabb.max).mul(.5);
			const halfDims = node.aabb.max.minus(node.aabb.min).mul(.5);
			const separation = Math.abs(v.dot(p1.minus(center))) - absV.dot(halfDims);
			if (separation > 0) {
				continue;
			}

			if (node.isLeaf) {
				const value = callback({p1, p2, maxFraction}, node);
				if (value === 0) {
					return;
				}

				if (value > 0) {
					maxFraction = value;
					t = p1.plus(p2.minus(p1).mul(maxFraction));
					aabb = new AABB(
						Math.min(p1.x, t.x),
						Math.min(p1.y, t.y),
						Math.max(p1.x, t.x),
						Math.max(p1.y, t.y),
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
			const perimeter = walk.aabb.perimeter;
			const combinedPerimeter = leaf.aabb.combine(walk.aabb).perimeter;
			const cost = combinedPerimeter * 2;
			const inheritanceCost = 2 * (combinedPerimeter - perimeter);

			let cost0;
			if (walk.children[0].isLeaf) {
				cost0 = leaf.aabb.combine(walk.children[0].aabb).perimeter + inheritanceCost;
			} else {
				const oldArea = walk.children[0].aabb.perimeter;
				const newArea = leaf.aabb.combine(walk.children[0].aabb).perimeter;
				cost0 = (newArea - oldArea) + inheritanceCost;
			}

			let cost1;
			if (walk.children[1].isLeaf) {
				cost1 = leaf.aabb.combine(walk.children[1].aabb).perimeter + inheritanceCost;
			} else {
				const oldArea = walk.children[1].aabb.perimeter;
				const newArea = leaf.aabb.combine(walk.children[1].aabb).perimeter;
				cost1 = (newArea - oldArea) + inheritanceCost;
			}

			if (cost < cost0 && cost < cost1) {
				break;
			}

			if (cost0 < cost1) {
				walk = walk.children[0];
			} else {
				walk = walk.children[1];
			}
		}

		const oldParent = walk.parent;
		const newParent = new Node(leaf.aabb.combine(walk.aabb), oldParent, walk.height + 1);
		if (oldParent != null) {
			if (oldParent.children[0] === walk) {
				oldParent.children[0] = newParent;
			} else {
				oldParent.children[1] = newParent;
			}

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

		const parent = leaf.parent;
		const grandParent = parent.parent;
		const sibling = parent.children[0] === leaf ? parent.children[1] : parent.children[0];

		if (grandParent != null) {
			if (grandParent.children[0] === parent) {
				grandParent.children[0] = sibling;
			} else {
				grandParent.children[1] = sibling;
			}

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
		if (node.isLeaf || node.height < 2) {
			return node;
		}

		const a = node;
		const b = node.children[0];
		const c = node.children[1];
		const balance = c.height - b.height;
		if (balance > 1) {
			const f = c.children[0];
			const g = c.children[1];

			c.children[0] = a;
			c.parent = a.parent;
			a.parent = c;

			if (c.parent != null) {
				if (c.parent.children[0] === a) {
					c.parent.children[0] = c;
				} else {
					c.parent.children[1] = c;
				}
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
			const d = b.children[0];
			const e = b.children[1];

			b.children[0] = a;
			b.parent = a.parent;
			a.parent = b;

			if (b.parent != null) {
				if (b.parent.children[0] === a) {
					b.parent.children[0] = b;
				} else {
					b.parent.children[1] = b;
				}
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
		if (this.root == null) {
			return 0;
		}

		return this.root.height;
	}
}

class BroadPhase {
	constructor(manifolds) {
		this.tree = new AABBTree();
		this.shapeToNode = {};
		this.pairs = manifolds;
		this.queryCallback = (nodeA, nodeB) => {
			// don't collide if on same body
			if (nodeA.shape.body === nodeB.shape.body) {
				return true;
			}

			// perform collision filtering
			const groupA = nodeA.shape.body.filterGroup;
			const groupB = nodeB.shape.body.filterGroup;
			const maskA = nodeA.shape.body.exclusionMask;
			const maskB = nodeB.shape.body.exclusionMask;
			if (!(groupA & maskB && groupB & maskA)) {
				return true;
			}

			// use standard order and add to pairs
			const [a, b] = Shape.order(nodeA.shape, nodeB.shape);
			this.pairs.add({a, b});
			return true;
		};
	}
	insert(shape) {
		const node = this.tree.insert(shape.aabb);
		node.shape = shape;
		this.shapeToNode[shape.id] = node;
		this.tree.query(node, this.queryCallback);
	}
	remove(shape) {
		this.tree.remove(this.shapeToNode[shape.id]);
		delete this.shapeToNode[shape.id];
		for (const [key, pair] of this.pairs.map) {
			if (pair.shapeA === shape || pair.shapeB === shape) {
				this.pairs.delete(key);

				if (pair.isCollided) {
					pair.shapeA.body.setAsleep(false);
					pair.shapeB.body.setAsleep(false);
				}
			}
		}
	}
	flush() {
		this.tree.clear();
		this.shapeToNode = {};
	}
	collectMovedNodes() {
		const moved = [];
		for (const node of Object.values(this.shapeToNode)) {
			const shape = node.shape;
			const displacement = shape.body.position.minus(shape.body.prevPos);
			if (this.tree.checkMove(node, shape.aabb, displacement)) {
				moved.push(node);
			}
		}
		return moved;
	}
	updatePairs() {
		for (const [key, pair] of this.pairs.map) {
			const fatA = this.shapeToNode[pair.shapeA.id].aabb;
			const fatB = this.shapeToNode[pair.shapeB.id].aabb;
			if (!fatA.test(fatB)) {
				this.pairs.delete(key);
			}
		}

		// query the tree and add any new manifolds
		const movedNodes = this.collectMovedNodes();
		for (const node of movedNodes) {
			this.tree.query(node, this.queryCallback);
		}
	}
	query(aabb, callback) {
		this.tree.query({aabb}, (_, nodeB) => {
			callback(nodeB.shape);
			return true;
		});
	}
	raycast(inputRay, callback) {
		this.tree.rayCast(inputRay, (ray, node) => {
			return callback(ray, node.shape);
		});
	}
	debugGetNodes() {
		return Object.keys(this.shapeToNode).map((id) => {
			const node = this.shapeToNode[id];
			return {
				shape: node.shape,
				aabb: node.aabb.clone(),
			};
		});
	}
}

module.exports = {
	Node,
	AABBTree,
	BroadPhase,
};
