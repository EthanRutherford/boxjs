(() => {
	//Regex for getting preload directives
	const preloadRegex = /\/\/#[ \t]+preload[ \t]+(\S*)/g;
	//virtual filesystem
	const cache = {files: {}, packages: {}, outputDir: "/"};
	//symbols
	const sDeps = "/deps";
	const sCode = "/code";
	const sModule = "/module";
	const sPromise = "/promise";
	//add a path to the virtual filesystem
	function addPath(data, pathName) {
		pathName = pathName.substr(1);
		if (pathName === "") return data;
		const parts = pathName.split("/");
		for (const part of parts) {
			if (!(part in data)) {
				data[part] = {};
			}
			data = data[part];
		}
		return data;
	}
	//get path from the virtual filesystem
	function getPath(data, pathName) {
		pathName = pathName.substr(1);
		if (pathName === "") return data;
		const parts = pathName.split("/");
		for (const part of parts) {
			if (!(part in data)) {
				return undefined;
			}
			data = data[part];
		}
		return data;
	}
	//merge two objects (naive, assumes nothing more complex than JSON)
	function merge(target, source) {
		if (!(target instanceof Object)) {
			return source;
		}
		if (target instanceof Array) {
			return target.concat(source);
		}
		for (const key of Object.keys(source)) {
			if (key in target) {
				target[key] = merge(target[key], source[key]);
			} else {
				target[key] = source[key];
			}
		}
		return target;
	}
	//determine if path is a package path
	function isPackage(name) {
		return !(name[0] === "/" || name.startsWith("./") || name.startsWith("../"));
	}
	//call to parse urls
	function parseUrl(src, relativeTo) {
		//create a url out of the source
		const url = new URL(src, this.location.origin + (relativeTo || ""));
		//return the pathname
		return url.pathname;
	}
	//reads the source for any preload directives
	function getDependencies(code) {
		const deps = [];
		let match;
		while ((match = preloadRegex.exec(code))) {
			const [, dep] = match;
			deps.push(dep);
		}
		return deps;
	}
	//creates a new function which only has access to global scope
	function getModuleExecutor(code, src, basename) {
		const sourceUrl = (basename ? `preload://modules/${basename}` : location.origin) + src;
		if (code[code.length - 1] !== "\n") code += "\n";
		//stringify the source, adding the sourceURL for debugging
		code = JSON.stringify(`${code}//# sourceURL=${sourceUrl}`).slice(1, -1);
		//create the code which creates the module executor
		code = `return eval("(require, module, exports) => {\\n${code}\\n}");`;
		//use the Function constructor to escape to global scope
		//and return the constructed executor
		return new Function(code)();
	}
	//executes the code, and sets module exports
	function execute(code, src, obj, pack) {
		//initialize the module object
		const module = {};
		Object.defineProperty(module, "exports", {
			get: () => obj[sModule],
			set: (value) => obj[sModule] = value,
		});
		//create the executor
		const executor = getModuleExecutor(code, src, pack.name);
		//construct and execute the function
		executor((dep) => requireCore(dep, src, pack), module, module.exports);
		//return module exports
		return module.exports;
	}
	//call to load the resource
	function load(src) {
		return fetch(new Request(src, {method: "GET"}));
	}
	//looks for a .js or .json path
	function resolvePath(pack, pathName) {
		if (!pathName.endsWith(".js") && !pathName.endsWith(".json")) {
			const parts = pathName.split("/");
			const last = parts.splice(-1)[0];
			const rest = parts.join("/");
			const obj = getPath(pack.files, rest);
			const namejs = `${last}.js`;
			const namejson = `${last}.json`;
			if (last in obj && "index.js" in obj[last]) {
				return {url: pathName + "/index.js", obj: obj[last]["index.js"]};
			}
			if (namejs in obj) {
				return {url: pathName + ".js", obj: obj[namejs]};
			}
			if (namejson in obj) {
				return {url: pathName + ".json", obj: obj[namejson]};
			}
			return undefined;
		}
		return {url: pathName, obj: getPath(pack.files, pathName)};
	}
	//the core resolver, which returns an object with the resource path, and the cache obj
	function resolveCore(pack, src, relativeTo) {
		const url = parseUrl(src, relativeTo);
		return resolvePath(pack, url);
	}
	//the core requirer
	function requireCore(src, relativeTo, pack = cache) {
		if (isPackage(src)) {
			//swap filesystem
			const [packname, ...rest] = src.split("/");
			const file = rest.join("/");
			pack = cache.packages[packname];
			if (!pack || !pack.files) {
				throw new Error(`package "${packname}" required without being preloaded`);
			}
			relativeTo = "/";
			src = file || pack.entry;
		}

		const data = resolveCore(pack, src, relativeTo);
		if (!data.obj) {
			throw new Error(`${data.url} was required without being preloaded.`);
		}

		if (!data.obj[sModule]) {
			if (data.url.endsWith(".js")) {
				data.obj[sModule] = {};
				execute(data.obj[sCode], data.url, data.obj, pack);
			} else if (data.url.endsWith(".json")) {
				data.obj[sModule] = JSON.parse(data.obj[sCode]);
			}
		}
		return data.obj[sModule];
	}
	function preloadPackage(name, set) {
		set.add(name);
		const pack = cache.packages[name] = cache.packages[name] || {name};
		if (pack[sPromise]) {
			return pack[sPromise];
		}

		const promise = new Promise((resolve, reject) => {
			load(`${cache.outputDir}preload_modules/${name}.json`).then((response) => {
				if (!response.ok) {
					reject(response.statusText);
				} else {
					response.json().then((data) => {
						merge(pack, data);
						preloadAll(pack[sDeps], null, set).then(resolve);
					});
				}
			});
		});
		const promises = [promise, preloadAll(pack[sDeps], null, set)];
		return pack[sPromise] = Promise.all(promises);
	}
	function preloadJSON(src, set) {
		set.add(src);
		const obj = addPath(cache.files, src);
		if (obj[sPromise]) {
			return obj[sPromise];
		}

		return new Promise((resolve, reject) => {
			load(src).then((response) => {
				if (!response.ok) {
					reject(response.statusText);
				} else {
					response.text().then((text) => {
						obj[sCode] = text;
						obj[sModule] = JSON.parse(text);
						resolve();
					});
				}
			});
		});
	}
	function preloadJs(src, set) {
		set.add(src);
		const obj = addPath(cache.files, src);
		if (obj[sPromise]) {
			return obj[sPromise];
		}

		const promise = new Promise((resolve, reject) => {
			load(src).then((response) => {
				if (!response.ok) {
					reject(response.statusText);
				} else {
					response.text().then((text) => {
						obj[sCode] = text;
						const deps = getDependencies(text);
						preloadAll(deps, src, set).then(resolve);
					});
				}
			});
		});
		const promises = [promise, preloadAll(obj[sDeps], src, set)];
		return obj[sPromise] = Promise.all(promises);
	}
	function preloadCss(src, set) {
		set.add(src);
		if (document.querySelectorAll("link[href='" + src + "']").length) {
			return true;
		}
		return new Promise((resolve, reject) => {
			const link = document.createElement("link");
			link.rel = "stylesheet";
			link.type = "text/css";
			link.href = src;
			document.head.appendChild(link);
			link.onload = () => {
				resolve(true);
			};
			link.onerror = (event) => {
				reject(event);
			};
		});
	}
	//the core preloader
	function preloadCore(src, relativeTo, set = new Set()) {
		//preload a package
		if (isPackage(src)) {
			const name = src.split("/")[0];
			if (set.has(name)) return Promise.resolve();
			return preloadPackage(name, set);
		}

		const url = parseUrl(src, relativeTo);
		if (set.has(url)) return Promise.resolve();
		if (src.endsWith(".json")) {
			return preloadJSON(url, set);
		}
		if (src.endsWith(".js")) {
			return preloadJs(url, set);
		}
		if (src.endsWith(".css")) {
			return preloadCss(src, set);
		}
		throw new Error("non-module preloads must specify file type");
	}
	//call preloadCore on all, return promise.all
	function preloadAll(array, relativeTo, set) {
		if (!array) {
			return Promise.resolve();
		}
		const promises = array.map((item) => preloadCore(item, relativeTo, set));
		return Promise.all(promises);
	}
	//you can use this to pre-define a virtual filesystem with dependencies
	//doing so can serve as an optimization, as you can begin fetching dependencies
	//while fetching the module itself instead of needing to wait for it
	const define = (input) => {
		merge(cache, input);
	};
	//external main loader, which exports to preload.main
	//this loads dependencies and begins execution, it does not return a value
	const main = (src) => {
		//preload everything, then execute the main module
		//this will then execute its dependencies as it comes to them
		preloadCore(src, this.location.pathname).then(() => {
			//call the worker function, with relativeTo set to current location
			requireCore(src, this.location.pathname);
		});
	};
	//external require function, which exports to preload.require
	//this behaves differently than the module requirer; it returns
	//a promise which resolves to the module's exports
	const require = (src) => {
		return preloadCore(src, this.location.pathname).then(() => {
			return requireCore(src, this.location.pathname);
		});
	};
	this.preload = {define, main, require};
})();

