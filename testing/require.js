(() => {
	//map of all loaded modules
	const loaded = {};
	//Regex for getting preload directives
	const preloadRegex = /\/\/#[ \t]+preload[ \t]+(\S*)/g;
	//call to parse urls
	function parseUrl(src, relativeTo) {
		//create a url out of the source
		let url = new URL(src, relativeTo);
		//return the href
		return url.href;
	}
	//reads the source for requirements
	function getRequirements(code) {
		let reqs = [];
		let match;
		while ((match = preloadRegex.exec(code))) {
			let [, req] = match;
			reqs.push(req);
		}
		return reqs;
	}
	//executes the code, and returns the result of module.exports
	function execute(code, src) {
		let module = {};
		Object.defineProperty(module, "exports", {
			get: () => loaded[src].module,
			set: (value) => loaded[src].module = value,
		});
		//let the debugger know what name to use in debug statements
		code += `//# sourceURL=${src}`;
		//construct and execute the function
		let func = new Function("module", "require", code);
		func(module, (req) => requireCore(req, src));
	}
	//call to load the resource
	function load(src) {
		//return a promise which resolves with the module exports
		return new Promise((resolve, reject) => {
			let request = new XMLHttpRequest();
			request.open("get", src);
			request.onload = () => {
				if (request.status === 200)
					resolve(request.response);
				else
					reject(Error("Module failed to load, status: " + request.statusText));
			};
			request.onerror = () => {
				reject(Error("Module failed to load due to network error"));
			};
			request.send();
		});
	}
	//require a javascript file
	function requireJs(src) {
		//if it isn't in the loaded map, it wasn't preloaded. throw an error
		if (!loaded[src])
			throw new Error(`${src} was required without being preloaded.`);
		//if we don't already have the module, execute the code
		if (!loaded[src].module)
			execute(loaded[src].code, src);
		return loaded[src].module;
	}
	//require a json file
	function requireJSON(src) {
		//if it isn't in the loaded map, it wasn't preloaded. throw an error
		if (!loaded[src])
			throw new Error(`${src} was required without being preloaded.`);
		//if we haven't already parsed it, parse JSON
		if (!loaded[src].module)
			loaded[src].module = JSON.parse(loaded[src].code);
		return loaded[src].module;
	}
	//the recursive inner require call
	//this is what does the work inside modules
	function requireCore(src, relativeTo) {
		//parse the url
		let url = parseUrl(src, relativeTo);
		//check to see if this is css
		if (src.includes(".css"))
			return null;
		//check to see if this is json
		if (src.includes(".json"))
			return requireJSON(url);
		//load the javascript
		return requireJs(url);
	}
	//preload a javascript file
	function preloadJs(src, set) {
		//if we've already checked this file, return true
		if (set.has(src))
			return true;
		//if we already have a promise, return the promise
		if (loaded[src])
			return loaded[src].promise;
		//otherwise, preload with a promise which resolves when the module is loaded
		loaded[src] = {
			promise: new Promise((resolve, reject) => {
				//load the module
				load(src).then((result) => {
					//now we have the text, add it to the loaded module
					loaded[src].code = result;
					//get the requirements
					let reqs = getRequirements(result);
					//preload with relative path on all requirements, and resolve when done
					Promise.all(reqs.map((req) => preloadCore(req, src, set))).then(() => {
						resolve(true);
					}).catch((error) => {
						reject(error);
					});
				}).catch((error) => {
					reject(error);
				});
			}),
		};
		//add this file to the stuff we've already seen
		set.add(src);
		//return the promise
		return loaded[src].promise;
	}
	//preload a json file
	function preloadJSON(src, set) {
		//if we've already checked this file, return true
		if (set.has(src))
			return true;
		//if we already have a promise, return the promise
		if (loaded[src])
			return loaded[src].promise;
		//otherwise, preload with a promise which resolves when the json is loaded
		loaded[src] = {
			promise: new Promise((resolve, reject) => {
				load(src).then((result) => {
					//add text to the loaded module
					loaded[src].code = result;
					resolve(true);
				}).catch((error) => {
					reject(error);
				});
			}),
		};
		//add this file to the stuff we've already seen
		set.add(src);
		//return the promise
		return loaded[src].promise;
	}
	//preload a css file
	function preloadCss(src) {
		//if a link with that name exists, return true
		if (document.querySelectorAll("link[href='" + src + "']").length)
			return true;
		//otherwise, return a promise that resolves when the css is loaded
		return new Promise((resolve, reject) => {
			let link = document.createElement("link");
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
	//core preloader
	function preloadCore(src, relativeTo, set) {
		//parse the url
		let url = parseUrl(src, relativeTo);
		//check to see if we are loading css
		if (src.includes(".css"))
			return preloadCss(url);
		//check to see if we are loading json
		if (src.includes(".json"))
			return preloadJSON(url, set);
		//otherwise load javascript
		return preloadJs(url, set);
	}
	//external main loader, which exports to window.main
	const main = function(src) {
		//preload everything, then execute the main module
		//this will then execute its dependencies as it comes to them
		preloadCore(src, this.location.href, new Set()).then(() => {
			//call the worker function, with relativeTo set to origin
			requireCore(src, this.location.href);
		});
	};
	//external require function, which exports to window.require
	//this behaves differently than the module requirer, it returns
	//a promise which resolves to the module's exports
	const require = function(src) {
		return preloadCore(src, this.location.href, new Set()).then(() => {
			return requireCore(src, this.location.href);
		});
	};
	//export as unmodifyable props on window
	Object.defineProperty(this, "main", {get: () => main});
	Object.defineProperty(this, "require", {get: () => require});
})();

