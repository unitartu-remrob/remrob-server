var Dockerode = require('dockerode');
const compose = require('docker-compose');
const writeYamlFile = require('write-yaml-file');
const readYamlFile = require('read-yaml-file');
// const YAML = require('js-yaml');
var generator = require('generate-password');
var path = require('path');
const fs = require("fs");

var docker = new Dockerode();

// Proto-DB (please don't crash mr server)
const roboPasswords = {
	"robo-1": '',
	"robo-2": '',
	"robo-3": '',
	"robo-4": '',
}

const getContainerNames = (async (req, res) => {
	// This endpoint can be used to get all the fetchable containers before sending inspect calls
	res.json({names: Object.keys(roboPasswords)});
});

const setPassword = (composeData) => {
	// Generate the password for vnc and push it into the compose data
	const {services: {vnc: {environment}}} = composeData;

	const passwordToken = generator.generate({
		length: 20,
		numbers: true
	});
	const passwordEnv = `PASSWORD=${passwordToken}`
	
	// Check if template already has something set, either replace or push new
	const pwi = environment.findIndex(env => env.includes("PASSWORD"));
	if (pwi) {
		environment[pwi] = passwordEnv;
	} else {
		environment.push(passwordEnv);
	}
	// Now set the reference
	composeData.services.vnc.environment = environment
	return passwordToken
}

const generateCompose = async (id) => {
	const composeDir = path.join(__dirname, `${id}/`);
	const yamlData = await readYamlFile(`${composeDir}/docker-compose.yaml`);
	// Set the temporary container password
	roboPasswords[id] = setPassword(yamlData);
	console.log(roboPasswords)

	// const yamlData = await readYamlFile(`${__dirname}/robotont-ip.yaml`);
	console.log(yamlData.services.vnc.environment)

	return writeYamlFile(`${composeDir}/${id}.yaml`, yamlData);
}

const generateUrl = (id) => {
	const vncUrl = new URL("http://localhost/novnc/vnc.html");
	vncUrl.searchParams.append("autoconnect", "true");
  vncUrl.searchParams.append("resize", "remote");
	vncUrl.searchParams.append("password", roboPasswords[id]);
	// order for the following matters
	vncUrl.searchParams.append("path", "novnc");
	vncUrl.href = vncUrl.href.concat(`?token=${id}`);
	return vncUrl
}

const testRoute = (async (req, res) => {
	// await generateCompose("1");
	res.json(roboPasswords)
});

const listContainers = ((req, res) => {
	const options = {
		all: true,
		filters: {
			ancestor: ["robotont:base"],
		}
	}
	docker.listContainers(options, function(err, containers) {
		res.json(containers)
	});
})

// const listAllContainers = ((req, res) => {
// 	const options = {
// 		all: true,
// 		filters: {
// 			ancestor: ["robotont:base"],
// 		}
// 	}
// 	docker.listContainers(options, function(err, containers) {
// 		res.json(containers)
// 	});
// })

const startContainer = (async (req, res) => {
	const { id } = req.params;
	const composeDir = path.join(__dirname, `${id}/`)
	if (!fs.existsSync(composeDir)) {
		res.json("Not available")
  } else {
		await generateCompose(id);
		const options = { cwd: composeDir, composeOptions: ["--file", `${id}.yaml`], log: true }
		compose.upAll(options).then(
			() => {
				// await new Promise(resolve => setTimeout(resolve, 10000)); // Artificial buffer
				const targetUrl = generateUrl(id);
				res.json({
					link: targetUrl
				})
			},
			(err) => {
				console.log("Starting container failed", err.message)
				res.json("failed")
			}
		)
	}
});

const getUrl = (req, res) => {
	const { id } = req.params;
	const vncUrl = generateUrl(id);
	res.json({link: vncUrl});
}

const stopContainer = ((req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(id);
	container.stop({t: 5}, (err, data) => {
		if (!err) {
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
});

const removeContainer = ((req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(id);
	container.remove((err, data) => {
		if (!err) {
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
});

const inspectContainer = ((req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(id);

	container.inspect((err, data) => {
		if (!err) {
			// console.log(util.inspect(data))
			const { Id, State, Config, NetworkSettings, Extrahosts } = data
			// res.json(data);
			res.json({
				Id,
				State,
				Config,
				NetworkSettings,
				Extrahosts,
			})
		} else {
			res.json(err)
		}
	})
})

const calculate_cpu_percent = (d) => {
	if (Object.keys(d.pids_stats).length === 0) {
		return 0;
	}
	const cpu_count = d["cpu_stats"]["cpu_usage"]["percpu_usage"].length;
	let cpu_percent = 0.0
	const cpu_delta = d["cpu_stats"]["cpu_usage"]["total_usage"] -
							d["precpu_stats"]["cpu_usage"]["total_usage"];
	const system_delta = d["cpu_stats"]["system_cpu_usage"] -
									d["precpu_stats"]["system_cpu_usage"];
	if (system_delta > 0.0) {
			cpu_percent = cpu_delta / system_delta * 100.0 * cpu_count;
	}
	return cpu_percent.toFixed(2);
}

const getCpuPercentage = ((req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(id);
	container.stats({stream: false}, (err, data) => {
		if (!err) {
			const cpu_percent = calculate_cpu_percent(data);
			res.json({
				cpu_percentage: cpu_percent
			})
		} else {
			res.json(err);
		}
	})
})


/**
 * Inspect
 * @param  {Object}   opts     Options (optional)
 * @param  {Function} callback Callback, if supplied will query Docker.
 * @return {Object}            ID only and only if callback isn't supplied.
 */
//  Container.prototype.inspect = function(opts, callback)


/**
 * Stop
 * @param  {Object}   opts     Container stop options, like 't' (optional)
 * @param  {Function} callback Callback
 */
//  Container.prototype.stop = function(opts, callback)


module.exports = {
	names: getContainerNames,
	list: listContainers,
	start: startContainer,
	stop: stopContainer,
	connect: getUrl,
	remove: removeContainer,
	inspect: inspectContainer,
	cpu: getCpuPercentage,
	test: testRoute
}