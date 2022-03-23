var Dockerode = require('dockerode');
const compose = require('docker-compose');
const writeYamlFile = require('write-yaml-file');
const readYamlFile = require('read-yaml-file');
// const YAML = require('js-yaml');
var generator = require('generate-password');
var path = require('path');
const fs = require("fs");

var docker = new Dockerode();

const activePasswords = {
	"robo-1": '',
	"robo-2": '',
	"robo-3": '',
	"robo-4": '',
}

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
	const composeDir = path.join(__dirname, `robotont-${id}/`);
	const yamlData = await readYamlFile(`${composeDir}/docker-compose.yaml`);
	// Set the temporary container password
	activePasswords[`robo-${id}`] = setPassword(yamlData);

	// const yamlData = await readYamlFile(`${__dirname}/robotont-ip.yaml`);
	console.log(yamlData.services.vnc.environment)

	return writeYamlFile(`${composeDir}/robo-${id}.yaml`, yamlData);
}

const generateUrl = (id) => {
	const vncUrl = new URL("http://localhost/novnc/vnc.html");
	vncUrl.searchParams.append("autoconnect", "true");
  vncUrl.searchParams.append("resize", "remote");
	vncUrl.searchParams.append("password", activePasswords[`robo-${id}`]);
	// order for the following matters
	vncUrl.searchParams.append("path", "novnc");
	vncUrl.href = vncUrl.href.concat(`?token=robo${id}`);
	return vncUrl
}

const testRoute = (async (req, res) => {
	await generateCompose("1");
	res.json("Success")
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

const startContainer = (async (req, res) => {
	const { id } = req.params;
	const composeDir = path.join(__dirname, `robotont-${id}/`)
	if (!fs.existsSync(composeDir)) {
		res.json("Not available")
  } else {
		await generateCompose(id);
		const options = { cwd: composeDir, composeOptions: ["--file", `robo-${id}.yaml`], log: true }
		compose.upAll(options).then(
			() => {
				// await new Promise(resolve => setTimeout(resolve, 10000));
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
	const container = docker.getContainer(`robo-${id}`);
	container.stop({t: 5}, (err, data) => {
		if (!err) {
			// console.log(util.inspect(data))
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
});

const removeContainer = ((req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(`robo-${id}`);
	container.remove((err, data) => {
		if (!err) {
			// console.log(util.inspect(data))
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
});

const inspectContainer = ((req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(`robo-${id}`);
	container.inspect((err, data) => {
		if (!err) {
			// console.log(util.inspect(data))
			res.json(data)
		} else {
			res.send("whoopsie")
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

const containerStatus = ((req, res) => {
  res.json({hello: "world"})
})


module.exports = {
	list: listContainers,
	status: containerStatus,
	start: startContainer,
	stop: stopContainer,
	connect: getUrl,
	remove: removeContainer,
	inspect: inspectContainer,
	test: testRoute
}