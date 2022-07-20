var Dockerode = require('dockerode');
const compose = require('docker-compose');
const axios = require('axios');
const writeYamlFile = require('write-yaml-file');
const readYamlFile = require('read-yaml-file');
var generator = require('generate-password');
var path = require('path');
const fs = require("fs");
var db = require("../data/db.js");

var docker = new Dockerode();

// Proto-DB (please don't crash mr server)
const roboPasswords = { }

const filterOptions = {
	all: true,
	filters: {
		label: ["com.docker.compose.service=vnc"]
	}
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
	if (pwi >= 0) {
		environment[pwi] = passwordEnv;
	} else {
		environment.push(passwordEnv);
	}
	// Now set the reference
	composeData.services.vnc.environment = environment
	return passwordToken
}

const generateCompose = async (id) => {
	const yamlPath =
		(process.env.NET === "local") ? "local" : "macvlan";

	// Read in the template
	const composeFile = path.join(__dirname, `${yamlPath}/${id}.yaml`);
	const yamlData = await readYamlFile(composeFile);

	// Set the temporary container password
	roboPasswords[id] = setPassword(yamlData);
	console.log(roboPasswords)

	// const yamlData = await readYamlFile(`${__dirname}/robotont-ip.yaml`);
	console.log(yamlData.services.vnc.environment)

	// Write out the compose file to be used for a session

	const tempComposeDir = path.join(__dirname, `${yamlPath}/temp/${id}`);
	if (!fs.existsSync(tempComposeDir)){
		fs.mkdirSync(tempComposeDir);
	}

	const tempFile = path.join(__dirname, `${yamlPath}/temp/${id}/docker-compose.yaml`);
	return writeYamlFile(tempFile, yamlData);
}

const generateUrl = (id) => {
	const vncUrl = new URL(`http:/localhost/novnc/vnc.html`); // hostname excluded in return
	vncUrl.searchParams.append("autoconnect", "true");
  vncUrl.searchParams.append("resize", "remote");
	vncUrl.searchParams.append("password", roboPasswords[id]);
	// order for the following matters
	vncUrl.searchParams.append("path", "novnc");
	vncUrl.href = vncUrl.href.concat(`?token=${id}`);
	return vncUrl.pathname + vncUrl.search
}

const getUrl = (req, res) => {
	const { id } = req.params;
	const vncUrl = generateUrl(id);
	res.json({ link: vncUrl });
}


const assignContainer = (req, res) => {
	const { user, user_booking } = res.locals;
	
	// In case admin tries to get a container without an active booking
	if (user_booking === undefined) {
		return res.status(403).send('No active sessions available')
	}
	console.log(user_booking, user)
	let now = new Date();

	// TODO: differentiate between simulation and physical robot containers
	// Check first if there is an already active session
	db('inventory')
		.where({ user: user.sub })
		.andWhere('end_time', '>', now.toISOString())
		.then((inv) => {
		if (inv.length) {
			res.json(inv[0])
			// res.status(400).json('You already have an assigned container')
		} else {		
			// Get the first entry that is free (booking expired or user null)
			db('inventory').first()
				.where({
					status: true,
				})
				.andWhere({ user: null })
				.orWhere('end_time', '<', now.toISOString(),)
				.then(item => {
					// Update the user column with the respective user ID coming from the JWT token
					if (item) {
						const bookingData = {
							'user': user.sub,
							'end_time': user_booking.end
						}
						db('inventory')
							.update(bookingData)
							.where('id', item["id"])
							.then(id => {
								res.json({ ...item, ...bookingData })
							})
					} else {
						res.status(500).send('No free inventory available')
					}
				})
		}
	})
}

const listContainers = (req, res) => {
	db('inventory')
		.where({ id: 11 })
		.update({
			"status": false,
			"user": 1
		}).then(console.log("updated"))
	
	db('inventory').where({ id: 11 }).then(item => {
		res.json(item)
	})
		
	//db('inventory').where('id', 11)
	// console.log()
	
	//console.log(req);
	// const id = res.locals.user.sub;
	// // res.json({"user": id}) // req.user
	// // return
	// const options = {
	// 	all: true,
	// 	filters: {
	// 		ancestor: ["robotont:base"],
	// 	}
	// }
	// docker.listContainers(options, function(err, containers) {
	// 	res.json(containers)
	// });
}

const startContainer = (req, res) => {
	const { id } = req.params;  // robo-{x}
	
	// GIVING THE USER ACCESS TO STARTING THEIR OWN CONTAINER
	// Two states for the container - exited or stopped
	// If exited, we need to start from compose, if it is just stopped, we need to simply start it
	// Try to start it, if error -> means it doesn't exist and we need to build it from compose

	const container = docker.getContainer(id);
	//res.json(container)
	container.start(async (err, data) => {
		if (!err) {
			// res.json("Container started successfully")
			// Unchanged, but return the same expected format
			res.json({
				path: generateUrl(id)
			})
		} else if (err.statusCode === 404) {
			// proceed with compose
			await startFromCompose(id, res);
		} else {
			res.json(err)
		}
	})
}

const startFromCompose = async (id, res) => {
	console.log(`Started ${id} from compose`)
	const yamlPath =
		(process.env.NET === "local") ? "local" : "macvlan";

	// cannot run multiple container in the same directory through docker-compose, so create separate ones
	const composeDir = path.join(__dirname, `${yamlPath}/temp/${id}`)

	if (!fs.existsSync(composeDir)) {
		console.log("Could not find temp dir")
	}

	await generateCompose(id);
	const options = { cwd: composeDir, composeOptions: ["--file", `docker-compose.yaml`], log: true }

	compose.upAll(options).then(
		() => {
			// await new Promise(resolve => setTimeout(resolve, 10000)); // Artificial buffer
			const targetUrl = generateUrl(id);
			db('inventory')
				.returning('end_time')
				.where({ slug: id })
				.update({
					"vnc_uri": targetUrl,
				}).then((item) => {
					// If the user isn't an admin, set container expiration
					const { user } = res.locals;
					if (user.is_administrator !== true) {
						const expiry = item[0].end_time;
						let now = new Date();
						let end = new Date(expiry);
						setTimeout(() => {
							docker.getContainer(id).stop({t: 1});
							console.log(`${id} stopped`)
						}, end - now)
					}
				})	
			res.json({
				path: targetUrl
			})
		},
		(err) => {
			console.log("Error in starting the container via the compose file", err.message)
		}
	)
}

const stopContainer = (req, res) => {
	const { id } = req.params;
	
	const container = docker.getContainer(id);
	container.stop({t: 1}, (err, data) => {
		if (!err) {
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
};

const removeContainer = (req, res) => {
	const { id } = req.params;

	const container = docker.getContainer(id);
	container.remove((err, data) => {
		if (!err) {
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
};

const restartContainer = (req, res) => {
	const { id } = req.params;

	const container = docker.getContainer(id);
	container.restart({t: 1}, (err, data) => {
		if (!err) {
			res.json(data)
		} else {
			res.send("whoopsie")
		}
	})
};

const inspectContainer = async (req, res) => {
	const { id } = req.params;

	const container = docker.getContainer(id);
	container.inspect((err, data) => {
		if (!err) {
			const { Id, State, Config, NetworkSettings, Extrahosts } = data
			res.json({
				Id,
				State,
				Config,
				NetworkSettings,
				Extrahosts
			})
		} else if (err.statusCode === 404) {
			res.status(404).send('no such container')
		} else {
			res.status(502).send('Could not connect to the Docker socket')
		}
	})
}

const inspectStats = async (req, res) => {
	const { id } = req.params;
	const container = docker.getContainer(id);
	// Fetch both inspect and stats results, return conspectus
	
	// This one takes over a second for some reason, probably have to add in a separate endpoint due to it
	// console.time("Stats")
	const Stats = await container.stats({stream: false}).catch(e => null);
	// console.timeEnd("Stats")
	container.inspect((err, data) => {
		if (!err) {
			const { Id, State, Config, NetworkSettings, Extrahosts } = data
			
			const cpu_percent = calculate_cpu_percent(Stats);
			res.json({
				Id,
				State,
				Config,
				NetworkSettings,
				Extrahosts,
				cpu_percent
			})
		} else if (err.statusCode === 404) {
			res.status(404).send('no such container')
		} else {
			res.status(502).send('Could not connect to the Docker socket')
		}
	})
}

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


module.exports = {
	list: listContainers,
	start: startContainer,
	stop: stopContainer,
	restart: restartContainer,
	connect: getUrl,
	remove: removeContainer,
	inspect: inspectContainer,
	stats: inspectStats,
	assign: assignContainer
}