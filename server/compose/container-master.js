var Dockerode = require('dockerode');
const compose = require('docker-compose');
const writeYamlFile = require('write-yaml-file');
const readYamlFile = require('read-yaml-file');
var generator = require('generate-password');
var path = require('path');
const fs = require("fs");
var db = require("../data/db.js");


// ----------------------------------------------------------------
// Initialize Dockerode (the interface to the Docker Engine API)
// ----------------------------------------------------------------
var docker = new Dockerode();
// ----------------------------------------------------------------

const setEnvironment = async (composeData, user, robot_cell) => {
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
	environment.push(`ROBOT_CELL=${robot_cell}`)

	composeData.services.vnc.environment = environment

	return new Promise((resolve, reject) => {
		if (!user.fresh) {
			const name = `remrob:${user.sub}`
			const image = docker.getImage(name);
			image.inspect((err, data) => {
				// If no error, means the image exists, else retain base image
				if (!err) {
					composeData.services.vnc.image = name
				}
				resolve(passwordToken)
			})
		} else {
			resolve(passwordToken)
		}
	})
}

const generateCompose = async (id, yamlPath, user) => {
	// Read in the template
	const composeFile = path.join(__dirname, `${yamlPath}/${id}.yaml`);
	const yamlData = await readYamlFile(composeFile);

	let cell = await db('inventory').first().where({ slug: id }).select('cell')
	if (cell === undefined) {
		cell = null
	} else { cell = cell.cell} // haha what dis

	// Create the token, add it to the yaml as a side effect
	const passwordToken = await setEnvironment(yamlData, user, cell);
	console.log(yamlData.services.vnc.image)

	// Write out the compose file to be used for a session
	const tempComposeDir = path.join(__dirname, `${yamlPath}/temp/${id}`);
	if (!fs.existsSync(tempComposeDir)){
		fs.mkdirSync(tempComposeDir);
	}

	const tempFile = path.join(__dirname, `${yamlPath}/temp/${id}/docker-compose.yaml`);
	await writeYamlFile(tempFile, yamlData);
	return passwordToken
}

const generateUrl = (id, tokenPw) => {
	const vncUrl = new URL(`http:/localhost/novnc/vnc.html`); // hostname excluded in return
	vncUrl.searchParams.append("autoconnect", "true");
  vncUrl.searchParams.append("resize", "remote");
	vncUrl.searchParams.append("password", tokenPw);
	// order for the following matters
	vncUrl.searchParams.append("path", "novnc");
	vncUrl.href = vncUrl.href.concat(`?token=${id}`);
	return vncUrl.pathname + vncUrl.search
}

const startContainer = (req, res) => {
	const { id } = req.params;  // robo-{x}
	
	// GIVING THE USER ACCESS TO STARTING THEIR OWN CONTAINER
	// Two states for the container - exited or stopped
	// If exited, we need to start from compose, if it is just stopped, we need to simply start it
	// Try to start it, if error -> means it doesn't exist and we need to build it from compose
	
	const container = docker.getContainer(id);
	container.start(async (err, data) => {
		if (!err) {
			// res.json("Container started successfully")
			// Unchanged, but return the same expected format
			const { user_booking } = res.locals;
			const is_simulation = (user_booking) ? user_booking.is_simulation : (req.query.is_simulation === 'true')
			const inventoryTable = (is_simulation) ? 'simulation_containers' : 'inventory';
			db(inventoryTable).first()
				.where({ slug: id })
				.select('vnc_uri').then(item => {
					if (item) {
						res.json({
							path: item.vnc_uri
						})
					} else {
						res.json("Invalid id")
					}
				})
		} else if (err.statusCode === 404) {
			// TODO: check if the id is valid, otherwise gonna crash
			await startFromCompose(id, req, res);
		} else {
			res.json(err)
		}
	})
}

const startFromCompose = async (id, req, res) => {
	console.log(`Starting ${id} from compose...`)

	HEADERS = res.locals.authHeader; // REF for talking to flask
	// Check whether to use a saved image or start fresh, if no image available start fresh
	const { fresh } = req.query;
	const { user, user_booking } = res.locals;
	const is_admin = (user.is_administrator === true)
	user.fresh = (fresh === 'true');

	// Admin client expected to inform if the container to be started is a sim
	const is_simulation = (user_booking && !is_admin) ? user_booking.is_simulation : (req.query.is_simulation === 'true');

	const yamlPath = (is_simulation) ? "local" : "macvlan";
	const inventoryTable = (is_simulation) ? 'simulation_containers' : 'inventory';

	// cannot run multiple container in the same directory through docker-compose, so create separate ones
	const composeDir = path.join(__dirname, `${yamlPath}/temp/${id}`)

	if (!fs.existsSync(composeDir)){
		fs.mkdirSync(composeDir);
		// return res.sendStatus(404);
	}

	const tokenPw = await generateCompose(id, yamlPath, user);
	const options = { cwd: composeDir, composeOptions: ["--file", `docker-compose.yaml`], log: true }

	compose.upAll(options).then(
		() => {
			const targetUrl = generateUrl(id, tokenPw);
			db(inventoryTable)
				.returning('end_time')
				.where({ slug: id })
				.update({
					"vnc_uri": targetUrl,
				}).then((item) => {
					// If the user isn't an admin, set container expiration
					if (!is_admin) {
						const expiry = item[0].end_time;
						let now = new Date();
						let end = new Date(expiry);
						setTimeout(() => {
							killContainer(id)
						}, end - now - 3000)
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
	container.stop({t: 2}, (err, data) => {
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

const killContainer = (id) => {
	return new Promise((resolve, reject) => {
		const container = docker.getContainer(id)
		container.kill().then(res => {
			container.remove().then(res => {
				resolve(0)
			})		
		}).catch(err => {
			if (err.statusCode === 409) {
				container.remove().then(res => {
					resolve(0)
				})	
			} else {
				resolve(1)
			}
		})
	})
}

const commitContainer = (req, res) => {
	const { id } = req.params;
	const { user } = res.locals;

	const container = docker.getContainer(id);
	container.commit({repo: "remrob", tag: user.sub}, (err, data) => {
		if (!err) {
			docker.pruneImages() // Clean up old versions
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
	start: startContainer,
	stop: stopContainer,
	restart: restartContainer,
	commit: commitContainer,
	remove: removeContainer,
	inspect: inspectContainer,
	stats: inspectStats,

	docker,
	calculate_cpu_percent,
	killContainer
}