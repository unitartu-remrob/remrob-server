var Dockerode = require('dockerode');
const dockerCompose = require('docker-compose');
const writeYamlFile = require('write-yaml-file');
const readYamlFile = require('read-yaml-file');
var generator = require('generate-password');
var path = require('path');
const fs = require("fs");
var db = require("../data/db.js");

const SessionCompose = require('./session-compose.js');
let sessionComposer = null;

// ----------------------------------------------------------------
// Initialize Dockerode (the interface to the Docker Engine API)
// ----------------------------------------------------------------
var docker = new Dockerode();
// ----------------------------------------------------------------

const startContainer = (req, res) => {
	const { id: containerId } = req.params;  // robo-{x}
	const { user, user_booking, authHeader } = res.locals;
	const { fresh } = req.query;

	// A booking contains the information about whether the container is a simulation or not, admin needs to specify in the query
	const isAdmin = (user.is_administrator === true)
	const is_simulation = (user_booking && !isAdmin) ? user_booking.is_simulation : (req.query.is_simulation === 'true')

	const composeParams = {
		containerId,
		authHeader,
		isAdmin,
		userId: user.sub, 						// what if there's no user?
		userBooking: user_booking,
		useBaseImage: (fresh === 'true'),
		is_simulation: is_simulation
	}

	// global session orchestrator
	sessionComposer = new SessionCompose(composeParams);

	// GIVING THE USER ACCESS TO STARTING THEIR OWN CONTAINER
	// Two states for the container - exited or stopped
	// If exited, we need to start from compose, if it is just stopped, we need to simply start it
	// Try to start it, if error -> means it doesn't exist and we need to build it from compose

	const container = docker.getContainer(containerId);
	container.start(async (err, data) => {
		if (!err) {
			const { inventoryTable, containerId } = sessionComposer;
			db(inventoryTable).first()
				.where({
					slug: containerId
				})
				.select('vnc_uri').then(item => {
					if (item) {
						res.json({
							path: item.vnc_uri
						})
					} else {
						res.json("Invalid container ID")
					}
				})
		} else if (err.statusCode === 404) { // code 404 means container does not exist, and we need to build it from compose
			// TODO: check if the id is valid, otherwise gonna crash
			await startFromCompose(res);
		} else {
			res.json(err)
		}
	})
}

const startFromCompose = async (res) => {

	const { containerId, isAdmin, sessionType, inventoryTable } = sessionComposer;
	console.log(`Starting ${containerId} from compose...`)

	// Check whether to use a saved image or start fresh, if no image available start fresh

	// cannot run multiple container in the same directory through docker-compose, so create separate ones
	const composeDir = path.join(__dirname, `${sessionType}/temp/${containerId}`)

	if (!fs.existsSync(composeDir)){
		fs.mkdirSync(composeDir);
	}

	const tokenPw = await generateCompose(); // generate compose file and collect the token password for creating the vnc connection link
	const options = { cwd: composeDir, composeOptions: ["--file", `docker-compose.yaml`], log: true }

	dockerCompose.upAll(options).then(
		() => {
			const targetUrl = generateUrl(tokenPw);
			db(inventoryTable)
				.returning('end_time')
				.where({ slug: containerId })
				.update({
					"vnc_uri": targetUrl,
				}).then((item) => {
					// If the user isn't an admin, set container expiration
					if (!isAdmin) {
						const expiry = item[0].end_time;
						let now = new Date();
						let end = new Date(expiry);
						setTimeout(() => {
							killContainer(containerId)
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

const generateCompose = async () => {
	const { containerId, sessionType } = sessionComposer;
	// Read in the template
	const composeFileHandle = path.join(__dirname, `${sessionType}/${containerId}.yaml`);
	const composeData = await readYamlFile(composeFileHandle);

	// Create the token, add it to the yaml as a side effect
	const passwordToken = await setEnvironment(composeData);
	console.log(composeData.services.vnc.image)

	// Write out the compose file to be used for a session
	const tempComposeDir = path.join(__dirname, `${sessionType}/temp/${containerId}`);
	if (!fs.existsSync(tempComposeDir)){
		fs.mkdirSync(tempComposeDir);
	}

	const tempFile = path.join(__dirname, `${sessionType}/temp/${containerId}/docker-compose.yaml`);
	await writeYamlFile(tempFile, composeData);
	return passwordToken
}

const generateUrl = (tokenPw) => {
	const vncUrl = new URL(`http:/localhost/novnc/vnc.html`); // hostname excluded in return
	vncUrl.searchParams.append("autoconnect", "true");
  vncUrl.searchParams.append("resize", "remote");
	vncUrl.searchParams.append("password", tokenPw);
	// order for the following matters
	vncUrl.searchParams.append("path", "novnc");
	vncUrl.href = vncUrl.href.concat(`?token=${sessionComposer.containerId}`);
	return vncUrl.pathname + vncUrl.search
}


const setEnvironment = async (composeData) => {
	// Generate the password for vnc and push it into the compose data
	const { services: { vnc: { environment } } } = composeData;
	const { containerId, userId, is_simulation, useBaseImage } = sessionComposer;

	// Generate a VNC password token
	const passwordToken = generator.generate({
		length: 20,
		numbers: true
	});
	const passwordEnv = `PASSWORD=${passwordToken}`

	// In case the template already has something set for password, replace it instead of adding a new one
	const pwi = environment.findIndex(env => env.includes("PASSWORD"));
	if (pwi >= 0) {
		environment[pwi] = passwordEnv;
	} else {
		environment.push(passwordEnv);
	}
	
	// Specify the robot cell, for simulation this will be null
	if (!is_simulation) { 
		const { cell } = await db('inventory').first().where({ slug: containerId }).select(['cell']);
		environment.push(`ROBOT_CELL=${cell}`)
	}

	const { volumes, volEnv } = await sessionComposer.setVolumeMounts(composeData);
	composeData.services.vnc.volumes = volumes;
	composeData.services.vnc.environment = [...environment, ...volEnv];

	// Set the user-specific image if required
	return new Promise((resolve, reject) => {
		if (!useBaseImage) {
			const name = `robotont:${userId}`
			const image = docker.getImage(name);
			image.inspect((err, data) => {
				// If no error, means the image exists, else retain base image
				if (!err) {
					composeData.services.vnc.image = name
				}
			})
		}
		resolve(passwordToken);
	})
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
	container.commit({repo: "robotont", tag: user.sub}, (err, data) => {
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