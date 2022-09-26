var Dockerode = require('dockerode');
const compose = require('docker-compose');
const axios = require('axios');
const writeYamlFile = require('write-yaml-file');
const readYamlFile = require('read-yaml-file');
var generator = require('generate-password');
var path = require('path');
const fs = require("fs");
var db = require("../data/db.js");
const { v4: uuidv4 } = require('uuid');

var docker = new Dockerode();

let HEADERS;

const filterOptions = {
	all: true,
	filters: {
		label: ["com.docker.compose.service=vnc"]
	}
}

const setGitRepository = async (composeData, user) => {
	const { services: { vnc: { environment, volumes } } } = composeData;
	
	const user_data = await db('user').first().where({ id: user.sub }).select(['first_name', 'last_name']);
	let { first_name, last_name } = user_data;
	if (first_name === null) {
		first_name = "first_name"
	}
	if (last_name === null) {
		last_name = "last_name"
	}
	first_name = first_name.replace(/[\x00-\x08\x0E-\x1F\x7F-\uFFFF]/g, '')
	last_name = last_name.replace(/[\x00-\x08\x0E-\x1F\x7F-\uFFFF]/g, '')

	const repoContainerName = `${first_name}-${last_name}`;
	const repoHostName = `${first_name}-${last_name}-${user.sub}`;

	// Authentication token for pushing from inside the container:
	const git_auth_token = uuidv4();

	// Update the db with the session token and repo name
	db('user').update({ git_token: git_auth_token, user_repo: repoHostName }).where({ id: user.sub }).then(res => {
	}).catch(e => console.log(e)); // tralalalalalalalalalalalalalalalalala

	// Push to the compose data:
	environment.push(`GIT_PAT=${git_auth_token}`)
	volumes.push(`${process.env.REPOS_ROOT}/${repoHostName}:/home/kasutaja/${repoContainerName}`) // lil bit of hard coding
	
	// Pass back the reference:
	composeData.services.vnc.volumes = volumes
	composeData.services.vnc.environment = environment

	// Create the repo, if it already existed - nothing happens
	await axios.get(`${process.env.DB_SERVER}/check_user`, { params: { 'user_name': repoHostName }, headers: HEADERS }).catch(e => console.log(e));
	
	// Now we need to pull the repository we just created
	// TODO: don't attempt clone if it already exists
	await axios.get(`${process.env.DB_SERVER}/clone_jwt`, { params: { 'user_name': repoHostName }, headers: HEADERS }).catch(e => console.log(e));

	return composeData
}

const pushToRepo = (token) => {
	axios.get(`${process.env.DB_SERVER}/commit_push`, { params: { 'token': token } }).then(res => {
		
	});
}

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
	// lalalalalalalalaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
	environment.push(`ROBOT_CELL=${robot_cell}`)

	composeData = await setGitRepository(composeData, user);
	composeData.services.vnc.environment = environment // git has been set, top of with vnc pw token

	return new Promise((resolve, reject) => {
		if (!user.fresh) {
			const name = `robotont:${user.sub}`
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
	// Set the user image if required
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

const listContainers = (req, res) => {
	// Testing endpoint
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
			// await new Promise(resolve => setTimeout(resolve, 10000)); // Artificial buffer
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
						}, end - now - 6000)
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
	const container = docker.getContainer(id);
	console.log(`${id} stopping`)
	container.stop({t: 2}, (err, data) => {
		if (!err) {
			container.remove()
			console.log(`${id} purged`)
		}
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
	list: listContainers,
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