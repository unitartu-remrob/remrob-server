const  { docker, calculate_cpu_percent } = require('./container-master.js')
var db = require("../data/db.js");
const ping = require('ping');

const config = require('config');
const subnet = config.get('Network.SUBNET_PREFIX');

const getInventory = async (stock) => {
	const id = (stock === "simulation_containers") ? 'container_id' : 'robot_id';
	const inv = await db.select().table(stock)
		.orderBy(id, 'asc')
	return inv
}

const getStats = async (slug, vnc_uri) => {
	const container = docker.getContainer(slug)

	const Stats = await container.stats({stream: false}).catch(e => null);
	const inspectData = await container.inspect({stream: false}).catch(e => null);

	const { State, Config, NetworkSettings, Extrahosts } = inspectData
	const cpu_percent = calculate_cpu_percent(Stats);
	return {
		State,
		Config,
		NetworkSettings,
		Extrahosts,
		cpu_percent,
		vnc_uri
	}
}

const containerMonitor = async (table_id, ws) => {
	const inv = await getInventory(table_id);
	const calls = [];
	const hosts = [];
	const users = [];
	inv.forEach(({ slug, vnc_uri, robot_id, user }) => {
		calls.push(
			getStats(slug, vnc_uri)
		);
		if (table_id === "inventory") {
			let host = `${subnet}.${robot_id}`
			hosts.push(ping.promise.probe(host, {timeout: 1}))
		}
		users.push(
			db('user').where({id: user}).first()
		)
	})

	const results = await Promise.allSettled(calls);
	const pings = await Promise.allSettled(hosts);
	const user_info = await Promise.allSettled(users);

	inv.forEach(({ robot_id, slug, user, end_time, issue }, index) => {
		// Add slug ID to know which container was rejected
		results[index]["slug"] = slug;
		results[index]["robot_id"] = robot_id;
		if (user_info[index].value != undefined) {
			const { first_name, last_name } = user_info[index].value;
			results[index]["user"] = `${first_name} ${last_name}`;
		}
		//
		if (table_id == "inventory") {
			results[index]["robot_status"] = pings[index].value.alive;
		}
		// Add booking info about the specific container
		results[index]["booking"] = {
			user, end_time, issue
		}
	})
	ws.send(JSON.stringify(results))
}

const liveStats = async (ws, req) => {
	const { version } = req.params;
	console.log("New socket from client")

	const table_id =
		(version === "simulation")
			? "simulation_containers"
			: "inventory"; // anything else defaults to physbots

	const pollInterval = setInterval(containerMonitor, 1500, table_id, ws)

	ws.on('message', msg => {
		// Force update
		containerMonitor(table_id, ws)
  })
	
	ws.on('close', something => {
		clearInterval(pollInterval);
		console.log("they left...  what a pity")
	})

	// send first data immediately
	await containerMonitor(table_id, ws)
}

const robotMonitor = (ws, req) => {
	const { id } = req.params;

	let host = `${subnet}.${id}`
	const pollInterval = setInterval(() => {
		ping.sys.probe(host, function(isAlive) {
			ws.send(JSON.stringify(isAlive))
		})
	}, 1500)
	
	ws.on('close', something => {
		clearInterval(pollInterval);
		console.log("they left...  what a pity")
	})
}

module.exports = {
	liveStats,
	robotMonitor
}