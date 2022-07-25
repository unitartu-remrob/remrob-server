const  { docker, calculate_cpu_percent } = require('./container-master.js')
// const axios = require('axios');
var db = require("../data/db.js");

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
		inv.forEach(({ slug, vnc_uri }) => {
			calls.push(
				getStats(slug, vnc_uri)
			);
		})

		const results = await Promise.allSettled(calls);
		
		inv.forEach(({ slug }, index) => {
			// Add slug ID to know which container was rejected
			results[index]["slug"] = slug;
		})
		ws.send(JSON.stringify(results))
}

const liveStats = async (ws, req) => {
	const { version } = req.params;
	const table_id =
		(version == "simulation")
			? "simulation_containers"
			: "inventory"; // anything else defaults to physbots

  console.log("New socket from client")

	const pollInterval = setInterval(containerMonitor, 1500, table_id, ws)

	ws.on('message', msg => {
		// Force update
		containerMonitor(table_id, ws)
  })
	
	ws.on('close', something => {
		clearInterval(pollInterval);
		console.log("they left...  what a pity")
	})
}

module.exports = {
	liveStats
}