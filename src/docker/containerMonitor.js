import ping from 'ping';

import { USER_TABLE, SIMTAINER_INVENTORY_TABLE, ROBOT_INVENTORY_TABLE } from '../constants.js';
import config from 'config';
import db from '../data/db.js';
import { getInventoryTable } from '../session/inventory.js';
import docker from './index.js';
import { inspectContainer } from './containerManager.js';
import calculateCpuPercentage from '../util/calculateCpuPercentage.js';

const subnet = config.get('RobotNetworkSubnetPrefix');

const CONTAINER_MONITOR_INTERVAL = 1500;
const ROBOT_MONITOR_INTERVAL = 1500;

const getInventoryStock = async (inventoryTableName) => {
	const stockItemId =
		inventoryTableName === SIMTAINER_INVENTORY_TABLE ? 'container_id' : 'robot_id';

	return await db(inventoryTableName).select().orderBy(stockItemId, 'asc');
};

const getContainerInfo = async (containerId) => {
	const container = docker.getContainer(containerId);
	const containerStats = await container.stats({ stream: false }).catch(() => null);

	const inspectData = await inspectContainer(containerId).catch(() => null);

	const cpu_percent = calculateCpuPercentage(containerStats);

	return {
		...inspectData,
		cpu_percent,
	};
};

const containerMonitor = async (tableId, ws) => {
	const inventoryStock = await getInventoryStock(tableId);

	const containerInfoCalls = [];
	const hostsCalls = [];
	const usersCalls = [];

	inventoryStock.forEach(({ slug, robot_id, user }) => {
		containerInfoCalls.push(getContainerInfo(slug));

		if (tableId === ROBOT_INVENTORY_TABLE) {
			const host = `${subnet}.${robot_id}`;

			hostsCalls.push(ping.promise.probe(host, { timeout: 1 }));
		}

		usersCalls.push(db(USER_TABLE).where({ id: user }).first());
	});

	const results = await Promise.allSettled(containerInfoCalls);
	const pings = await Promise.allSettled(hostsCalls);
	const userInfo = await Promise.allSettled(usersCalls);

	inventoryStock.forEach(({ robot_id, slug, end_time, vnc_uri, open_to_public }, index) => {
		// add slug ID to know which container was rejected
		results[index]['slug'] = slug;
		results[index]['robot_id'] = robot_id;
		results[index]['end_time'] = end_time;
		results[index]['vnc_uri'] = vnc_uri;

		if (tableId == SIMTAINER_INVENTORY_TABLE) {
			results[index]['open_to_public'] = open_to_public;
		}

		if (userInfo[index].value != undefined) {
			const { first_name, last_name } = userInfo[index].value;
			results[index]['user'] = `${first_name} ${last_name}`;
		}

		if (tableId == ROBOT_INVENTORY_TABLE) {
			results[index]['robot_status'] = pings[index].value.alive;
		}		
	});

	ws.send(JSON.stringify(results));
};

const liveStats = async (ws, req) => {
	const { version } = req.params;
	console.log('New socket from client');

	const tableId = getInventoryTable(version === 'simulation');

	const pollInterval = setInterval(containerMonitor, CONTAINER_MONITOR_INTERVAL, tableId, ws);

	ws.on('message', () => {
		// force update
		containerMonitor(tableId, ws);
	});

	ws.on('close', () => {
		clearInterval(pollInterval);
		console.log('they left...  what a pity');
	});

	// send first data immediately
	await containerMonitor(tableId, ws);
};

const robotMonitor = (ws, req) => {
	const { id } = req.params;

	const host = `${subnet}.${id}`;

	const pollInterval = setInterval(() => {
		ping.sys.probe(host, function (isAlive) {
			ws.send(JSON.stringify(isAlive));
		});
	}, ROBOT_MONITOR_INTERVAL);

	ws.on('close', () => {
		clearInterval(pollInterval);
	});
};

export { liveStats, robotMonitor };
