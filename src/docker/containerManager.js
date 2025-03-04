import docker from './index.js';
import dockerCompose from 'docker-compose';

import { composeContainerConfig } from '../session/sessionComposer.js';

const startContainer = async (composeParams) => {
	const id = composeParams.containerId;
	const container = docker.getContainer(id);

	try {
		await container.start();
	} catch (err) {
		if (err.statusCode === 404) {
			// container does not exist -> start it via compose
			const containerConfig = await composeContainerConfig(composeParams);

			try {
				await dockerCompose.upAll(containerConfig);
			} catch (err) {
				console.log('Error in starting the container via the compose file;', err);
				throw err;
			}
		} else if (err.statusCode === 304) {
			// container already running
		} else {
			console.log('Error in starting the container;', err);
			throw err;
		}
	}
};

const inspectContainer = async (id) => {
	const container = docker.getContainer(id);

	const { State, Config, NetworkSettings } = await container.inspect();

	return {
		status: State.Status,
		createdAt: State.StartedAt,
		ipAddress: Object.values(NetworkSettings.Networks)[0].IPAddress,
		image: Config.Image,
	};
};

const getContainerStats = async (id) => {
	const container = docker.getContainer(id);

	return await container.stats({ stream: false });
};

const stopContainer = async (id) => {
	const container = docker.getContainer(id);

	return await container.stop({ t: 3 });
};

const removeContainer = async (id) => {
	const container = docker.getContainer(id);

	return await container.remove();
};

const killContainer = async (id) => {
	const container = docker.getContainer(id);

	try {
		await container.kill();
		await container.remove();
	} catch (err) {
		if (err.statusCode === 409) {
			// container is already stopped -> remove it
			try {
				await container.remove();
			} catch (err) {
				// container already removed
			}
		} else if (err.statusCode === 404) {
			// container does not exist
		} else {
			throw err;
		}
	}
};

export {
	startContainer,
	inspectContainer,
	getContainerStats,
	stopContainer,
	removeContainer,
	killContainer,
};
