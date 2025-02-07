import { inspectContainer, getContainerStats } from '../../../docker/containerManager.js';
import calculateCpuPercentage from '../../../util/calculateCpuPercentage.js';

export default async (req, res) => {
	const { id } = req.params;

	// Fetch both 'inspect' and 'stats' and return a combined result
	try {
		const stats = await getContainerStats(id);
		const inspectData = await inspectContainer(id);
		const cpuPercentage = calculateCpuPercentage(stats);	

		res.json({
			...inspectData,
			cpu_percent: cpuPercentage,
		});
	} catch (error) {
		if (error.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			console.log(error.message);
			res.status(500).send('Server error: Failed to get container stats');
		}
	}
};
