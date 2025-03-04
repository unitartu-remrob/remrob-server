import { stopContainer } from '../../../../docker/containerManager.js';

export default async (_, res) => {
	const { simtainer_id } = res.locals;

	try {
		await stopContainer(simtainer_id);
		res.json('Container successfully stopped');
	} catch (err) {
		if (err.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			res.status(500).send('Server error: Failed to stop public container');
		}
	}
};
