import { removeContainer } from '../../../../docker/containerManager.js';

export default async (_, res) => {
	const { simtainer_id } = res.locals;

	try {
		await removeContainer(simtainer_id);
		res.json('Container successfully removed');
	} catch (err) {
		if (err.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			console.log(err);
			res.status(500).send('Server error: Failed to remove public container');
		}
	}
};
