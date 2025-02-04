import { removeContainer } from '../../../docker/containerManager.js';

export default async (req, res) => {
	const { id } = req.params;
	try {
		await removeContainer(id);
		res.json('Container successfully removed');
	} catch (err) {
		if (err.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			// add a log here
			res.status(500).send('Server error: Failed to remove container');
		}
	}
};
