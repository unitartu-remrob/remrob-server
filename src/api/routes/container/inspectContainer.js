import { inspectContainer } from '../../../docker/containerManager.js';

export default async (req, res) => {
	const { id } = req.params;

	try {
		const { Id, State, Config, NetworkSettings, Extrahosts } = await inspectContainer(id);

		res.json({
			Id,
			State,
			Config,
			NetworkSettings,
			Extrahosts,
		});
	} catch (err) {
		if (err.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			res.status(500).send('Server error: Failed to inspect container');
		}
	}
};
