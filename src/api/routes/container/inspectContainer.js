import { inspectContainer } from '../../../docker/containerManager.js';

export default async (req, res) => {
	const { id } = req.params;

	try {
		const { status, createdAt, ipAddress, image } = await inspectContainer(id);

		res.json({
			status,
			createdAt,
			ipAddress,
			image,
		});
	} catch (err) {
		if (err.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			console.log(err.message);
			res.status(500).send('Server error: Failed to inspect container');
		}
	}
};
