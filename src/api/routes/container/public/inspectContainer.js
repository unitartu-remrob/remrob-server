import { inspectContainer } from '../../../../docker/containerManager.js';

export default async (_, res) => {
	const { simtainer_id } = res.locals;

	try {
		const { status, createdAt, image } = await inspectContainer(simtainer_id);

		res.json({
			status,
			createdAt,
			image,
		});
	} catch (err) {
		if (err.statusCode === 404) {
			res.status(404).send('No container with that ID');
		} else {
			console.log(err.message);
			res.status(500).send('Server error: Failed to inspect public container');
		}
	}
};
