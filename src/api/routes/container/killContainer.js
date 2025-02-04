import { killContainer } from '../../../docker/containerManager.js';

// Endpoint experimental
export default async (req, res) => {
	const { id } = req.params;
	try {
		await killContainer(id);
		res.json('Container terminated successfully');
	} catch (err) {
		res.status(404).send('whoopsie');
	}
};
