import { getPublicContainers } from "../../../../session/inventory.js";

export default async (req, res) => {
	try {
		const { remrob_session_cookie } = req.cookies;

		const publicContainers = await getPublicContainers();

		publicContainers.forEach(container => {
			container.occupied = !!container.public_user;

			if (container.public_user !== remrob_session_cookie) {
				delete container.public_user;
			}
		});
		
		res.json(publicContainers.sort((a, b) => a.container_id - b.container_id));
	} catch (err) {
		console.log(err);
		if (err.statusCode === 404) {
			res.status(404).send(`Error: ${err.message}`);
		} else {
			res.status(500).send('Server error: could not retrieve public containers');
		}
	}
};
