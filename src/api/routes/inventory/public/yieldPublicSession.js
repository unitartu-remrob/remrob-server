import { verifyPublicSessionToken, unlockPublicContainer } from '../../../../session/inventory.js';

export default async (req, res) => {
	try {
		const { id } = req.params;
		const { remrob_session_cookie } = req.cookies;

		if (remrob_session_cookie !== undefined) {
			const publicUserContainer = await verifyPublicSessionToken(remrob_session_cookie, id)

			if (publicUserContainer && publicUserContainer.slug !== id) {
				return res.status(400).send(`Error: Cannot yield ${id}, you have no claim over it`);
			} else if (publicUserContainer) {
				await unlockPublicContainer(id);
				res.clearCookie('remrob_session_cookie');
				return res.json("Session successfully given up");
			} else {
				// cookie expired
			}
		} else {
			return res.status(404).send('Error: No session to yield');
		}
	} catch (err) {
		console.log(err);
		if (err.statusCode === 404) {
			res.status(404).send(`Error: ${err.message}`);
		} else {
			res.status(500).send('Server error: could not retrieve public containers');
		}
	}
};
