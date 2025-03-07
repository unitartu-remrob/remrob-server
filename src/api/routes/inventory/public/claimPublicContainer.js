import config from 'config';
import { claimPublicContainer } from '../../../../session/assignment.js';
import { verifyPublicSessionToken } from '../../../../session/inventory.js';

const PUBLIC_SESSION_TIMEOUT = config.get('PublicSessionTimeoutMinutes') * 60 * 1000;

export default async (req, res) => {
	try {
		const { id } = req.params;
		const { remrob_session_cookie } = req.cookies;

		if (remrob_session_cookie !== undefined) {
			const publicUserContainer = await verifyPublicSessionToken(remrob_session_cookie, id)

			if (publicUserContainer && publicUserContainer.slug !== id) {
				return res.status(400).send(`Error: Cannot claim ${id}, you have already claimed ${publicUserContainer.slug}`);
			} else if (publicUserContainer) {
				return res.json(publicUserContainer);
			} else {
				// cookie expired
			}
		}

		const container = await claimPublicContainer(id);

		res.cookie('remrob_session_cookie', container.sessionToken, {
			maxAge: PUBLIC_SESSION_TIMEOUT,
			httpOnly: true,
			sameSite: 'Strict',
		});
		res.json(container);
	} catch (err) {
		console.log(err);
		if ([400, 403].includes(err.statusCode)) {
			res.status(err.statusCode).send(`Error: ${err.message}`);
		} else {
			res.status(500).send('Server error: Failed to assign container');
		}
	}
};
