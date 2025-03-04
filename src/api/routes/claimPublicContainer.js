import config from 'config';
import { claimPublicContainer } from '../../session/assignment.js';

const PUBLIC_SESSION_TIMEOUT = config.get('PublicSessionTimeoutMinutes') * 60 * 1000;

export default async (_, res) => {
	try {
		const container = await claimPublicContainer();

		res.cookie('remrob_session_cookie', container.sessionToken, {
			maxAge: PUBLIC_SESSION_TIMEOUT,
			httpOnly: true,
			sameSite: 'Strict',
		});
		res.json(container);
	} catch (err) {
		console.log(err);
		if (err.statusCode === 404) {
			res.status(404).send(`Error: ${err.message}`);
		} else {
			res.status(500).send('Server error: Failed to assign container');
		}
	}
};
