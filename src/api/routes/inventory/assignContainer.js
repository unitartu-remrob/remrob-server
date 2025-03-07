import { assignContainer } from '../../../session/assignment.js';

export default async (_, res) => {
	const { user, userBooking } = res.locals;

	if (userBooking === undefined) {
		// in case admin tries to get a container assigned without an active booking
		return res.status(403).send('No active sessions available');
	}

	try {
		const container = await assignContainer(user, userBooking);

		res.json(container);
	} catch (err) {
		console.log(err);
		res.status(500).send('Server error: Failed to assign container');
	}
};
