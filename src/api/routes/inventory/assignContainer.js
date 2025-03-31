import { assignContainer } from '../../../session/assignment.js';
import { LOCALROB_APP } from '../../../constants.js';

export default async (req, res) => {
	const { user, userBooking } = res.locals;
	const { is_simulation: isSim } = req.query;

	if (userBooking === undefined && !LOCALROB_APP) {
		// in case admin tries to get a container assigned without an active booking
		return res.status(403).send('No active sessions available');
	}

	try {
		const container = await assignContainer(user, userBooking, LOCALROB_APP, isSim === 'true');

		res.json(container);
	} catch (err) {
		console.log(err);
		res.status(500).send('Server error: Failed to assign container');
	}
};
