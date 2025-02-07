import { startContainer } from '../../../docker/containerManager.js';
import { getVncUrl } from '../../../session/sessionComposer.js';

export default async (req, res) => {
	const { id: containerId } = req.params; // id = robo(sim)-{x}
	const {
		user: { sub: userId, is_administrator: isAdmin },
		userBooking,
	} = res.locals;

	const { rosVersion, imageTag } = req.body;

	const isSimtainer =
		userBooking && !isAdmin ? userBooking.is_simulation : req.query.is_simulation === 'true';

	try {
		await startContainer({
			containerId,
			userId,
			userBooking,
			isSimtainer,
			isAdmin,
			rosVersion,
			imageTag
		});

		const vncUrl = await getVncUrl(containerId, isSimtainer);

		res.json({ path: vncUrl });
	} catch (err) {
		res.status(500).json(`Failed to start container: ${err.message}`);
	}
};
