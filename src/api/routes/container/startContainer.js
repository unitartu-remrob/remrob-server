import { startContainer } from '../../../docker/containerManager.js';
import { getVncUrl } from '../../../session/sessionComposer.js';
import ErrorWithStatus from '../../../util/erorrs.js';
import { getAvailableRemrobImages } from '../getImages.js';
import { ROS_VERSION_JAZZY, JAZZY_STARTUP_DELAY } from '../../../constants.js';

export const validateImageParams = async (rosVersion, imageTag) => {
	if (!rosVersion || !imageTag) {
		throw new ErrorWithStatus('Missing required parameters: rosVersion, imageTag', 400);
	}

	const remrobImages = await getAvailableRemrobImages();

	const image = remrobImages.find(
		(image) => image.rosVersion === rosVersion && image.imageTag === imageTag
	);

	if (!image) {
		throw new ErrorWithStatus(`ROS ${rosVersion} image ${imageTag} not present`, 400);
	}
};

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
		await validateImageParams(rosVersion, imageTag);

		await startContainer({
			containerId,
			userId,
			userBooking,
			isSimtainer,
			isAdmin,
			rosVersion,
			imageTag,
		});

		if (rosVersion === ROS_VERSION_JAZZY) {
			// for jazzy we need to wait a bit more for container to get ready
			await new Promise((resolve) => setTimeout(resolve, JAZZY_STARTUP_DELAY));
		}

		const vncUrl = await getVncUrl(containerId, isSimtainer);

		res.json({ path: vncUrl });
	} catch (err) {
		if (err.statusCode === 400) {
			return res.status(400).json(err.message);
		} else {
			res.status(500).json('Server error: Failed to start container');
		}
	}
};
