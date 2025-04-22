import { startContainer } from '../../../../docker/containerManager.js';
import { getVncUrl } from '../../../../session/sessionComposer.js';
import { ROS_VERSION_JAZZY, JAZZY_STARTUP_DELAY } from '../../../../constants.js';

import { validateImageParams } from '../startContainer.js';

export default async (req, res) => {
	const { simtainer_id: containerId } = res.locals;

	const { rosVersion, imageTag } = req.body;

	try {
		await validateImageParams(rosVersion, imageTag);

		await startContainer(
			{
				containerId,
				isSimtainer: true,
				rosVersion,
				imageTag,
			},
			true
		);

		if (rosVersion === ROS_VERSION_JAZZY) {
			// for jazzy we need to wait a bit more for container to get ready
			await new Promise((resolve) => setTimeout(resolve, JAZZY_STARTUP_DELAY));
		}

		const vncUrl = await getVncUrl(containerId, true);

		res.json({ path: vncUrl });
	} catch (err) {
		console.log(err.message);
		res.status(500).json(`Server error: Failed to start public container`);
	}
};
