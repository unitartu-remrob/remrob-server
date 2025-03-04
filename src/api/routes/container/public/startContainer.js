import { startContainer } from '../../../../docker/containerManager.js';
import { getVncUrl } from '../../../../session/sessionComposer.js';

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

		const vncUrl = await getVncUrl(containerId, true);

		res.json({ path: vncUrl });
	} catch (err) {
		console.log(err.message);
		res.status(500).json(`Server error: Failed to start public container`);
	}
};
