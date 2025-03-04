import config from 'config';
import getDockerImages from '../../docker/getDockerImages.js';

const remrobImages = config.get('RemrobDockerImages');

const overrides = remrobImages.filter((image) => image.overridenBy !== undefined);

export default async (_, res) => {
	const builtImages = await getDockerImages();

	const remrobAvailableImages = remrobImages
		.filter((image) => builtImages.includes(image.imageTag))
		.map((image) => {
			const override = overrides.find(
				(override) =>
					override.imageTag === image.imageTag &&
					builtImages.includes(override.overridenBy)
			);

			image = { ...image }; // make copy so can delete the overridenBy property
			delete image.overridenBy;

			return override ? { ...image, imageTag: override.overridenBy } : image;
		});

	res.json(remrobAvailableImages);
};
