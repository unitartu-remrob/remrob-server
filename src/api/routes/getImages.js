import config from 'config';
import getDockerImages from '../../docker/getDockerImages.js';

const remrobImages = config.get('RemrobDockerImages');

export const getAvailableRemrobImages = async () => {
	const builtImages = await getDockerImages();
	const overrides = remrobImages.filter((image) => image.overridenBy !== undefined);

	return remrobImages
		.filter((image) => image.enabled)
		.filter((image) => builtImages.includes(image.imageTag) || builtImages.includes(image.overridenBy))
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
};

export default async (_, res) => {
	const remrobAvailableImages = await getAvailableRemrobImages();

	res.json(remrobAvailableImages);
};
