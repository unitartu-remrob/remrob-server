import config from 'config';

const availableImages = config.get('AvailableImages');

export default (_, res) => {
	res.json(availableImages);
};
