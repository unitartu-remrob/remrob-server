import docker from './index.js';

const getDockerImages = async () => {
	const images = await docker.listImages();
	
	return images.map(image => image.RepoTags[0]).filter(image => image !== undefined);
}

export default getDockerImages;