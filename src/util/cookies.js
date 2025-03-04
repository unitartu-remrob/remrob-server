import generator from 'generate-password';

export const generatePublicSessionCookieToken = () => {
	return generator.generate({
		length: 32,
		numbers: true,
	});
};
