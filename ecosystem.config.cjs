module.exports = {
	apps: [
		{
			name: 'remrob',
			script: 'index.js',
			instances: '4',
			exec_mode: 'cluster',
			env_production: {
				NODE_ENV: 'production',
			},
			env_development: {
				NODE_ENV: 'development',
			},
		},
	],
};
