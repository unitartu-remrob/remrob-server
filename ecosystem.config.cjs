module.exports = {
	apps: [
		{
			name: 'remrob',
			script: 'index.js',
			instances: '4',
			exec_mode: 'cluster',
			instance_var: 'INSTANCE_ID',
			env_production: {
				NODE_ENV: 'production',
			},
			env_development: {
				NODE_ENV: 'development',
			},
		},
	],
};
