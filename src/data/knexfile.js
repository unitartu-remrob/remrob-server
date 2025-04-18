import dotenv from 'dotenv';
dotenv.config();

export default {
	development: {
		client: 'pg',
		connection: process.env.DB_URL,
		searchPath: ['knex', 'public'],
		migrations: {
			directory: './data/migrations',
		},
		seeds: { directory: './data/seeds' },
	},

	testing: {
		client: 'pg',
		connection: process.env.DB_URL,
		searchPath: ['knex', 'public'],
		migrations: {
			directory: './data/migrations',
		},
		seeds: { directory: './data/seeds' },
	},

	production: {
		client: 'pg',
		connection: process.env.DB_URL,
		searchPath: ['knex', 'public'],
		migrations: {
			directory: './data/migrations',
		},
		seeds: { directory: './data/seeds' },
	},
};
