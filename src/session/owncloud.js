import axios from 'axios';
import db from '../../data/db.js';

import { USER_TABLE } from '../constants.js';

const OWNCLOUD_AUTH = {
	username: process.env.OCS_USER,
	password: process.env.OCS_TOKEN,
};

export const getOwncloudShareLink = async (userId) => {
	const { owncloud_id } = await db(USER_TABLE)
		.first()
		.where({ id: userId })
		.select(['owncloud_id']);

	return owncloud_id;
};

export const updateOwncloudShareLink = (ownCloudShareToken, userId) => {
	db(USER_TABLE).update({ owncloud_id: ownCloudShareToken }).where({ id: userId });
};

export const createOwncloudShareLink = async (name) => {
	// requesting that the cloud-synced user folder under the /remrob directory be shared with a token

	const createdShareLink = await axios
		.post(
			`${process.env.OCS_URL}/shares`,
			{
				// assuming the target cloud account already has a folder named "remrob"
				path: `remrob/${name}`,
				shareType: '3',
				permissions: '1',
			},
			{
				OWNCLOUD_AUTH,
				headers: {
					'OCS-APIRequest': true,
				},
			}
		)
		.catch((e) => {
			console.log(`Failed to create an owncloud share link for ${name}: ${e.message}`);
		});
	if (createdShareLink !== undefined) {
		const { data } = createdShareLink.data.ocs;

		return data.token;
	} else {
		return null;
	}
};
