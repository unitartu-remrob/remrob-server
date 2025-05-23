import config from 'config';
import db from '../data/db.js';
import { killContainer } from '../docker/containerManager.js';
import { SIMTAINER_INVENTORY_TABLE, ROBOT_INVENTORY_TABLE } from '../constants.js';

const UTC_OFFSET = config.get('UTC_OFFSET_HOURS') * 3600000;
const PUBLIC_SESSION_TIMEOUT = config.get('PublicSessionTimeoutMinutes') * 60 * 1000;

const getCurrentTime = () => {
	return new Date(new Date().getTime() + UTC_OFFSET).toISOString();
};

const getExpirationDate = (sessionDuration) => {
	// will return date in "YYYY-MM-DDTHH:mm" format
	return new Date(new Date().getTime() + UTC_OFFSET + sessionDuration).toISOString().slice(0, 16);
};

const getInventoryTable = (isSimtainer) => {
	return isSimtainer ? SIMTAINER_INVENTORY_TABLE : ROBOT_INVENTORY_TABLE;
};

const getRobotCell = async (containerId) => {
	const { cell } = await db(ROBOT_INVENTORY_TABLE)
		.first()
		.where({ slug: containerId })
		.select(['cell']);

	return cell;
};

const checkIfUserAlreadyHasItem = async (inventoryTable, user) => {
	const inventoryItems = await db(inventoryTable)
		.where({ user: user.sub })
		.andWhere('end_time', '>', getCurrentTime());

	if (inventoryItems.length) {
		return inventoryItems[0];
	} else {
		return null;
	}
};

const findFreeInventoryItem = async (inventoryTable, robot = null, publicContainerId = null) => {
	const query = db(inventoryTable)
		.first()
		.andWhere(function () {
			this.orWhere({ user: null })
				.orWhere({ end_time: null })
				.orWhere('end_time', '<', getCurrentTime());
		});

	if (inventoryTable === ROBOT_INVENTORY_TABLE) {
		// for robot inventory filter booked robot type and allow only active robots
		query.andWhere({ project: robot, status: true });
	}

	if (publicContainerId) {
		query.andWhere({
			open_to_public: true,
			public_user: null,
			slug: publicContainerId,
		});
	}

	const freeItem = await query;

	return freeItem ?? null;
};

const lockInventoryItem = async (inventoryTable, inventoryItem, user, userBooking) => {
	const bookingData = {
		user: user.sub,
		end_time: userBooking.end,
	};

	await db(inventoryTable).update(bookingData).where('id', inventoryItem.id);

	const lockedItem = {
		...inventoryItem,
		...bookingData,
	};

	await killContainer(inventoryItem.slug);
	setSessionTimeout(inventoryTable, lockedItem);

	return lockedItem;
};

const getPublicContainers = async () => {
	return await db(SIMTAINER_INVENTORY_TABLE)
		.where({
			open_to_public: true,
			user: null,
		})
		.select(['container_id', 'slug', 'end_time', 'public_user']);
};

const lockPublicContainer = async (inventoryItem, sessionToken) => {
	const sessionExpirationTime = getExpirationDate(PUBLIC_SESSION_TIMEOUT);

	const sessionData = {
		public_user: sessionToken,
		end_time: sessionExpirationTime,
	};

	await db(SIMTAINER_INVENTORY_TABLE).update(sessionData).where('id', inventoryItem.id);

	const lockedItem = {
		...inventoryItem,
		...sessionData,
		sessionToken,
	};

	await killContainer(inventoryItem.slug);
	setSessionTimeout(SIMTAINER_INVENTORY_TABLE, lockedItem, true);

	return lockedItem;
};

const unlockPublicContainer = async (slug) => {
	const cleanUpPayload = {
		public_user: null,
		end_time: null,
	};

	await db(SIMTAINER_INVENTORY_TABLE).update(cleanUpPayload).where('slug', slug);
	killContainer(slug);
};

const verifyPublicSessionToken = async (sessionToken) => {
	const publicContainer = await db(SIMTAINER_INVENTORY_TABLE)
		.where({ public_user: sessionToken })
		.first();

	if (!publicContainer) {
		return null;
	} else {
		return publicContainer;
	}
};

const setSessionTimeout = (inventoryTable, inventoryItem, publicSession = false) => {
	const now = new Date();
	const end = new Date(inventoryItem.end_time);

	const cleanUpPayload = {
		[publicSession ? 'public_user' : 'user']: null,
		end_time: null,
	};

	setTimeout(
		() => {
			db(inventoryTable)
				.update(cleanUpPayload)
				.where('id', inventoryItem.id)
				.then(() => {
					console.log(
						`Session @${inventoryItem.slug} expired (user #${inventoryItem.user}), terminating container`
					);
					killContainer(inventoryItem.slug);
				});
		},
		end - now - 3000 // 3 second buffer for edge cases
	);
};

export {
	getInventoryTable,
	getRobotCell,
	checkIfUserAlreadyHasItem,
	findFreeInventoryItem,
	getPublicContainers,
	lockInventoryItem,
	verifyPublicSessionToken,
	lockPublicContainer,
	unlockPublicContainer,
};
