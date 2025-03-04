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

const findFreeInventoryItem = async (inventoryTable, searchForPublic = false) => {
	const query = db(inventoryTable)
		.first()
		.where(function () {
			this.orWhere({ user: null })
				.orWhere({ end_time: null })
				.orWhere('end_time', '<', getCurrentTime());
		});

	if (inventoryTable === ROBOT_INVENTORY_TABLE) {
		// for robot inventory check if the robot is active
		query.andWhere({ status: true });
	}

	if (searchForPublic) {
		query.andWhere({
			open_to_public: true,
			public_user: null,
		});
	}

	const freeItem = await query;

	if (freeItem) {
		return freeItem;
	} else {
		return null;
	}
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
		[publicSession ? "public_user" : "user"]: null,
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
	lockInventoryItem,
	verifyPublicSessionToken,
	lockPublicContainer,
};
