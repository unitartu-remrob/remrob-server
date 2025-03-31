import {
	getInventoryTable,
	checkIfUserAlreadyHasItem,
	findFreeInventoryItem,
	lockInventoryItem,
	lockPublicContainer,
} from './inventory.js';

import db from '../data/db.js';
import config from 'config';

import { generatePublicSessionCookieToken } from '../util/cookies.js';
import ErrorWithStatus from '../util/erorrs.js';

import { SIMTAINER_INVENTORY_TABLE, ROBOT_INVENTORY_TABLE, USER_TABLE } from '../constants.js';
import { markBookingAsActivated } from './booking.js';

const inventoryRobotsAvailable = config.get('RobotsAvailable');

const assignContainer = async (user, userBooking, isLocalRob, isSim) => {
	if (isLocalRob) {
		const reservedUser = await checkIfReservedUser(user.sub);

		if (reservedUser) {
			const robotUser = reservedUser.email;
			return await assignLocalRobContainer(robotUser, isSim);
		}
	}

	const isSimtainer = userBooking.is_simulation;
	const inventoryTable = getInventoryTable(isSimtainer);

	const inventoryItem = await checkIfUserAlreadyHasItem(inventoryTable, user);

	if (inventoryItem !== null) {
		return inventoryItem;
	}

	const freeItem = await findFreeInventoryItem(inventoryTable);

	if (freeItem === null) {
		throw new ErrorWithStatus('No free inventory available', 404);
	}

	const lockedItem = await lockInventoryItem(inventoryTable, freeItem, user, userBooking);

	if (!isLocalRob) {
		markBookingAsActivated(userBooking.id);
	}

	return lockedItem;
};

const claimPublicContainer = async (containerId) => {
	const publicContainer = await findFreeInventoryItem(SIMTAINER_INVENTORY_TABLE, containerId);

	if (publicContainer === null) {
		throw new ErrorWithStatus(`Container ${containerId} is not available for taking`, 403);
	}

	const publicSessionCookieToken = generatePublicSessionCookieToken();
	const lockedItem = await lockPublicContainer(publicContainer, publicSessionCookieToken);

	return lockedItem;
};

const checkIfReservedUser = async (userId) => {
	const user = await db(USER_TABLE).where({ id: userId }).select('email').first();

	const isRobotUser = inventoryRobotsAvailable.some((robot) => user.email.includes(robot));

	return isRobotUser ? user : null;
};

const assignLocalRobContainer = async (robotUser, isSim) => {
	const robotId = robotUser.split('-')[1];

	if (!isSim) {
		const robotContainer = await db(ROBOT_INVENTORY_TABLE).first().where('robot_id', robotId);

		if (robotContainer) {
			return robotContainer;
		} else {
			throw new ErrorWithStatus(`No robot container available for user ${robotUser}`, 404);
		}
	} else {
		return await findFreeInventoryItem(SIMTAINER_INVENTORY_TABLE);
	}
};

export { assignContainer, claimPublicContainer };
