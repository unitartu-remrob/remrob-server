import {
	getInventoryTable,
	checkIfUserAlreadyHasItem,
	findFreeInventoryItem,
	lockInventoryItem,
	lockPublicContainer,
} from './inventory.js';

import { generatePublicSessionCookieToken } from '../util/cookies.js';
import ErrorWithStatus from '../util/erorrs.js';

import { SIMTAINER_INVENTORY_TABLE } from '../constants.js';
import { markBookingAsActivated } from './booking.js';

const assignContainer = async (user, userBooking) => {
	const isSimtainer = userBooking.is_simulation;
	const bookingId = userBooking.id;

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
	markBookingAsActivated(bookingId);

	return lockedItem;
};

const claimPublicContainer = async () => {
	const freeItem = await findFreeInventoryItem(SIMTAINER_INVENTORY_TABLE, true);

	if (freeItem === null) {
		throw new ErrorWithStatus('No free inventory available', 404);
	}

	const publicSessionCookieToken = generatePublicSessionCookieToken();
	const lockedItem = await lockPublicContainer(freeItem, publicSessionCookieToken);

	return lockedItem;
};

export { assignContainer, claimPublicContainer };
