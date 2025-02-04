import config from 'config';

import {
	getInventoryTable,
	checkIfUserAlreadyHasItem,
	findFreeInventoryItem,
	lockInventoryItem,
} from './inventory.js';

import { markBookingAsActivated } from './booking.js';

const UTC_OFFSET = config.get('Time.UTC_OFFSET');

const assignContainer = async (user, userBooking) => {
	const isSimtainer = userBooking.is_simulation;
	const bookingId = userBooking.id;

	const inventoryTable = getInventoryTable(isSimtainer);
	const currentTime = new Date(new Date().getTime() + UTC_OFFSET * 3600000).toISOString();

	const inventoryItem = await checkIfUserAlreadyHasItem(inventoryTable, user, currentTime);

	if (inventoryItem !== null) {
		return inventoryItem;
	}

	const freeItem = await findFreeInventoryItem(inventoryTable, currentTime);

	if (freeItem === null) {
		throw new Error('No free inventory available');
	}

	const lockedItem = await lockInventoryItem(inventoryTable, freeItem, user, userBooking);
	markBookingAsActivated(bookingId);

	return lockedItem;
};

export { assignContainer };
