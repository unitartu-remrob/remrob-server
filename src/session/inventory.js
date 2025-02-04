import db from '../../data/db.js';
import { killContainer } from '../docker/containerManager.js';
import { SIMTAINER_INVENTORY_TABLE, ROBOT_INVENTORY_TABLE } from '../constants.js';

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

const checkIfUserAlreadyHasItem = async (inventoryTable, user, endTime) => {
	const inventoryItems = await db(inventoryTable)
		.where({ user: user.sub })
		.andWhere('end_time', '>', endTime);

	if (inventoryItems.length) {
		return items[0];
	} else {
		return null;
	}
};

const findFreeInventoryItem = async (inventoryTable, currentTime) => {
	const query = db(inventoryTable)
		.first()
		.where(function () {
			this.orWhere({ user: null })
				.orWhere({ end_time: null })
				.orWhere('end_time', '<', currentTime);
		});

	if (inventoryTable === ROBOT_INVENTORY_TABLE) {
		// for robot inventory check if the robot is active
		query.andWhere({ status: true });
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

const setSessionTimeout = (inventoryTable, inventoryItem) => {
	let now = new Date();
	let end = new Date(inventoryItem.end_time);

	setTimeout(
		() => {
			db(inventoryTable)
				.update({
					user: null,
					end_time: null,
				})
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
};
