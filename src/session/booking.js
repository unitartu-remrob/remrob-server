import db from '../data/db.js';
import { BOOKING_TABLE } from '../constants.js';

export const markBookingAsActivated = (bookingId) => {
	db(BOOKING_TABLE).update({ activated: true }).where('id', bookingId);
};
