const verifyTimeInterval = (bookings) => {
	let currentDate = new Date();

	const timeSlot = bookings.find((booking) => {
		const { start, end } = booking;
		const startDate = new Date(start);
		const endDate = new Date(end);

		// check if the current time falls within the allowed interval
		// if the current time is between the start and end time, then user has access to their booking
		if (startDate < currentDate && endDate > currentDate) {
			return true;
		}
	});
	return timeSlot;
};

export default verifyTimeInterval;
