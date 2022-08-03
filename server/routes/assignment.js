var db = require("../data/db.js");

const assignContainer = (req, res) => {
	const { user, user_booking } = res.locals;
	
	// In case admin tries to get a container without an active booking
	if (user_booking === undefined) {
		return res.status(403).send('No active sessions available')
	}

	const inventoryTable = (user_booking.is_simulation) ? 'simulation_containers' : 'inventory';
	// If simulation container, add an empty filter, else check if the corresponding robot is active
	const status_filter = (user_booking.is_simulation) ? (b) => {b.where('container_id', '<', 1000)}: { status: true } 
	let now = new Date();

	// Check first if there is an already active session
	db(inventoryTable)
		.where({ user: user.sub })
		.andWhere('end_time', '>', now.toISOString())
		.then((inv) => {
		if (inv.length) {
			res.json(inv[0])
			// res.status(400).json('You already have an assigned container')
		} else {		
			// Get the first entry that is free (booking expired or user null)
			db(inventoryTable).first()
				.where(status_filter)
				.andWhere({ user: null })
				.orWhere('end_time', '<', now.toISOString(),)
				.then(item => {
					// Update the user column with the respective user ID coming from the JWT token
					if (item) {					
						const bookingData = {
							'user': user.sub,
							'end_time': user_booking.end
						}
						db(inventoryTable)
							.update(bookingData)
							.where('id', item["id"])
							.then(id => {
								res.json({ ...item, ...bookingData })
							})
					} else {
						res.status(500).send('No free inventory available')
					}
				})
		}
	})
}

module.exports = {
	assignContainer
}