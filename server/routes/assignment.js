var db = require("../data/db.js");
// const axios = require('axios');
const { killContainer } = require("../compose/container-master");

const config = require('config');
// EST: times 3 summer time, times 2 winter time
const UTC_OFFSET = process.env.UTC_OFFSET || config.get('Time.UTC_OFFSET');

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
	const adjustedTime = new Date(now.getTime() + UTC_OFFSET * 3600000);

	// Check first if there is an already active session
	db(inventoryTable)
		.where({ user: user.sub })
		.andWhere('end_time', '>', adjustedTime.toISOString())
		.then((inv) => {
		if (inv.length) {
			res.json(inv[0])
			// res.status(400).json('You already have an assigned container')
		} else {		
			// Get the first entry that is free (booking expired or user null)
			db(inventoryTable).first()
				.where(status_filter)
				.andWhere( function () {
					this.orWhere({ user: null })
						.orWhere({ end_time: null }) // This probably redundant
						.orWhere('end_time', '<', now.toISOString(),)
				}).then(item => {
					// Update the inventory item user column with the respective user ID coming from the JWT token
					if (item) {	
						console.log(item)
						const bookingData = {
							'user': user.sub,
							'end_time': user_booking.end
						}
						db(inventoryTable)
							.update(bookingData)
							.where('id', item["id"])
							.then(async blank => {
								killContainer(item.slug).then(async() => {
									const claimed_item = { ...item, ...bookingData } // start_time: user_booking.start 
									setSessionTimeout(claimed_item, inventoryTable)
									res.json(claimed_item)
								})	
							})
						// Mark that the booking has been activated
						db('bookings')
							.update({ "activated": true })
							.where('id', user_booking.id)
							.then(resp => {
								console.log("Booking activated!")
							})
					} else {
						res.status(500).send('No free inventory available')
					}
				})
		}
	})
}

const setSessionTimeout = (item, inv) => {
		let now = new Date();
		let end = new Date(item['end_time']);
		setTimeout(() => {
			db(inv)
				.update({
					'user': null,
					'end_time': null
				})
				.where('id', item['id'])
				.then(blank => {
					console.log(`Session @${item['slug']} expired (user #${item['user']})`)
				})
		}, end - now - 5000) // 5 second buffer for edge cases
}

const yieldContainer = (req, res) => {
	const { id } = req.params;
	const { user, user_booking } = res.locals;
	const inventoryTable = (user_booking.is_simulation) ? 'simulation_containers' : 'inventory';

	const clear = {
		'user': null,
		'end_time': null
	}

	killContainer(id);

	db(inventoryTable)
		.update(clear)
		.where({ slug: id })
		.then(item => {
			res.json("Container yielded")
		})
	db('bookings')
		.update({ user_id: null })
		.where({ id: user_booking.id })
		.then(item => {
			
		})
}


module.exports = {
	assignContainer,
	yieldContainer
}