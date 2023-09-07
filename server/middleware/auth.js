// https://github.com/jkasun/stack-abuse-express-jwt

const jwt = require("jsonwebtoken");
const axios = require('axios');

var db = require("../data/db.js");
const verifyTimeInterval = require('../util/time');

const authenticateJWT = (req, res, next) => {
	const tokenHeader = req.headers.authorization;

	const authHeader = {
			'Authorization': tokenHeader,
			'Content-Type': 'application/json'
	}

	if (tokenHeader) {
			const token = tokenHeader.split(' ')[1];

			jwt.verify(token, process.env.JWT_SECRET, (err, user) => {
					if (err) {
						return res.sendStatus(401);
					}

					res.locals.user = user;
					res.locals.authHeader = authHeader;
					next();
			});
	} else {
			res.sendStatus(401);
	}
}

const authenticateAdminJWT = (req, res, next) => {
	const token = req.cookies.refresh_token_cookie
	jwt.verify(token, process.env.JWT_SECRET, (err, user) => {
		if (err) {
			res.sendStatus(401);
		}
		if (user.is_administrator !== true) {
			return res.sendStatus(403);
		}		
		next();
	});
}

const authenticateAdminWs = (ws, req, next) => {
	const token = req.cookies.refresh_token_cookie

	jwt.verify(token, process.env.JWT_SECRET, (err, user) => {
		if (err) {
			return ws.send(401);
		}
		if (user.is_administrator !== true) {
			return ws.send(403);
		}		
		next();
	});
}


const checkSession = (req, res, next) => {
	// Get the user ID of who's trying to access container management
	const { user } = res.locals;

	axios.get(`${process.env.DB_SERVER}/bookings/${user.sub}`, { headers: req.headers }).then((resp) => {
		const { user_bookings } = resp.data;
		const active_booking = verifyTimeInterval(user_bookings)
		
		if (active_booking !== undefined || user.is_administrator === true) {
			// If valid session, pass forward to container api with the corresponding container allowed
			res.locals.user_booking = active_booking;
			next()
		} else {
			res.status(403).send('No active sessions available')
		}
	}).catch(err => {
		console.log("Node request to Flask DB rejected");
	})
}

const checkContainerOwnership = (req, res, next) => {
	// MIDDLEWARE THAT CHECKS IF THE CONTAINER THAT IS TARGETED BELONGS TO THE USER BY QUERYING DB
	const { user } = res.locals;
	const { id } = req.params;
	
	if (user.is_administrator !== true) {
		const { user_booking } = res.locals;
		const { is_simulation } = user_booking;
		const table_id = (is_simulation) ? "simulation_containers": "inventory";
		db(table_id)
			.where({ user: user.sub })
			.select('slug')
			.then(inv_item => {
				const claim_true = inv_item.find(item => item.slug === id);
				if (claim_true === undefined) {
					res.status(403).send('This container does not belong to you')
				} else {
					next()
				}
			})
	} else {
		next()
	} 
}

module.exports = {
	authenticateJWT,
	authenticateAdminJWT,
	authenticateAdminWs,
	checkSession,
	checkOwnership: checkContainerOwnership
}