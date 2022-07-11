// https://github.com/jkasun/stack-abuse-express-jwt

const jwt = require("jsonwebtoken");
const axios = require('axios');

const verifyTimeInterval = require('../util/time');

const authenticateJWT = (req, res, next) => {
	const authHeader = req.headers.authorization;
	console.log(authHeader)
	if (authHeader) {
			const token = authHeader.split(' ')[1];

			jwt.verify(token, process.env.JWT_SECRET, (err, user) => {
					if (err) {
							return res.sendStatus(403);
					}

					res.locals.user = user;
					// res.locals.authHeader = authHeader;
					next();
			});
	} else {
			res.sendStatus(401);
	}
}

const authenticateAdminJWT = (req, res, next) => {
	const authHeader = req.headers.authorization;

	if (authHeader) {
			const token = authHeader.split(' ')[1];

			jwt.verify(token, process.env.JWT_SECRET, (err, user) => {
					if (err) {
							return res.sendStatus(403);
					}
					if (user.is_administrator !== true) {
						return res.sendStatus(401);
					}
					res.locals.user = user;
					next();
			});
	} else {
			res.sendStatus(401);
	}
}

const checkSession = async (req, res, next) => {

	console.log("Checking session validity...")
	// Get the user ID of who's trying to access container management
	const { user } = res.locals;

	if (user.is_administrator === true) {
		// Admin validated without time restrictions
		next()
	} else {
		axios.get(`http://localhost:5000/api/v1/bookings/${user.sub}`, { headers: req.headers }).then((resp) => {
			if (verifyTimeInterval(resp.data.user_bookings)) {
				// If valid session, pass forward to container api
				next()
			} else {
				res.status(403).send('No active sessions available')
			}
		}).catch(err => {
			console.warn("error", err.response.data);
			res.sendStatus(err.response.status)
		})
	}
}

module.exports = {
	authenticateJWT,
	authenticateAdminJWT,
	checkSession
}