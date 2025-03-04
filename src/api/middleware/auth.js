// https://github.com/jkasun/stack-abuse-express-jwt

import jwt from 'jsonwebtoken';
import axios from 'axios';

import db from '../../data/db.js';
import verifyTimeInterval from '../../util/time.js';
import { getInventoryTable } from '../../session/inventory.js';

const verifyToken = (token, callback) => {
	return jwt.verify(token, process.env.JWT_SECRET, callback);
};

const authenticateJWT = (req, res, next) => {
	const bearerToken = req.headers.authorization;

	if (!bearerToken) {
		return res.sendStatus(401);
	}

	const jwtToken = bearerToken.split(' ')[1];

	verifyToken(jwtToken, (err, user) => {
		if (err) {
			return res.sendStatus(401);
		}

		res.locals.user = user;
		res.locals.authHeader = {
			Authorization: bearerToken,
			'Content-Type': 'application/json',
		};
		next();
	});
};

const authenticateAdminJWT = (req, res, next) => {
	authenticateJWT(req, res, () => {
		const { user } = res.locals;

		if (user.is_administrator !== true) {
			return res.sendStatus(403);
		}

		next();
	});
};

const authenticateAdminJWTWebSocket = (ws, req, next) => {
	// WebSockets regular auth headers were being funky, that is why going with the cookie check
	const token = req.cookies.refresh_token_cookie;

	if (!token) {
		return ws.send(401);
	}

	verifyToken(token, (err, user) => {
		if (err) {
			return ws.send(401);
		}
		if (user.is_administrator !== true) {
			return ws.send(403);
		}
		next();
	});
};

const checkSession = (req, res, next) => {
	// get the user ID of who's trying to access container management
	const { user } = res.locals;

	axios
		.get(`${process.env.DB_SERVER}/bookings/${user.sub}`, {
			headers: req.headers,
		})
		.then((resp) => {
			const { user_bookings: userBookings } = resp.data;
			const activeBooking = verifyTimeInterval(userBookings);

			if (activeBooking !== undefined || user.is_administrator === true) {
				// admin is not bound by any session
				res.locals.userBooking = activeBooking;
				next();
			} else {
				res.status(403).send('No active sessions available');
			}
		})
		.catch((err) => {
			console.log('Node request to Flask DB rejected', err);
			res.status(502).send('Server error');
		});
};

const checkContainerOwnership = (req, res, next) => {
	// middleware that checks if the targeted container belongs to the user
	const { user, userBooking } = res.locals;
	const { id } = req.params;

	if (user.is_administrator === true) {
		// admin can control any container
		return next();
	}

	const { is_simulation } = userBooking;

	const tableId = getInventoryTable(is_simulation);

	db(tableId)
		.where({ user: user.sub })
		.select('slug')
		.then((inventoryItem) => {
			const itemOwnershipClaimIsValid = inventoryItem.find((item) => item.slug === id);

			if (itemOwnershipClaimIsValid === undefined) {
				res.status(403).send(`The container ${id} does not belong to you`);
			} else {
				next();
			}
		});
};

export {
	authenticateJWT,
	authenticateAdminJWT,
	authenticateAdminJWTWebSocket,
	checkSession,
	checkContainerOwnership,
};
