var express = require('express');
var router = express.Router();

const {
	authenticateJWT,
	authenticateAdminJWT,
	checkSession,
	checkOwnership
}  = require('../middleware/auth');

const  {
	list,
	start,
	stop,
	restart,
	connect,
	remove,
	inspect,
	stats,
	assign
} = require('../compose/container-master.js')


//
router.get('/list', [authenticateJWT, checkSession], list);

router.get('/inspect/:id', [authenticateJWT, checkSession, checkOwnership], inspect);
router.get('/stats/:id', [authenticateJWT, checkSession, checkOwnership], stats);

router.get('/assign', [authenticateJWT, checkSession], assign)

router.post('/start/:id', [authenticateJWT, checkSession, checkOwnership], start);
router.post('/stop/:id', [authenticateJWT, checkSession, checkOwnership], stop);
router.post('/restart/:id', [authenticateJWT, checkSession, checkOwnership], stop);
router.post('/remove/:id', [authenticateAdminJWT], remove);

router.get('/connection/:id', connect);


// router.get('/cpu/:id', cpu);


module.exports = router