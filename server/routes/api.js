var express = require('express');
var router = express.Router();

const {
	authenticateJWT,
	authenticateAdminJWT,
	authenticateAdminWs,
	checkSession,
	checkOwnership
}  = require('../middleware/auth');

const {
	liveStats,
	robotMonitor
} = require('../compose/live.js')
const {
	assignContainer,
	yieldContainer
} = require('./assignment')
const  {
	start,
	stop,
	restart,
	commit,
	remove,
	inspect,
	stats,
} = require('../compose/container-master.js')


router.ws("/live/:version", authenticateAdminWs, liveStats);
router.ws("/robot-status/:id", robotMonitor);


router.get('/inspect/:id', [authenticateJWT, checkSession, checkOwnership], inspect);
router.get('/stats/:id', [authenticateJWT, checkSession, checkOwnership], stats);

router.get('/assign', [authenticateJWT, checkSession], assignContainer)
// router.post('/yield/:id', [authenticateJWT, checkSession, checkOwnership], yieldContainer)

router.post('/start/:id', [authenticateJWT, checkSession, checkOwnership], start);
router.post('/stop/:id', [authenticateJWT, checkSession, checkOwnership], stop);
router.post('/restart/:id', [authenticateJWT, checkSession, checkOwnership], stop);
router.post('/commit/:id', [authenticateJWT, checkSession, checkOwnership], commit);
router.post('/remove/:id', [authenticateJWT, checkSession, checkOwnership], remove);



module.exports = router