var express = require('express');
var router = express.Router();

const {
	authenticateJWT,
	authenticateAdminJWT,
	checkSession
}  = require('../middleware/auth');

const  {
	names,
	list,
	start,
	stop,
	connect,
	remove,
	inspect,
	cpu
} = require('../compose/container-master.js')


router.get('/names', names)
//
router.get('/list', [authenticateJWT, checkSession], list);

router.post('/start/:id', [authenticateJWT, checkSession], start);
router.post('/stop/:id', [authenticateJWT, checkSession], stop);

router.get('/connection/:id', connect);

router.post('/remove/:id', [authenticateAdminJWT], remove);

router.get('/inspect/:id', [authenticateJWT, checkSession], inspect);
router.get('/cpu/:id', cpu);


module.exports = router