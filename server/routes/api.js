var express = require('express');
var router = express.Router();

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

router.get('/list', list);

router.post('/start/:id', start);
router.get('/connection/:id', connect);

router.post('/stop/:id', stop);
router.post('/remove/:id', remove);

router.get('/inspect/:id', inspect);
router.get('/cpu/:id', cpu);


module.exports = router