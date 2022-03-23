var express = require('express');
var router = express.Router();

const  {
	test,
	list,
	status,
	start,
	stop,
	connect,
	remove,
	inspect
} = require('../compose/container-master.js')


router.get('/', function(req, res) {
  res.json({api: "located"});
});

router.get('/list', list);

router.get('/status/:id', status)

router.post('/start/:id', start);
router.get('/connection/:id', connect);

router.post('/stop/:id', stop);
router.post('/remove/:id', remove);

router.get('/inspect/:id', inspect);



router.get('/test', test);



module.exports = router