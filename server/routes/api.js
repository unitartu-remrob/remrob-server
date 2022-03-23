var express = require('express');
var router = express.Router();
var url = require('url');

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




// router.post('/startContainer/:id', function(req, res, next) {
  
// });

// a function that checks container status, it's gonna be used by start and stop to check right?

router.get('/', function(req, res) {
  res.json({api: "located"});
});

router.get('/list', list);

router.get('/status/:id', status)

// router.get('/startContainer/:id')
router.post('/start/:id', start);
router.get('/connection/:id', connect);

router.post('/stop/:id', stop);
router.post('/remove/:id', remove);

router.get('/inspect/:id', inspect);



router.get('/test', test);



module.exports = router



// const targets = []
// for (let i = 0; i < 4; i++) {
//   targets.push(new URL("http://localhost/novnc/vnc.html"));
// }

// targets.forEach((el, i) => {
//   el.searchParams.append("password", "remrob");
//   el.searchParams.append("autoconnect", "true");
//   el.searchParams.append("resize", "remote");
//   el.searchParams.append("path", "novnc");
//   el.index = i + 1;
//   el.href = el.href.concat(`?token=robo${el.index}`);
// })
