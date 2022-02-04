var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/', function(req, res, next) {
  // res.render('index', { title: 'Express' });
  // proxy.web(req, res, {
  //   target: `ws://${process.env.CONTAINER}`
  // })
});

module.exports = router;
