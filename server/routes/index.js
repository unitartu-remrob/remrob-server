var express = require('express');
var router = express.Router();
var url = require('url');

// http://localhost/novnc/vnc.html?password=remrob&autoconnect=true&path=novnc?token=robo2

const targets = []
for (let i = 0; i < 4; i++) {
  targets.push(new URL("http://localhost/novnc/vnc.html"));
}

targets.forEach((el, i) => {
  el.searchParams.append("password", "remrob");
  el.searchParams.append("autoconnect", "true");
  el.searchParams.append("resize", "remote");
  el.searchParams.append("path", "novnc");
  el.index = i + 1;
  el.href = el.href.concat(`?token=robo${el.index}`);
})

/* GET home page. */
router.get('/', function(req, res, next) {
  res.render('index', { title: 'Welcome to remrob', links: targets });
});

module.exports = router;
