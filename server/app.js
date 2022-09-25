var createError = require('http-errors');
var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
var cors = require('cors');

require('dotenv').config()
process.env.NODE_ENV = process.env.NODE_ENV || "development";
// console.log(`Running ${process.env.NODE_ENV} environment`);

// var indexRouter = require('./routes/index');
// var usersRouter = require('./routes/users');
var app = express();
var expressWs = require('express-ws')(app);
var containerAPI = require('./routes/api'); // must be loaded after setting up ws

// const { proxy, scriptUrl } = require('rtsp-relay')(app);
// const handler = proxy({
//   url: `rtsp://admin:@192.168.200.211:554/h264Preview_01_sub`,
//   verbose: false,
// });

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'hbs');

app.use(cors());
app.use(logger('dev'));
app.use(express.json()); // body-parser
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());

// app.use('/', indexRouter);
// app.use('/users', usersRouter);

app.use('/', containerAPI)
//app.ws('/api/stream', handler);

var db = require('./data/db.js');

// app.ws('/api/stream/:workcell', (ws, req) => {
//   const { workcell } = req.params  
//   const query_cell = parseInt(workcell)
//   if (!query_cell) {
//     console.log("Invalid stream param")
//     return ws.close()
//   }
//   db('cameras').first()
//     .where({ cell: query_cell })
//     .then( ({ camera_ip }) => {
//       // console.log(camera_ip);
//       proxy({
//         url: `rtsp://${process.env.CAM_CREDENTIALS}@${camera_ip}:554/h264Preview_01_main`,
//         verbose: false,
//       })(ws)
//     })
// });

// serve frontend at index
// app.use(express.static(path.join(__dirname, 'client/build')));

// catch 404 and forward to error handler
app.use(function(req, res, next) {
  next(createError(404));
});

// error handler
app.use(function(err, req, res, next) {
  // set locals, only providing error in development
  res.locals.message = err.message;
  res.locals.error = req.app.get('env') === 'development' ? err : {};

  // render the error page
  res.status(err.status || 500);
  res.render('error');
});

module.exports = app;
