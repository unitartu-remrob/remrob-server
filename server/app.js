var createError = require('http-errors');
var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
var cors = require('cors');

require('dotenv').config()

// var indexRouter = require('./routes/index');
// var usersRouter = require('./routes/users');
var containerAPI = require('./routes/api');

var app = express();
app.use(cors());

process.env.NODE_ENV = process.env.NODE_ENV || "development";
// console.log(`Running ${process.env.NODE_ENV} environment`);

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'hbs');

app.use(logger('dev'));
app.use(express.json()); // body-parser
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());

// app.use('/', indexRouter);
// app.use('/users', usersRouter);


app.use('/api/container', containerAPI)

// serve frontend at index
app.use(express.static(path.join(__dirname, 'client/build')));

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
