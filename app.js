import createError from 'http-errors';
import express from 'express';
import path from 'path';
import cookieParser from 'cookie-parser';
import logger from 'morgan';
import cors from 'cors';
import dotenv from 'dotenv';
import expressWs from 'express-ws';

// load env
dotenv.config();

process.env.NODE_ENV = process.env.NODE_ENV || 'development';
console.log(`Running ${process.env.NODE_ENV} server`);

const app = express();
expressWs(app);

import containerAPI, { mountWsRoutes } from './src/api/index.js';
mountWsRoutes();

// view engine setup
app.set('views', path.join(import.meta.dirname, 'views'));
app.set('view engine', 'hbs');

app.use(cors());
app.use(logger('dev'));
app.use(express.json()); // body-parser
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());

// ------------------------------
app.use('/', containerAPI);
// ------------------------------

// Documentation
// app.use('/guide', authenticateAdminJWT, express.static(process.env['DOC_ROOT']));

// catch 404, forward rest to error handler
app.use(function (req, res, next) {
	next(createError(404));
});

// error handler
app.use(function (err, req, res) {
	// set locals, only providing error in development
	res.locals.message = err.message;
	res.locals.error = req.app.get('env') === 'development' ? err : {};

	// render the error page
	res.status(err.status || 500);
	res.render('error');
});

export default app;
