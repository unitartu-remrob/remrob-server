import createError from 'http-errors';
import express from 'express';
import path from 'path';
import { fileURLToPath } from 'url';
import cookieParser from 'cookie-parser';
import logger from 'morgan';
import cors from 'cors';
import dotenv from 'dotenv';
import expressWs from 'express-ws';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// load env
dotenv.config();

process.env.NODE_ENV = process.env.NODE_ENV || 'development';
console.log(`Running ${process.env.NODE_ENV} server`);

const app = express();
expressWs(app);

import containerAPI, { mountWsRoutes } from './api/index.js';
mountWsRoutes();

// view engine setup
app.set('views', path.join(__dirname, 'views'));
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
	res.locals.error = process.env.NODE_ENV === 'development' ? err : {};

	// render the error page
	res.status(err.status || 500);
	res.render('error');
});

export default app;
