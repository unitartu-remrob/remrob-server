import app from './src/app.js';

const port = process.env.PORT || 9000;
app.set('port', port);

app.listen(port, () => {
	console.log('Server is running on port', port);
});
