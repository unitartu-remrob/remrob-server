const axios = require('axios');

const auth = {
	username: process.env.OCS_USER,
	password: process.env.OCS_TOKEN
}

const createShareLink = async (name, email) => {
	const shareStuff = await axios.post(`${process.env.OCS_URL}/shares`,
		{
			// assuming the target cloud account already has a folder named "remrob"
			path: `remrob/${name}`,
			shareType: "3",
			permissions: "1",
			shareWith: "chooky823@gmail.com"
		},
		{
			auth,
			headers: {
				"OCS-APIRequest": true
			}
	}).catch(e => { console.log(e) })
	if (shareStuff !== undefined) {
		const { data } = shareStuff.data.ocs;
		console.log(data)
		return data.token
	} else {
		return null
	}
}


const getShareLink = async (req, res) => {
	const { name, email } = req.query;
	const stuff = await createShareLink(name, email)
	console.log(stuff)
	res.send(stuff)
}

module.exports = {
	createShareLink,
	getShareLink
}
