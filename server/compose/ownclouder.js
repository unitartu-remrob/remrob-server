const axios = require('axios');

const auth = {
	username: process.env.OCS_USER,
	password: process.env.OCS_TOKEN
}

const createShareLink = async (name) => {
	const shareStuff = await axios.post(`${process.env.OCS_URL}/shares`,
		{
			// assuming the target cloud account already has a folder named "remrob"
			path: `remrob/${name}`,
			shareType: "3",
			permissions: "1"
		},
		{
			auth,
			headers: {
				"OCS-APIRequest": true
			}
	}).catch(e => { console.log(e) })
	if (shareStuff !== undefined) {
		const { data } = shareStuff.data.ocs;
		return data.token
	} else {
		return ""
	}
}

module.exports = {
	createShareLink
}
