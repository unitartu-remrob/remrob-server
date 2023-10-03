const simpleGit = require('simple-git');
const path = require('path');
const fs = require('fs');
const axios = require('axios');
const { v4: uuidv4 } = require('uuid');

require('dotenv').config({ path: path.join(__dirname, '.env') })
const {
	REPOS_ROOT,
	GITLAB_PROJECT_URL,
	GITLAB_GROUP_NS_ID,
	GITLAB_API_URL,
	GITLAB_PAT // it's not possible to create other access token with a regular access token, a PAT is needed
} = process.env;

const headers = {
	Authorization: `Bearer ${GITLAB_PAT}`,
	'Content-Type': 'application/json'
};

class gitMaster {
	constructor(repoName, firstName="remrob", email="remrob@ut.ee") {
		this.repoName = repoName;
		this.repoPath = path.join(REPOS_ROOT, this.repoName, "catkin_ws", "src");
		this.remote = `${GITLAB_PROJECT_URL}/${this.repoName}.git`;
		this.firstName = firstName;
		this.initLocalRepo(firstName, email);

		this.studentTokenName = "remrob_student";
		this.studentGitToken = "";
		// this.remoteWithAuth = `${beginning}${TOKEN_NAME}:${TOKEN}@${end}/${this.repoName}.git`;
	}

	constructRemoteWithAuth = () => {
		const injectPoint = GITLAB_PROJECT_URL.indexOf('gitlab');
		const beginning = GITLAB_PROJECT_URL.slice(0, injectPoint);
		const end = GITLAB_PROJECT_URL.slice(injectPoint);
		return `${beginning}${this.studentTokenName}:${this.studentGitToken}@${end}/${this.repoName}.git`;
	}

	initLocalRepo = async(firstName, email) => {
		try {
			this.repo = simpleGit(this.repoPath);
			this.repo.init(); // in case of old workspace
			await this.repo.addConfig('user.name', firstName);
    	await this.repo.addConfig('user.email', email);
		} catch (error) {
			console.error('Failed to initialize local repo:', error);
		}
	}

	initRemoteRepo = async(userName) => {
		const description = `
		🔋 Project of Remrob student: ${userName}
			🧲 🔩
		`;
		const payload = JSON.stringify({
			name: this.repoName,
			description: description,
			path: this.repoName,
			initialize_with_readme: false,
			namespace_id: GITLAB_GROUP_NS_ID
		});

		try {
			const response = await axios.post(`${GITLAB_API_URL}/projects`, payload, { headers });
			const { name, id } = response.data;
			console.log(`Student repository ${name} was created`);
			return id;
		} catch (error) {
			console.error('An error occurred:', error.response.data);
			return false;
		}
	}

	createAccessToken = async(projectId) => {

		const today = new Date(); // Get the current date
		const sixMonthsLater = new Date(today); sixMonthsLater.setMonth(sixMonthsLater.getMonth() + 6);

		const payload = JSON.stringify({
			name: this.studentTokenName,
			scopes: [
				"read_repository",
				"write_repository"
			],
			access_level: 30,
			expires_at: sixMonthsLater.toISOString().slice(0, 10)
		});

		try {
			const response = await axios.post(`${GITLAB_API_URL}/projects/${projectId}/access_tokens`, payload, {
				headers
			});
			console.log(`Access token for project ${projectId} was created`);
			const { token } = response.data;
			return token 

		} catch (error) {
			console.error(`Failed to create access token for project ${projectId}`, error.response.data);
			return false;
		}
	}

	gitSetOrigin = async() => {
		try {
			await this.repo.removeRemote('origin').catch(_ => {});
			await this.repo.addRemote('origin', this.remote);
		} catch (error) {
			console.error('Failed to set repo remote:', error);
		}
	}

	gitCommit = async() => {
		await this.repo.add('.');
		// Check for changes in the staging area
		const diffSummary = await this.repo.diffSummary(['--cached']);
		if (diffSummary.files.length > 0) {
			// Create a random commit message
			const commitMessage = uuidv4().substring(0, 8);
			// Commit changes
			try {
				await this.repo
					.addConfig('user.name', 'remrob_bot') // mark the commit as made by the bot
					.commit(commitMessage)
					.addConfig('user.name', this.firstName);
			} catch (error) {
				console.error('Failed to commit:', error);
			}
		}
	}

	gitPushUpstream = async() => {
		try {
			await this.gitCommit();
			const remoteWithAuth = this.constructRemoteWithAuth()
			console.log(remoteWithAuth)
			await this.repo.push(remoteWithAuth, ['--set-upstream', 'master']);
			return true;
		} catch (error) {
			console.error('Failed initial push to remote:', error);
			return false;
		}
	}

	gitPull = async() => {
		try {
			await this.repo.pull('origin', 'master');
		} catch {

		}
	}

	gitPush = async() => {
		try {
		 	await this.gitCommit();
			const remoteWithAuth = this.constructRemoteWithAuth();
			await this.repo.push(remoteWithAuth);
		} catch {
			console.log('End of session push failed')
		}
	}

	// gitClone = async(force = false) => {

	// 	const newRepo = simpleGit();
	
	// 	if (fs.existsSync(this.repoPath)) {
	// 		if (force) {
	// 			// Delete the folder (equivalent of rm -r) and clone again
	// 			fs.rmdirSync(this.repoPath, { recursive: true });
	// 			newRepo.clone(this.repoUrl, this.repoPath);
	// 		} else {
	// 			newRepo.cwd(this.repoPath).pull();
	// 		}
	// 	} else {
	// 		newRepo.clone(this.repoUrl, this.repoPath);
	// 	}
	// }
}


module.exports = gitMaster