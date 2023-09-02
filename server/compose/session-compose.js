var path = require('path');
const fs = require("fs");
const { exec } = require("child_process");

var db = require("../data/db.js");
const gitMaster = require('./code_upload/git-master.js');

const {
	createShareLink
} = require('./ownclouder.js');

class SessionCompose {
		constructor({
			containerId,
			authHeader, // REF for talking to flask
			isAdmin,
			userId,
			userBooking,
			is_simulation,
			useBaseImage
		}) {
			this.containerId = containerId;
			this.authHeader = authHeader;
			this.isAdmin = isAdmin;
			this.userId = userId;
			this.userBooking = userBooking;
			this.is_simulation = is_simulation;
			this.useBaseImage = useBaseImage;

			this.inventoryTable = this.getInventoryTable();
			this.sessionType = this.getSessionType();

			this.cleanFolderName = null;
			this.extFolderName = null;
			// this.workspaceMountPath = null;
			// this.submissionMountPath = null;
			// this.gitMountPath = null;

			// this.volumes = [];
			this.environment = [];
		}

		getInventoryTable = () => {
			return (this.is_simulation) ? 'simulation_containers' : 'inventory';
		}

		getSessionType = () => {
			// returns the folder name of the session type (contains the corresponding session templates)
			return (this.is_simulation) ? "local" : "macvlan"; 
		}

		cleanChars = (name) => {
			// Remove crazy characters from name
			return name.replace(/[\x00-\x08\x0E-\x1F\x7F-\uFFFF]/g, '').replace(/\s/g,'')
		}

		getUserName = async () => {
			const user_data = await db('user').first().where({ id: this.userId }).select(['first_name', 'last_name']);
			let { first_name, last_name } = user_data;
			first_name = first_name ?? "Roberta";
			last_name = last_name ?? "Roos";
			return { first_name, last_name }
		}

		createShare = () => {
			// will make POST request to owncloud requesting that the folder named `$extFolderName` under the /remrob directory be shared with a token
			// the returned access token is stored in the db, later it is included in a URL issued to user for accessing their owncloud submission folder
			createShareLink(this.extFolderName).then(userToken => {
				if (userToken !== undefined) {
					db('user').update({
						owncloud_id: userToken
					}).where({
						id: this.userId
					}).then(res => {
					}).catch(e => console.log(e));
				}
			});
		}

		setVolumeMounts = async (composeData) => {
			const { services: { vnc: { volumes } } } = composeData;
			// this.volumes = volumes;
			// this.environment = environment;
		
			const { first_name, last_name } = await this.getUserName(this.userId);
		
			const clean_name = this.cleanChars(first_name);
			const clean_surname = this.cleanChars(last_name);
		
			const containerRepoName = `${first_name}-${last_name}`;

			this.cleanFolderName = `${clean_name}-${clean_surname}-${this.userId}`;
			this.extFolderName = `${first_name}-${last_name}-${this.userId}`;

			// Set the workspace mount:
			// ----------------------------------
			// Workspace mount uses the cleaned name and surname so the workspace cp command works
			const workspaceMountPath = `${process.env.WORKSPACE_ROOT}/${this.cleanFolderName}`
			await this.createWorkspaceFolder(workspaceMountPath);
			volumes.push(`${workspaceMountPath}/catkin_ws:/home/kasutaja/catkin_ws`)

			// Set the git repo mount:
			// ----------------------------------
			const prettyName = `${first_name} ${last_name}`;
			await this.setGitRepository(prettyName);
			
			// const gitMountPath = `${process.env.REPOS_ROOT}/${this.cleanFolderName}`
			// volumes.push(`${gitMountPath}:/home/kasutaja/${containerRepoName}`) // this is unnecessary when the git repo is within the catkin_ws

			// Set the video submission mount:
			// ----------------------------------
			const submissionMountPath = `${process.env.OWNCLOUD_ROOT}/${this.extFolderName}`;
			const isSet = await this.setSubmissionMount(submissionMountPath); // will create the submission directory in case it doesn't exist
			// If the folder is not accessible (problems with sync), the user will not be able to submit videos in that session
			if (isSet) {
				volumes.push(`${submissionMountPath}:/home/kasutaja/Submission`)
			}
			
			return  { volumes, volEnv: this.environment }
		}

		createWorkspaceFolder = (mountDir) => {
			return new Promise((resolve, reject) => {
				if (!fs.existsSync(mountDir)) {
					// create dir if first time connecting
					fs.mkdirSync(mountDir, { recursive: true });
					// Copy a clean catkin_ws to the user's folder
					exec(`cp -r ${process.env.BASE_WS_ROOT}/catkin_ws ${mountDir}`, (error, stdout, stderr) => {
						if (error) {
								console.log(`error: ${error.message}`);
								reject("failed to initialize workspace folder")
						}
						if (stderr) {
								console.log(`stderr: ${stderr}`);
								reject("failed to initialize workspace folder")
						}
						// console.log(`Workspace copied, stdout: ${stdout}`);
						resolve("workspace folder initialized")
					});
				} else {
					resolve("workspace folder already exists")
				}
			})
		}

		setSubmissionMount = async(mountDir) => {
			if (!fs.existsSync(mountDir)){
				// If the user folder doesn't exist, we haven't made it before and it means user is connecting for the first time
				// So let's create a directory with submodule folders for them
				try {
					fs.mkdirSync(mountDir);
				} catch {
					console.log("sync broken")
					return false
				}
				// Create submission folders (assumption of 6 modules)
				const moduleCount = [ ...Array(6).keys() ].map( i => i + 1 );
				await Promise.all(
					moduleCount.map(i => fs.mkdir(`${mountDir}/solutions-module-${i}`, () => undefined))
				)	
				// Now we need to make a POST request to the owncloud servers and ask them to share the folder we just created.
				// The sync occurs every 5 minutes, so that is the min time we will wait before requesting the share
				setTimeout(() => {
					this.createShare()
				}, 310000)
			} else if (!this.isAdmin) {
				// directory exists
				// check if the user has a share token (if for some reason the previous share request failed)
				// this runs every time, so not very efficient, should be done elsewhere
				const { owncloud_id } = await db('user').first().where({ id: this.userId }).select(['owncloud_id']);
				if (owncloud_id === null  || owncloud_id === "") {
					this.createShare()
				}
			}
			return true
		}

		setGitRepository = async(prettyName) => {

			let isFirstSession = false;
			let {
				git_token: studentGitToken,
				user_repo: repoHostName,
				project_id: projectId,
				first_name: firstName,
				email
			} = await db('user').first().where({ id: this.userId }).select([
				'git_token', 'user_repo', 'project_id', 'first_name', 'email'
			]);

			if (repoHostName === null) {
				// if there's no project ID, it means there's no initialized repo for the user
				isFirstSession = true;
				repoHostName = this.cleanFolderName;
			}
			this.gitMaster = new gitMaster(repoHostName, firstName, email);
			console.log(this.gitMaster.repoPath)

			// Initialize the repository on the remote, and retrieve the access token
			if (projectId === null) {
				projectId = await this.gitMaster.initRemoteRepo(prettyName);
				if (projectId) {
					db('user').update({
						project_id: projectId
					}).where({
						id: this.userId
					}).then(res => {
						
					}).catch(e => console.log(e));
				
				} else {
					console.log(`remote for ${repoHostName} already exists, attempt first commit again`)
				}
			}

			if (studentGitToken === null) {
				// if the token is missing, attempt to create one
				studentGitToken = await this.gitMaster.createAccessToken(projectId);
				if (studentGitToken) {
					db('user').update({
							git_token: studentGitToken
						}).where({
							id: this.userId
						}).then(res => {
				
						}).catch(e => console.log(e));
				} else {
					console.log(`access token creation for ${repoHostName} failed`)
				}
			}

			this.gitMaster.studentGitToken = studentGitToken;

			if (isFirstSession) {
				try {
					// now set the origin (for <user>/catkin_ws/src) to the remote repository and push the initial commit
					await this.gitMaster.gitSetOrigin();
					const pushSuccessful = await this.gitMaster.gitPushUpstream();
					if (pushSuccessful) {
						// if everything passes without error mark in the database that the user has a repo now
						db('user').update({
								user_repo: repoHostName
							}).where({
								id: this.userId
							}).then(res => {
					
							}).catch(e => console.log(e));
						console.log(`initial commit and push successful for ${repoHostName}`) 
					}
				} catch (error) {
					console.error('Failed the initial push to remote:', error);
				}
			} else {
				// if user has decided to make their own commits from outside a remrob session
				this.gitMaster.gitPull();
				// this.gitMaster.gitPushUpstream();
			}
		}
}

module.exports = SessionCompose;