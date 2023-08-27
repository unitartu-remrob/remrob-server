var path = require('path');
const fs = require("fs");
const { exec } = require("child_process");
const { v4: uuidv4 } = require('uuid');
var db = require("../data/db.js");

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
				db('user').update({
					owncloud_id: userToken
				}).where({
					id: this.userId
				}).then(res => {
				}).catch(e => console.log(e));
			})
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
			this.createWorkspaceFolder(workspaceMountPath);
			volumes.push(`${workspaceMountPath}/catkin_ws:/home/kasutaja/catkin_ws`)

			// Set the video submission mount:
			// ----------------------------------
			const submissionMountPath = `${process.env.OWNCLOUD_ROOT}/${this.extFolderName}`;
			const isSet = await this.setSubmissionMount(submissionMountPath); // will create the submission directory in case it doesn't exist
			// If the folder is not accessible (problems with sync), the user will not be able to submit videos in that session
			if (isSet) {
				volumes.push(`${submissionMountPath}:/home/kasutaja/Submission`)
			}

			// Set the git repo mount:
			// ----------------------------------
			// const gitMountPath = `${process.env.REPOS_ROOT}/${this.extFolderName}`
			// const gitAuthToken = await setGitRepository(gitMountPath);
		
			// this.environment.push(`GIT_PAT=${gitAuthToken}`)
			// volumes.push(`${gitMountPath}:/home/kasutaja/${containerRepoName}`)
			
			return  { volumes, volEnv: this.environment }
		}

		createWorkspaceFolder = (mountDir) => {
			if (!fs.existsSync(mountDir)) {
				// create dir if first time connecting
				fs.mkdirSync(mountDir, { recursive: true });
				// Copy a clean catkin_ws to the user's folder
				exec(`cp -r ${process.env.BASE_WS_ROOT}/catkin_ws ${mountDir}`, (error, stdout, stderr) => {
					if (error) {
							console.log(`error: ${error.message}`);
							return;
					}
					if (stderr) {
							console.log(`stderr: ${stderr}`);
							return;
					}
					console.log(`stdout: ${stdout}`);
				});
			}
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
				const token = await db('user').first().where({ id: this.userId }).select(['owncloud_id']);
				if (token === null) {
					this.createShare()
				}
			}
			return true
		}

		setGitRepository = async(mountPath) => {

			const repoHostName = this.cleanFolderName; 
			
			if (!fs.existsSync(mountPath)) {
				// Authentication token for pushing from inside the container (only works on macvlan)
				const git_auth_token = uuidv4();
				// Update the db with the session token and repo name
				// ! user_repo currently not used anywhere, might be redundant data !
				db('user').update({
						git_token: git_auth_token,
						user_repo: repoHostName
					}).where({
						id: this.userId
					}).then(res => {
			
					}).catch(e => console.log(e));

				
				await axios.get(`${process.env.DB_SERVER}/check_user`, {
					params: {
						'user_name': repoHostName
					},
					headers: HEADERS
				}).then(async res => {
					console.log("Created new user repo")
					// Repo created on GitLab, clone it
					await axios.get(`${process.env.DB_SERVER}/clone_jwt`,
					{
						params: {
							'user_name': repoHostName
						},
						headers: HEADERS
					}).catch(e => console.log(e));
				}).catch(e => console.log(e));
		
				return git_auth_token
		
			} else {
				const { git_token } = await db('user').first().where({ id: this.userId }).select(['git_token']);
				return git_token
			}
		}
}

module.exports = SessionCompose;