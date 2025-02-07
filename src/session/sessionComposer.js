import fs from 'fs';
import path from 'path';
import writeYamlFile from 'write-yaml-file';
import readYamlFile from 'read-yaml-file';
import generator from 'generate-password';
import { exec } from 'child_process';

import config from 'config';
import { USER_TABLE, ROS_VERSION_JAZZY } from '../constants.js';
import db from '../data/db.js';
import { getInventoryTable, getRobotCell } from './inventory.js';
import {
	createOwncloudShareLink,
	getOwncloudShareLink,
	updateOwncloudShareLink,
} from './owncloud.js';

const __dirname = import.meta.dirname;
const composeRootPath = path.resolve(__dirname, '../../compose');

const DEFAULT_IMAGE = config.get('AvailableImages').find((image => image.default === true));

const isRecordingSubmissionsEnabled = config.get('RecordingSubmissionsEnabled');
const userWorkspacesEnabled = config.get('UserWorkspacesEnabled');

export const composeContainerConfig = async (composeParams) => {
	const composeFilePath = await new SessionCompose(composeParams).generateComposeFile();

	return {
		cwd: composeFilePath,
		composeOptions: ['--file', `docker-compose.yaml`],
		log: true,
	};
};

export const getVncUrl = async (containerId, isSimtainer) => {
	const inventoryTable = getInventoryTable(isSimtainer);

	const { vnc_uri } = await db(inventoryTable)
		.first()
		.where({
			slug: containerId,
		})
		.select('vnc_uri');

	return vnc_uri;
};

class SessionCompose {
	constructor({ containerId, userId, userBooking, isAdmin, isSimtainer, rosVersion, imageTag }) {
		this.containerId = containerId;
		this.userId = userId;
		this.userBooking = userBooking;
		this.isAdmin = isAdmin;
		this.isSimtainer = isSimtainer;

		this.rosVersion = rosVersion ?? DEFAULT_IMAGE.rosVersion;
		this.imageTag = imageTag ?? DEFAULT_IMAGE.imageTag;

		this.inventoryTable = getInventoryTable(isSimtainer);
		this.sessionType = this.getSessionType(isSimtainer);
		this.extFolderName = null;
		this.composeData = null;
	}

	generateComposeFile = async () => {
		// create a separate directory for the generated compose file
		const composeDir = path.join(
			composeRootPath,
			`${this.sessionType}/temp/${this.containerId}`
		);

		if (!fs.existsSync(composeDir)) {
			fs.mkdirSync(composeDir);
		}

		// read in the template
		

		const composeFileHandle = path.join(
			composeRootPath,
			`${this.sessionType}/${this.rosVersion}/${this.containerId}.yaml`
		);

		this.composeData = await readYamlFile(composeFileHandle);
		await this.setEnvironment();
		await this.setVolumeMounts();

		// write out the compose file to be used for a session
		const tempFile = path.join(
			composeRootPath,
			`${this.sessionType}/temp/${this.containerId}/docker-compose.yaml`
		);

		await writeYamlFile(tempFile, this.composeData);

		return composeDir;
	};

	getSessionType = (isSimtainer) => {
		// returns the folder name of the session type (contains the corresponding session templates)
		return isSimtainer ? 'local' : 'macvlan';
	};

	getWorkspaceMountPath = (folderName) => {
		return `${process.env.WORKSPACE_ROOT}/${folderName}`;
	};

	cleanChars = (name) => {
		// allow only alphanumeric characters, underscores, and hyphens
		return name.replace(/[^a-zA-Z0-9_-]/g, '');
	};

	updateVNCLink = async (vncUri) => {
		await db(this.inventoryTable).where({ slug: this.containerId }).update({
			vnc_uri: vncUri,
		});
	};

	getUserName = async () => {
		let { first_name, last_name } = await db(USER_TABLE)
			.first()
			.where({ id: this.userId })
			.select(['first_name', 'last_name']);

		first_name = first_name ?? 'Roberta';
		last_name = last_name ?? 'Roos';

		return { first_name, last_name };
	};

	setEnvironment = async () => {
		const {
			services: {
				vnc: { environment },
			},
		} = this.composeData;

		// generate a VNC password token
		const passwordToken = generator.generate({
			length: 20,
			numbers: true,
		});

		const passwordEnv = `PASSWORD=${passwordToken}`;
		const pwi = environment.findIndex((env) => env.includes('PASSWORD'));

		if (pwi >= 0) {
			environment[pwi] = passwordEnv;
		} else {
			environment.push(passwordEnv);
		}

		if (!this.isSimtainer) {
			const cell = await getRobotCell(this.containerId);
			environment.push(`ROBOT_CELL=${cell}`);
		}

		if (this.rosVersion === ROS_VERSION_JAZZY) {
			// ...get domain ID
			const rosDomainId = 42
			environment.push(`ROS_DOMAIN_ID=${rosDomainId}`);
		}

		this.composeData.services.vnc.image = this.imageTag;
		this.composeData.services.vnc.environment = environment;

		const vncUri = generateUrl(this.containerId, passwordToken);
		await this.updateVNCLink(vncUri);
	};

	setVolumeMounts = async () => {
		const {
			services: {
				vnc: { volumes },
			},
		} = this.composeData;

		const { first_name, last_name } = await this.getUserName(this.userId);

		const clean_name = this.cleanChars(first_name);
		const clean_surname = this.cleanChars(last_name);

		const cleanFolderName = `${clean_name}-${clean_surname}-${this.userId}`;
		this.extFolderName = `${first_name}-${last_name}-${this.userId}`;

		if (userWorkspacesEnabled) {
			// workspace mount uses the cleaned name and surname so the workspace cp command works
			const workspaceMountPath = `${process.env.WORKSPACE_ROOT}/${cleanFolderName}`;
			try {
				await this.createWorkspaceFolder(workspaceMountPath);
				volumes.push(`${workspaceMountPath}/catkin_ws:/home/kasutaja/catkin_ws`);
			} catch (e) {
				console.log(`Failed to mount user workspace for ${this.userId}: ${e.message}`);
			}
		}

		if (isRecordingSubmissionsEnabled) {
			const submissionMountPath = `${process.env.OWNCLOUD_ROOT}/${this.extFolderName}`;
			const isSet = await this.setSubmissionMount(submissionMountPath); // will create the submission directory in case it doesn't exist

			// if the folder is not accessible (problems with sync), the user will not be able to submit videos in that session
			if (isSet) {
				volumes.push(`${submissionMountPath}:/home/kasutaja/Submission`);
			}
		}

		this.composeData.services.vnc.volumes = volumes;
	};

	createWorkspaceFolder = (mountDir) => {
		return new Promise((resolve, reject) => {
			if (!fs.existsSync(mountDir)) {
				// create workspace dir if first time connecting
				fs.mkdirSync(mountDir, { recursive: true });
				// copy a clean catkin_ws to the user's folder
				exec(
					`cp -r ${process.env.BASE_WS_ROOT}/catkin_ws ${mountDir}`,
					(error, _, stderr) => {
						if (error) {
							reject(error);
						}
						if (stderr) {
							reject(stderr);
						}
						resolve('Workspace folder initialized');
					}
				);
			} else {
				resolve('Workspace folder already exists');
			}
		});
	};

	setSubmissionMount = async (mountDir) => {
		if (!fs.existsSync(mountDir)) {
			// if the user folder doesn't exist it means the user is connecting for the first time
			try {
				fs.mkdirSync(mountDir);
			} catch {
				console.log(
					'Failed to create user homework submission folder: Owncloud sync broken'
				);

				return false;
			}
			// create separate submission folders (assumption of 6 modules)
			const moduleCount = [...Array(6).keys()].map((i) => i + 1);
			await Promise.all(
				moduleCount.map((i) => fs.mkdir(`${mountDir}/solutions-module-${i}`))
			);

			// sync occurs every 5 minutes, so use it as the min wait time before requesting the cloud share
			setTimeout(() => {
				this.initHomeworkSubmissionFolder();
			}, 310000);
		} else { // submission folder exists for the user

			// check if the user has a share token in case the previous share request failed due to sync issues
			const owncloud_id = getOwncloudShareLink(this.userId);

			// if the user is missing the share token, request a new one
			if (owncloud_id === null || owncloud_id === '') {
				this.initHomeworkSubmissionFolder();
			}
		}

		return true;
	};

	initHomeworkSubmissionFolder = async () => {
		const ownCloudShareToken = await createOwncloudShareLink(this.extFolderName);

		if (ownCloudShareToken !== undefined) {
			updateOwncloudShareLink(ownCloudShareToken, this.userId);
		}
	};
}

const generateUrl = (containerId, sessionToken) => {
	const vncUrl = new URL(`http:/localhost/novnc/vnc.html`);
	vncUrl.searchParams.append('autoconnect', 'true');
	vncUrl.searchParams.append('resize', 'remote');
	vncUrl.searchParams.append('password', sessionToken);

	// order for the following matters
	vncUrl.searchParams.append('path', 'novnc');
	vncUrl.href = vncUrl.href.concat(`?token=${containerId}`);

	return vncUrl.pathname + vncUrl.search; // hostname excluded in return
};
