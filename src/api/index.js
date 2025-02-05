import express from 'express';

const router = express.Router();

import {
	authenticateJWT,
	authenticateAdminJWTWebSocket,
	checkSession,
	checkContainerOwnership,
} from './middleware/auth.js';

import { liveStats, robotMonitor } from '../docker/containerMonitor.js';

import assignContainer from './routes/assignContainer.js';

import startContainer from './routes/container/startContainer.js';
import inspectContainer from './routes/container/inspectContainer.js';
import containerStats from './routes/container/containerStats.js';
import stopContainer from './routes/container/stopContainer.js';
import removeContainer from './routes/container/removeContainer.js';

const userHasActiveSessionOrIsAdmin = [authenticateJWT, checkSession, checkContainerOwnership];

export const mountWsRoutes = () => {
	router.ws('/live/:version', authenticateAdminJWTWebSocket, liveStats);
	router.ws('/robot-status/:id', robotMonitor);
};

router.get('/assign', [authenticateJWT, checkSession], assignContainer);

router.get('/inspect/:id', userHasActiveSessionOrIsAdmin, inspectContainer);
router.get('/stats/:id', userHasActiveSessionOrIsAdmin, containerStats);
router.post('/start/:id', userHasActiveSessionOrIsAdmin, startContainer);
router.post('/stop/:id', userHasActiveSessionOrIsAdmin, stopContainer);
router.post('/remove/:id', userHasActiveSessionOrIsAdmin, removeContainer);

export default router;
