import express from 'express';

const router = express.Router();

import {
	authenticateJWT,
	authenticateAdminJWTWebSocket,
	checkSession,
	checkContainerOwnership,
	authenticatePublicSessionCookie,
} from './middleware/auth.js';

import { liveStats, robotMonitor } from '../docker/containerMonitor.js';

import assignContainer from './routes/inventory/assignContainer.js';

import getPublicContainers from './routes/inventory/public/getPublicContainers.js';
import claimPublicContainer from './routes/inventory/public/claimPublicContainer.js';
import yieldPublicSession from './routes/inventory/public/yieldPublicSession.js';

import startPublicContainer from './routes/container/public/startContainer.js';
import stopPublicContainer from './routes/container/public/stopContainer.js';
import removePublicContainer from './routes/container/public/removeContainer.js';
import inspectPublicContainer from './routes/container/public/inspectContainer.js';

import getImages from './routes/getImages.js';

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

// Public endpoints
router.get('/images', getImages);
router.get('/public-containers', getPublicContainers);
router.get('/claim/:id', claimPublicContainer);
router.delete('/claim/:id', yieldPublicSession);

router.get('/inspect-public-container', authenticatePublicSessionCookie, inspectPublicContainer);
router.post('/start-public-container', authenticatePublicSessionCookie, startPublicContainer);
router.post('/stop-public-container', authenticatePublicSessionCookie, stopPublicContainer);
router.post('/remove-public-container', authenticatePublicSessionCookie, removePublicContainer);

// Authenticated user endpoints
router.get('/assign', [authenticateJWT, checkSession], assignContainer);

router.get('/inspect/:id', userHasActiveSessionOrIsAdmin, inspectContainer);
router.get('/stats/:id', userHasActiveSessionOrIsAdmin, containerStats);
router.post('/start/:id', userHasActiveSessionOrIsAdmin, startContainer);
router.post('/stop/:id', userHasActiveSessionOrIsAdmin, stopContainer);
router.post('/remove/:id', userHasActiveSessionOrIsAdmin, removeContainer);

export default router;
