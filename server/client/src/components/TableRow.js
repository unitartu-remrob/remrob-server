import React, { useState, useEffect } from 'react';
import axios from "axios";
import {
	TableCell,
	TableRow,
	ButtonGroup,
	Button
} from '@mui/material';


const baseUrl = `http://localhost/api/container`;

const getUptime = (date) => {
	const startTime = new Date(date);
	const currentTime = new Date();
	const diff = (currentTime - startTime) / 3600000;
	const floor = Math.floor(diff)
	const hours = floor;
	const minutes = Math.round((diff - floor) * 60);
	return (hours != 0) ? `${hours} h ${minutes} min`
					: minutes < 1 ? '<1 min' : `${minutes} min`
}


const ContainerRow = ({container, reload}) => {

	const [vncLink, setVncLink] = useState("");
	// const [cpu_perc, setCpuPerc] = useState("-");

	// Handle non-existing and existing containers both
	// ----------------------------------------------------
	const { id, data } = container;
	let Status,
			StartedAt,
			IPAddress

	if (data.statusCode === 404) {
		Status = "disconnected";
	} else {
		({ Status, StartedAt } = data.State);

		const network = Object.keys(data.NetworkSettings.Networks)[0];
		({ IPAddress } = data.NetworkSettings.Networks[network]) // what is this abomination :D
	}
	const INACTIVE = (Status === "exited" || Status === "disconnected");
	// ----------------------------------------------------

	const startContainer = async (id) => {
		try {
			const res = await axios.post(`${baseUrl}/start/${id}`);
			const { data } = res;
			setVncLink(data.link);
			reload()
		} catch (e) {
			console.error(e);
		}
	}

	const getVncLink = async (id) => {
		try {
			const res = await axios.get(`${baseUrl}/connection/${id}`);
			const { data } = res;
			console.log("fetched vncLink")
			console.log(data.link)
			setVncLink(data.link);
		} catch (e) {
			console.error(e);
		}
	}

	// const getCpuPerc= async (id) => {
	// 	try {
	// 		const res = await axios.get(`${baseUrl}/cpu/${id}`);
	// 		const { data } = res;
	// 		setCpuPerc(data.cpu_percentage);
	// 	} catch (e) {
	// 		console.error(e);
	// 	}
	// }
	
	const stopContainer = async (id) => {
		try {
			const res = await axios.post(`${baseUrl}/stop/${id}`);
			const { data } = res;
			reload();
		} catch (e) {
			console.error(e);
		}
	}

	useEffect(() => {
		getVncLink(id);
		// setInterval(function() {
		// 	getCpuPerc(id);
		// }, 1000);
	}, [])


	return (
		<TableRow>
			<TableCell component="th" scope="row">
				{id /* {Names[0].replace('/','')} */}
			</TableCell>
			<TableCell align="left">{Status}</TableCell>
			<TableCell align="left">{!INACTIVE ? getUptime(StartedAt): '-'}</TableCell>
			<TableCell align="left">{IPAddress}</TableCell>
			{/* <TableCell align="left">{cpu_perc}</TableCell> */}
			<TableCell align="right">
				<ButtonGroup sx={{paddingRight: "2rem"}}>
					<Button
						onClick={() => startContainer(id)}
						color="success"
						disabled={!INACTIVE}>Start</Button>
					<Button
						onClick={() => stopContainer(id)}
						color="warning"
						disabled={INACTIVE}>Stop</Button>
				</ButtonGroup>
				<ButtonGroup>
				<Button
					target="_blank"
					href={vncLink}
					color="primary"
					disabled={INACTIVE}
					>Connect</Button>
				</ButtonGroup>
			</TableCell>
		</TableRow>
	);
}
 
export default ContainerRow;