import React, { useState, useEffect } from 'react';
import axios from "axios";
import {
	TableCell,
	TableRow,
	ButtonGroup,
	Button
} from '@mui/material';


const ContainerRow = ({container, reload}) => {

	const { index, Names, State, Status } = container;

	const [vncLink, setVncLink] = useState("");

	const baseUrl = `http://localhost/api/container`;

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
		getVncLink(index)
	}, []);

	return (
		<TableRow>
			<TableCell component="th" scope="row">
				{Names[0].replace('/','')}
			</TableCell>
			<TableCell align="left">{State}</TableCell>
			<TableCell align="left">{Status}</TableCell>
			<TableCell align="right">
				<ButtonGroup sx={{paddingRight: "2rem"}}>
					<Button
						onClick={() => startContainer(index)}
						color="success"
						disabled={State === "running"}>Start</Button>
					<Button
						onClick={() => stopContainer(index)}
						color="warning"
						disabled={State === "exited"}>Stop</Button>
				</ButtonGroup>
				<ButtonGroup>
				<Button
					target="_blank"
					href={vncLink}
					color="primary"
					disabled={State === "exited"}
					>Connect</Button>
				</ButtonGroup>
			</TableCell>
		</TableRow>
	);
}
 
export default ContainerRow;