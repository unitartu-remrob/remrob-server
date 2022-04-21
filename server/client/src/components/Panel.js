import React, { useState, useEffect } from 'react';
import Table from './Table';
import axios from "axios";
import IconButton from '@mui/material/IconButton';
import CachedIcon from '@mui/icons-material/Cached';


const baseUrl = `${process.env.REACT_APP_URL}/api/container`;
const Panel = () => {

	const ids = React.useRef()
	const [isLoading, setIsLoading] = useState(true);
  const [containers, setContainers] = useState({});

	// const listContainers = async () => {
	// 	try {
	// 		const res = await axios.get("http://localhost/api/container/list");
	// 		const { data } = res;
	// 		console.log(res);
	// 		setContainers(data);
	// 		setIsLoading(false)
	// 	} catch (e) {
	// 		console.error(e);
	// 	}
	// }
	
	// // Initial render
	// useEffect(() => {
	// 	listContainers();
	// }, []);

	const getAvailableContainers = async () => {
		try {
			const res = await axios.get(`${baseUrl}/names`);
			const { names } = res.data;
			console.log("loaded ids");
			return names
		} catch (e) {
			console.error(e);
		}
	}
	
	const fetchAllContainers = async () => {
		try {
			const containerResults = [];
			await Promise.all(ids.current.map(async id => {
				const res = await axios.get(`${baseUrl}/inspect/${id}`);
				const { data } = res
				containerResults.push({id: id, data: data})
			}));
			containerResults.sort((a, b) => {
				// This totally depends on the {robo-${id}} format
				return (parseInt(a.id.slice(-1)) > parseInt(b.id.slice(-1))) ? 1 : -1;
			})
			setContainers(containerResults);
			console.log("containers fetched", containerResults);
			setIsLoading(false)
		} catch (e) {
			console.error(e);
		}
	}

	// Initial render
	useEffect(async () => {
		ids.current = await getAvailableContainers();
		fetchAllContainers();
	}, []);
	
  return (
		<React.Fragment>
			<IconButton
				sx={{float: "right"}}
				onClick={() => fetchAllContainers()}>
				<CachedIcon fontSize="large"/>
			</IconButton>
			{isLoading ? (
				<h1>Loading...</h1>
			) : (
				// <div>{JSON.stringify(containers)}</div>
				<Table {...{containers}} reload={fetchAllContainers}/>
			)}
		</React.Fragment>
  );
}
 
export default Panel;