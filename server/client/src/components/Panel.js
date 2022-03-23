import React, { useState, useEffect } from 'react';
import Table from './Table';
import axios from "axios";
import IconButton from '@mui/material/IconButton';
import CachedIcon from '@mui/icons-material/Cached';


const Panel = () => {

	const [isLoading, setIsLoading] = useState(true);
  const [containers, setContainers] = useState([]);

	const listContainers = async () => {
		try {
			const res = await axios.get("http://localhost/api/container/list");
			const { data } = res;
			console.log(res);
			setContainers(data);
			setIsLoading(false)
		} catch (e) {
			console.error(e);
		}
	}
	
	// Initial render
	useEffect(() => {
		listContainers();
	}, []);

  return (
		<React.Fragment>
			<IconButton
				sx={{float: "right"}}
				onClick={listContainers}>
				<CachedIcon fontSize="large"/>
			</IconButton>
			{isLoading ? (
				<h1>Loading...</h1>
			) : (
				<Table {...{containers}} reload={listContainers}/>
			)}
		</React.Fragment>
  );
}
 
export default Panel;