import * as React from 'react';
import {
	Table,
	TableBody,
	TableCell,
	TableContainer,
	TableHead,
	TableRow,
} from '@mui/material';
import ContainerRow from './TableRow';

export default function AcccessibleTable({containers, reload}) {
  return (
    <TableContainer>
      <Table sx={{ minWidth: 650 }} aria-label="caption table">
        <TableHead>
          <TableRow>
            <TableCell>Container</TableCell>
            <TableCell align="left">Status</TableCell>
            <TableCell align="left">Uptime</TableCell>
            <TableCell align="left">Ip</TableCell>
            {/* <TableCell align="left">%CPU</TableCell> */}
            <TableCell align="center">Actions</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {containers.map(container => {
						return <ContainerRow
							key={container.id}
							container={container}
							reload={reload}/>
          })}
        </TableBody>
      </Table>
    </TableContainer>
  );
}
