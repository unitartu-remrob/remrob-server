const calculateCpuPercentage = (d) => {
	if (Object.keys(d.pids_stats).length === 0) {
		return 0;
	}

	const cpu_count = d['cpu_stats']['cpu_usage']['percpu_usage'].length;
	let cpu_percent = 0.0;
	const cpu_delta =
		d['cpu_stats']['cpu_usage']['total_usage'] - d['precpu_stats']['cpu_usage']['total_usage'];
	const system_delta = d['cpu_stats']['system_cpu_usage'] - d['precpu_stats']['system_cpu_usage'];

	if (system_delta > 0.0) {
		cpu_percent = (cpu_delta / system_delta) * 100.0 * cpu_count;
	}

	return cpu_percent.toFixed(2);
};

export default calculateCpuPercentage;
