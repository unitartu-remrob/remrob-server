import config from 'config';

export const ROBOT_INVENTORY_TABLE = 'inventory';
export const SIMTAINER_INVENTORY_TABLE = 'simulation_containers';

export const BOOKING_TABLE = 'bookings';
export const USER_TABLE = 'user';

export const ROS_VERSION_NOETIC = 'noetic';
export const ROS_VERSION_JAZZY = 'jazzy';

export const JAZZY_STARTUP_DELAY = 10000; // ms

export const LOCALROB_APP = config.get('LocalRemrob');
