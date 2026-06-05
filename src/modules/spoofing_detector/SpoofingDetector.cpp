#include "SpoofingDetector.hpp"

//connects the module to commands
ModuleBase::Descriptor SpoofingDetector::desc{
	SpoofingDetector::task_spawn,
	SpoofingDetector::custom_command,
	SpoofingDetector::print_usage
};

//initializes the ScheduledWorkItem
SpoofingDetector::SpoofingDetector() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

int SpoofingDetector::task_spawn(int argc, char *argv[])
{
	SpoofingDetector *obj = new SpoofingDetector();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	desc.object.store(obj);
	desc.task_id = task_id_is_work_queue;

	if (!obj->start()){
		delete obj;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return -1;
	}

	return 0;
}

int SpoofingDetector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SpoofingDetector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Carrier phase and IMU based GNSS spoofing detector.

The module subscribes to sensor_gps_raw and vehicle_imu. It collects windowed
measurements and can print a warning when spoofing is detected.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("spoofing_detector", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the spoofing detector");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int SpoofingDetector::print_status()
{
	// print status
	return 0;
}

bool SpoofingDetector::start()
{
	if (!_sensor_gps_raw_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void SpoofingDetector::addGpsSample(const sensor_gps_raw_s &gps_raw)
{
	if (_gps_buffer_count < GPS_BUFFER_SIZE) {
		_gps_buffer_count++;
	} else {
		for (int i = 1; i < GPS_BUFFER_SIZE; i++) {
			_gps_buffer[i - 1] = _gps_buffer[i];
		}
	}

	const int index = _gps_buffer_count - 1;

	_gps_buffer[index].timestamp_sample = gps_raw.timestamp_sample;

	for (int i = 0; i < 32; i++) {
		_gps_buffer[index].carrier_phase[i] = gps_raw.carrier_phase[i];
		_gps_buffer[index].flags[i] = gps_raw.flags[i];

		//checking for the CP_VALID and HALF_CYCLE flags
		_gps_buffer[index].carrier_phase_valid[i] =
			(gps_raw.flags[i] & sensor_gps_raw_s::FLAG_CP_VALID)
			&& (gps_raw.flags[i] & sensor_gps_raw_s::FLAG_HALF_CYCLE);

		//every time a new carrier phase value is added its detrended value should be set to zero
		_gps_buffer[index].detrended_carrier_phase[i] = 0.0;
		_gps_buffer[index].detrended_valid[i] = false;
	}

}

void SpoofingDetector::addImuSample(const vehicle_imu_s &imu)
{
	if (_imu_buffer_count < IMU_BUFFER_SIZE) {
		_imu_buffer_count++;
	} else {
		for (int i = 1; i < IMU_BUFFER_SIZE; i++) {
			_imu_buffer[i - 1] = _imu_buffer[i];
		}
	}

	const int index = _imu_buffer_count - 1;

	_imu_buffer[index].timestamp_sample = imu.timestamp_sample;
	_imu_buffer[index].delta_velocity_dt = imu.delta_velocity_dt;

	for (int i = 0; i < 3; i++) {
		_imu_buffer[index].delta_velocity[i] = imu.delta_velocity[i];
	}
}

bool SpoofingDetector::windowReady()
{
	return _gps_buffer_count >= GPS_BUFFER_SIZE && _imu_buffer_count > 0;
}

void SpoofingDetector::detrendSample()
{
	const int index = _gps_buffer_count - 1;

	//loop through every satellite
	for (int i = 0; i < 32; i++) {
		double sum = 0.0;
		int count = 0;

		//add 20 samples together
		for (int j = 0; j < GPS_BUFFER_SIZE; j++) {

			//only add sample if the flags are on
			if (_gps_buffer[j].carrier_phase_valid[i]) {
				sum += _gps_buffer[j].carrier_phase[i];
				count++;
			}
		}

		if (count > 0 && _gps_buffer[index].carrier_phase_valid[i]) {
			const double mean = sum / count;
			const double current = _gps_buffer[index].carrier_phase[i];

			_gps_buffer[index].detrended_carrier_phase[i] = current - mean;
			_gps_buffer[index].detrended_valid[i] = true;

		} else {
			_gps_buffer[index].detrended_carrier_phase[i] = 0.0;
			_gps_buffer[index].detrended_valid[i] = false;
		}
	}
}

bool SpoofingDetector::runSpoofingDetection()
{
	return false;
}

void SpoofingDetector::Run()
{
	if (should_exit()) {
		_sensor_gps_raw_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_imu_s imu;

	while (_vehicle_imu_sub.update(&imu)) {
		addImuSample(imu);
	}

	sensor_gps_raw_s gps_raw;

	while (_sensor_gps_raw_sub.update(&gps_raw)) {
		addGpsSample(gps_raw);

		if (windowReady()) {
			detrendSample();
			bool spoofed = runSpoofingDetection();

			if (spoofed && !_spoofing_detected_prev) {
				PX4_WARN("GNSS spoofing detected");
			}

			_spoofing_detected_prev = spoofed;
		}
	}
}

extern "C" __EXPORT int spoofing_detector_main(int argc, char *argv[])
{
	return ModuleBase::main(SpoofingDetector::desc, argc, argv);
}
