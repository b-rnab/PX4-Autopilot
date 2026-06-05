//prevents the header from beign included twice
#pragma once

//gives the module support like start, stop, status, ModulBase, Descriptor etc
#include <px4_platform_common/module.h>
//Lets the module run in the work queue
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

//Let the module read uORB topics
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
//the message types that will be used in the module
#include <uORB/topics/sensor_gps_raw.h>
#include <uORB/topics/vehicle_imu.h>

//the spoofind detector is a module and an item in the work queue
class SpoofingDetector : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	//bookkeeping object for the module
	static Descriptor desc;

	//constructor and destructor
	SpoofingDetector();
	~SpoofingDetector() override = default;

	//commands in the shell
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	//the main function
	void Run() override;

	//functions for spoofing detection
	void addGpsSample(const sensor_gps_raw_s &gps_raw);
	void addImuSample(const vehicle_imu_s &imu);
	void detrendSample();
	bool windowReady();
	bool runSpoofingDetection();
	bool start();

	//buffer structs
	struct GpsSample {
		uint64_t timestamp_sample{};
		double carrier_phase[32]{};
		double detrended_carrier_phase[32]{};
		uint8_t flags[32]{};
		bool carrier_phase_valid[32]{};
		bool detrended_valid[32]{};
	};

	struct ImuSample {
		uint64_t timestamp_sample{};
		float delta_velocity[3]{};
		uint32_t delta_velocity_dt{};
	};

	//window buffers
	static constexpr int GPS_BUFFER_SIZE = 20;
	static constexpr int IMU_BUFFER_SIZE = 1000;
	GpsSample _gps_buffer[GPS_BUFFER_SIZE]{};
	ImuSample _imu_buffer[IMU_BUFFER_SIZE]{};

	int _gps_buffer_count{0};
	int _imu_buffer_count{0};

	bool _spoofing_detected_prev{false};

	//subscribe to sensor_gps_raw and vehicle_imu and use sensor_gps_raw a triger
	uORB::SubscriptionCallbackWorkItem _sensor_gps_raw_sub{this, ORB_ID(sensor_gps_raw)};
	uORB::Subscription _vehicle_imu_sub{ORB_ID(vehicle_imu)};
};
