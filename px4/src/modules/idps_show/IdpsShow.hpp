/**
 * idps_show: plays this drone's part of an IDPS show from its SD card (decision D22).
 *
 * The PC uploads the drone's .trj file (MAVLink FTP) and then commands it with MAVLink
 * COMMAND_INT, command 0 (PX4's VEHICLE_CMD_CUSTOM_0, which commander leaves to modules),
 * addressed to this drone's MAV_SYS_ID:
 *
 *   param1  action: 1 = load (re-read and verify the file), 2 = start, 3 = abort, 4 = status,
 *           5 = clock sync (sent by the PC at about 5 Hz the whole time; not acknowledged)
 *   param2  drone ID: must equal MAV_SYS_ID and the file's drone ID
 *   param3  file CRC-32, high 16 bits   (load, start: must match the file on the SD card)
 *   param4  file CRC-32, low 16 bits
 *   x, y    a time in the PC's clock, x = whole seconds, y = microseconds:
 *           start: the show start; sync: the PC's time when it sent the message
 *
 * The COMMAND_ACK carries the state in result_param1 (MAVLink "progress") and the file CRC
 * in result_param2.
 *
 * Clock: every drone runs the show on the PC's clock, so all drones start and stay together
 * whatever their own crystals do. From the sync messages it keeps, per second, the one that
 * arrived fastest (receive time minus PC send time is smallest), and fits a line through the
 * last 16 of those minima: offset and drift. Wi-Fi delay only ever adds, so the minima sit
 * about 1 ms above the true offset, the same for every drone.
 *
 * Start sequence: from the start command it streams the home setpoint (so PX4 accepts
 * OFFBOARD), switches to OFFBOARD 3 s before the start, arms 2 s before, and at the start
 * plays the file at 50 Hz (position, velocity and acceleration, yaw fixed). At the end, or on
 * abort, it commands LAND. If anything else takes the drone out of OFFBOARD (a failsafe, the
 * pilot), it stops and leaves the drone to PX4.
 */
#pragma once

#include "TrjFile.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class IdpsShow : public ModuleBase<IdpsShow>, public px4::ScheduledWorkItem
{
public:
	enum class State : uint8_t { NoFile = 0, Ready = 1, Waiting = 2, Running = 3, Done = 4, Aborted = 5 };

	enum Action { LOAD = 1, START = 2, ABORT = 3, STATUS = 4, SYNC = 5 };

	IdpsShow();
	~IdpsShow() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void handle_command(const vehicle_command_s &cmd);
	const char *load();
	const char *start(const vehicle_command_s &cmd, hrt_abstime now);
	void abort(const char *why);
	void finish();
	void ack(const vehicle_command_s &cmd, uint8_t result);
	void send_vehicle_command(uint32_t command, float param1 = NAN, float param2 = NAN);
	void publish_setpoint(hrt_abstime now, float show_t);
	static bool pc_time_us(const vehicle_command_s &cmd, int64_t &t_us);
	void sync_add(hrt_abstime local, int64_t pc_us);
	bool sync_valid(hrt_abstime now) const;
	int64_t pc_now_us(hrt_abstime now) const;
	const char *state_name() const;

	static constexpr hrt_abstime MODE_LEAD = 3_s;          // switch to OFFBOARD this long before the start
	static constexpr hrt_abstime ARM_LEAD = 2_s;           // arm this long before the start
	static constexpr int64_t MIN_NOTICE = 4000000;         // start must be at least 4 s ahead
	static constexpr int64_t MAX_NOTICE = 600000000;       // and at most 600 s
	static constexpr hrt_abstime SYNC_STALE = 5_s;         // no sync for this long: clock not trusted to start
	static constexpr float MAX_HOME_OFFSET_M = 0.5f;       // drone must stand on its home spot

	// Clock sync with the PC: per-second minima of (local receive time - PC send time).
	static constexpr int SYNC_BUCKETS = 16;
	static constexpr hrt_abstime SYNC_BUCKET = 1_s;
	struct SyncPoint { hrt_abstime t; int64_t d; };
	SyncPoint _sync[SYNC_BUCKETS] {};
	int _sync_count{0};
	int _sync_head{0};
	SyncPoint _sync_cur{0, INT64_MAX};   // fastest sample of the current second, and its time
	hrt_abstime _sync_cur_start{0};
	hrt_abstime _sync_last{0};
	double _sync_a{0.0};               // offset at _sync_tref (µs)
	double _sync_b{0.0};               // drift (µs per µs)
	hrt_abstime _sync_tref{0};
	uint8_t _pc_system{0};             // the PC that started the show: only it may sync/abort it

	idps::TrjFile _file;
	uint32_t _file_crc{0};
	State _state{State::NoFile};
	int64_t _start_pc{0};             // show start in the PC's clock (µs)
	bool _mode_sent{false};
	bool _arm_sent{false};
	uint8_t _rgb[3] {};
	char _path[64] {};
	orb_advert_t _mavlink_log_pub{nullptr};

	vehicle_status_s _status{};
	vehicle_local_position_s _lpos{};
	bool _landed{true};

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
};
