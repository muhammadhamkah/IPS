#include "IdpsShow.hpp"

#include <lib/mathlib/mathlib.h>
#include <math.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <string.h>

IdpsShow::IdpsShow() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	snprintf(_path, sizeof(_path), "%s/idps_show.trj", PX4_STORAGEDIR);
}

bool IdpsShow::init()
{
	load();                        // a file left from an earlier upload is ready straight away
	ScheduleOnInterval(20_ms);     // 50 Hz
	return true;
}

const char *IdpsShow::state_name() const
{
	switch (_state) {
	case State::NoFile: return "no show file";

	case State::Ready: return "ready";

	case State::Waiting: return "waiting for start";

	case State::Running: return "flying the show";

	case State::Done: return "show finished";

	case State::Aborted: return "aborted";
	}

	return "?";
}

const char *IdpsShow::load()
{
	const char *err = _file.open(_path, _file_crc);

	if (err) {
		_state = State::NoFile;
		return err;
	}

	_state = State::Ready;
	const idps::TrjHeader &h = _file.header();
	mavlink_log_info(&_mavlink_log_pub, "IDPS show: drone %u, %.1f s loaded, crc %08lx",
			 (unsigned)h.drone_id, (double)((h.count - 1) / h.rate_hz), (unsigned long)_file_crc);
	return nullptr;
}

void IdpsShow::ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s a{};
	a.command = cmd.command;
	a.result = result;
	a.result_param1 = (uint8_t)_state;
	a.result_param2 = (int32_t)_file_crc;
	a.target_system = cmd.source_system;
	a.target_component = cmd.source_component;
	a.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(a);
}

void IdpsShow::send_vehicle_command(uint32_t command, float param1, float param2)
{
	vehicle_command_s c{};
	c.command = command;
	c.param1 = param1;
	c.param2 = param2;
	c.param3 = NAN;
	c.param4 = NAN;
	c.param5 = (double)NAN;
	c.param6 = (double)NAN;
	c.param7 = NAN;
	c.target_system = _status.system_id;
	c.target_component = _status.component_id;
	c.source_system = _status.system_id;
	c.source_component = _status.component_id;
	c.from_external = false;
	c.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(c);
}

const char *IdpsShow::start(const vehicle_command_s &cmd, hrt_abstime now)
{
	int64_t t_pc = 0;

	if (!pc_time_us(cmd, t_pc)) {
		return "start time missing (send COMMAND_INT)";
	}

	if (_state == State::Waiting || _state == State::Running) {
		// a repeated start (the PC missed our ack) for the same instant is fine
		return (t_pc == _start_pc && cmd.source_system == _pc_system) ? nullptr : "already started";
	}

	if (!_file.is_open() && load() != nullptr) {
		return "no valid show file";
	}

	if (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED || !_landed) {
		return "must be landed and disarmed";
	}

	if (!sync_valid(now)) {
		return "no clock sync from the PC";
	}

	const int64_t ahead = t_pc - pc_now_us(now);

	if (ahead < MIN_NOTICE || ahead > MAX_NOTICE) {
		return "start time not 4..600 s ahead";
	}

	if (!_lpos.xy_valid || !_lpos.z_valid || now > _lpos.timestamp + 500_ms) {
		return "no position (IDPS positions not arriving)";
	}

	const idps::TrjHeader &h = _file.header();
	const float dn = _lpos.x - h.home[0];
	const float de = _lpos.y - h.home[1];

	if (sqrtf(dn * dn + de * de) > MAX_HOME_OFFSET_M) {
		mavlink_log_critical(&_mavlink_log_pub, "IDPS show: %.2f m from home spot", (double)sqrtf(dn * dn + de * de));
		return "not on its home spot";
	}

	_start_pc = t_pc;
	_pc_system = cmd.source_system;
	_mode_sent = false;
	_arm_sent = false;
	_state = State::Waiting;
	mavlink_log_info(&_mavlink_log_pub, "IDPS show: starting in %.1f s", (double)(ahead * 1e-6));
	return nullptr;
}

void IdpsShow::abort(const char *why)
{
	if (_state == State::Waiting || _state == State::Running) {
		if (_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
		}

		mavlink_log_critical(&_mavlink_log_pub, "IDPS show aborted: %s", why);
	}

	_state = State::Aborted;
}

void IdpsShow::finish()
{
	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
	_state = State::Done;
	mavlink_log_info(&_mavlink_log_pub, "IDPS show: finished, landing");
}

bool IdpsShow::pc_time_us(const vehicle_command_s &cmd, int64_t &t_us)
{
	if (!PX4_ISFINITE(cmd.param5) || !PX4_ISFINITE(cmd.param6)) {
		return false;
	}

	// COMMAND_INT x, y arrive as x / 1e7 and y / 1e7
	const int64_t sec = llround(cmd.param5 * 1e7);
	const int64_t usec = llround(cmd.param6 * 1e7);

	if (sec < 0 || usec < 0 || usec >= 1000000) {
		return false;
	}

	t_us = sec * 1000000LL + usec;
	return true;
}

void IdpsShow::sync_add(hrt_abstime local, int64_t pc_us)
{
	const int64_t d = (int64_t)local - pc_us;

	if (_sync_cur.d != INT64_MAX && local >= _sync_cur_start + SYNC_BUCKET) {
		// close the bucket: keep its fastest sample
		_sync[_sync_head] = _sync_cur;
		_sync_head = (_sync_head + 1) % SYNC_BUCKETS;
		_sync_count = _sync_count < SYNC_BUCKETS ? _sync_count + 1 : SYNC_BUCKETS;
		_sync_cur = SyncPoint{0, INT64_MAX};

		// least-squares line through the bucket minima: d(t) = a + b (t - tref)
		_sync_tref = local;
		double sx = 0, sy = 0, sxx = 0, sxy = 0;
		const int64_t d0 = _sync[(_sync_head + SYNC_BUCKETS - 1) % SYNC_BUCKETS].d;

		for (int i = 0; i < _sync_count; i++) {
			const double x = (double)((int64_t)_sync[i].t - (int64_t)_sync_tref);
			const double y = (double)(_sync[i].d - d0);
			sx += x; sy += y; sxx += x * x; sxy += x * y;
		}

		const double n = _sync_count;
		const double var = sxx - sx * sx / n;
		_sync_b = (_sync_count >= 3 && var > 1.0) ? (sxy - sx * sy / n) / var : 0.0;
		_sync_b = math::constrain(_sync_b, -0.05, 0.05);
		_sync_a = (double)d0 + (sy - _sync_b * sx) / n;
	}

	if (_sync_cur.d == INT64_MAX) {
		_sync_cur_start = local;
	}

	if (d < _sync_cur.d) {
		_sync_cur = SyncPoint{local, d};
	}

	if (_sync_count == 0) {
		// until the first bucket closes, use the fastest sample so far
		_sync_tref = local;
		_sync_a = (double)_sync_cur.d;
		_sync_b = 0.0;
	}

	_sync_last = local;
}

bool IdpsShow::sync_valid(hrt_abstime now) const
{
	return _sync_count >= 3 && now < _sync_last + SYNC_STALE;
}

int64_t IdpsShow::pc_now_us(hrt_abstime now) const
{
	const double offset = _sync_a + _sync_b * (double)((int64_t)now - (int64_t)_sync_tref);
	return (int64_t)now - (int64_t)llround(offset);
}

void IdpsShow::handle_command(const vehicle_command_s &cmd)
{
	if (cmd.command != vehicle_command_s::VEHICLE_CMD_CUSTOM_0) {
		return;
	}

	const int action = (int)lroundf(cmd.param1);
	const int drone_id = (int)lroundf(cmd.param2);

	if (action == SYNC) {
		int64_t t_pc = 0;
		const bool running = _state == State::Waiting || _state == State::Running;

		if (drone_id == _status.system_id && pc_time_us(cmd, t_pc) && (!running || cmd.source_system == _pc_system)) {
			sync_add(cmd.timestamp, t_pc);
		}

		return;                                        // not acknowledged: it comes 5 times a second
	}

	if (drone_id != _status.system_id) {
		mavlink_log_critical(&_mavlink_log_pub, "IDPS show: command for drone %d, I am %u", drone_id,
				     (unsigned)_status.system_id);
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const char *err = nullptr;

	switch (action) {
	case STATUS:
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		return;

	case ABORT:
		abort("abort command");
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		return;

	case LOAD:
		if (_state == State::Waiting || _state == State::Running) {
			err = "busy";

		} else {
			err = load();
		}

		break;

	case START:
		err = start(cmd, now);
		break;

	default:
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		return;
	}

	// The PC states which file it expects; a different file on the card is refused.
	if (!err) {
		const uint32_t want = ((uint32_t)lroundf(cmd.param3) << 16) | ((uint32_t)lroundf(cmd.param4) & 0xffffu);

		if (_file.header().drone_id != _status.system_id) {
			err = "show file is for another drone";

		} else if (want != _file_crc) {
			err = "show file differs from the PC's (re-upload)";
		}

		if (err && action == START) {
			_state = State::Ready;
		}
	}

	if (err) {
		mavlink_log_critical(&_mavlink_log_pub, "IDPS show: %s", err);
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);

	} else {
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
	}
}

void IdpsShow::publish_setpoint(hrt_abstime now, float show_t)
{
	const idps::TrjHeader &h = _file.header();
	const float h_s = 1.f / h.rate_hz;
	float fidx = (show_t - h.t0) * h.rate_hz;
	const float last = (float)(h.count - 1);
	const bool moving = fidx > 0.f && fidx < last;
	fidx = math::constrain(fidx, 0.f, last);
	uint32_t k = (uint32_t)fidx;

	if (k >= h.count - 1) {
		k = h.count - 2;
	}

	const float f = fidx - (float)k;
	idps::TrjSample a{}, b{};

	if (!_file.sample(k, a) || !_file.sample(k + 1, b)) {
		abort("show file read error");
		return;
	}

	// Cubic Hermite through the samples' positions and velocities; acceleration linear.
	const float f2 = f * f, f3 = f2 * f;
	const float h00 = 2 * f3 - 3 * f2 + 1, h10 = f3 - 2 * f2 + f, h01 = -2 * f3 + 3 * f2, h11 = f3 - f2;
	const float d00 = 6 * f2 - 6 * f, d10 = 3 * f2 - 4 * f + 1, d01 = -6 * f2 + 6 * f, d11 = 3 * f2 - 2 * f;

	trajectory_setpoint_s sp{};

	for (int i = 0; i < 3; i++) {
		sp.position[i] = h00 * a.pos[i] + h10 * h_s * a.vel[i] + h01 * b.pos[i] + h11 * h_s * b.vel[i];
		sp.velocity[i] = moving ? (d00 * a.pos[i] + d10 * h_s * a.vel[i] + d01 * b.pos[i] + d11 * h_s * b.vel[i]) / h_s : 0.f;
		sp.acceleration[i] = moving ? a.acc[i] + f * (b.acc[i] - a.acc[i]) : 0.f;
		sp.jerk[i] = NAN;
		_rgb[i] = (uint8_t)lroundf(a.rgb[i] + f * ((float)b.rgb[i] - (float)a.rgb[i]));
	}

	sp.yaw = h.yaw;
	sp.yawspeed = NAN;
	sp.timestamp = now;

	offboard_control_mode_s ocm{};
	ocm.position = true;
	ocm.velocity = true;
	ocm.acceleration = true;
	ocm.timestamp = now;
	_offboard_control_mode_pub.publish(ocm);
	_trajectory_setpoint_pub.publish(sp);
}

void IdpsShow::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	_vehicle_status_sub.update(&_status);
	_vehicle_local_position_sub.update(&_lpos);

	vehicle_land_detected_s land{};

	if (_vehicle_land_detected_sub.update(&land)) {
		_landed = land.landed;
	}

	vehicle_command_s cmd{};

	while (_vehicle_command_sub.update(&cmd)) {
		if (cmd.from_external) {
			handle_command(cmd);
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_state != State::Waiting && _state != State::Running) {
		return;
	}

	const bool armed = _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool offboard = _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

	const int64_t to_start = _start_pc - pc_now_us(now);      // µs until the show starts

	if (_state == State::Waiting) {
		// Stream the home setpoint until the start, so PX4 accepts OFFBOARD.
		publish_setpoint(now, _file.header().t0);

		if (!sync_valid(now)) {
			abort("lost clock sync with the PC");
			return;
		}

		if (!_mode_sent && to_start <= (int64_t)MODE_LEAD) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);   // custom, OFFBOARD
			_mode_sent = true;
		}

		if (!_arm_sent && to_start <= (int64_t)ARM_LEAD) {
			if (!offboard) {
				abort("OFFBOARD not accepted");
				return;
			}

			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
			_arm_sent = true;
		}

		if (to_start <= 0) {
			if (!armed || !offboard) {
				abort(!armed ? "did not arm" : "left OFFBOARD before the start");
				return;
			}

			_state = State::Running;
		}

		return;
	}

	// Running
	if (!offboard || !armed) {
		// A failsafe or the pilot took over: leave the drone to PX4.
		_state = State::Aborted;
		mavlink_log_critical(&_mavlink_log_pub, "IDPS show stopped: left OFFBOARD");
		return;
	}

	const idps::TrjHeader &h = _file.header();
	const float show_t = h.t0 + (float)((double)(-to_start) * 1e-6);
	publish_setpoint(now, show_t);

	if (show_t > h.t0 + (float)(h.count - 1) / h.rate_hz + 0.5f) {
		finish();
	}
}

int IdpsShow::print_status()
{
	PX4_INFO("state: %s, file %s, crc %08lx", state_name(), _path, (unsigned long)_file_crc);

	if (_file.is_open()) {
		const idps::TrjHeader &h = _file.header();
		PX4_INFO("drone %u, %lu samples at %.0f Hz (%.1f s), home N %.2f E %.2f D %.2f", (unsigned)h.drone_id,
			 (unsigned long)h.count, (double)h.rate_hz, (double)((h.count - 1) / h.rate_hz),
			 (double)h.home[0], (double)h.home[1], (double)h.home[2]);
	}

	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("clock sync: %s, %d s of history, offset %.3f ms, drift %.1f ppm, last %.1f s ago",
		 sync_valid(now) ? "ok" : "not ready", _sync_count,
		 (double)(_sync_a + _sync_b * (double)((int64_t)now - (int64_t)_sync_tref)) * 1e-3, _sync_b * 1e6,
		 _sync_last ? (double)((now - _sync_last) * 1e-6) : -1.0);

	if (_state == State::Waiting) {
		PX4_INFO("start in %.4f s", (double)((_start_pc - pc_now_us(now)) * 1e-6));
	}

	PX4_INFO("light %u %u %u", _rgb[0], _rgb[1], _rgb[2]);
	return 0;
}

int IdpsShow::task_spawn(int argc, char *argv[])
{
	IdpsShow *instance = new IdpsShow();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int IdpsShow::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IdpsShow::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Plays this drone's part of an IDPS show from the SD card (idps_show.trj), started at a set
time by the IDPS PC with a MAVLink command. See the IDPS repository, docs/SHOW.md.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("idps_show", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int idps_show_main(int argc, char *argv[])
{
	return IdpsShow::main(argc, argv);
}
