#include "IdpsShow.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <math.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <string.h>

IdpsShow::IdpsShow() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)   // with the position controller
{
	snprintf(_path, sizeof(_path), "%s/idps_show.trj", PX4_STORAGEDIR);
}

bool IdpsShow::init()
{
	load();                        // a file left from an earlier upload is ready straight away
	ScheduleOnInterval(SETPOINT_PERIOD);
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

	case State::Launching: return "taking off to the grid";

	case State::Holding: return "holding in the grid";

	case State::Landing: return "returning to its pad";
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

bool IdpsShow::active() const
{
	return _state == State::Waiting || _state == State::Running || _state == State::Launching
	       || _state == State::Holding || _state == State::Landing;
}

const char *IdpsShow::start(const vehicle_command_s &cmd, hrt_abstime now)
{
	int64_t t_pc = 0;

	if (!pc_time_us(cmd, t_pc)) {
		return "start time missing (send COMMAND_INT)";
	}

	if (_file.is_open() && _file.header().batch > 0) {
		return start_in_air(t_pc, cmd, now);
	}

	if (_state == State::Waiting || _state == State::Running) {
		// a repeated start (the PC missed our ack) for the same instant is fine
		return (t_pc == _start_pc && cmd.source_system == _pc_system) ? nullptr : "already started";
	}

	if (!_file.is_open() && load() != nullptr) {
		return "no valid show file";
	}

	if (_file.header().batch > 0) {
		return "this show starts in the air: launch first";
	}

	const char *err = ready_on_pad(now, t_pc);

	if (err) {
		return err;
	}

	const int64_t ahead = t_pc - pc_now_us(now);
	_start_pc = t_pc;
	_pc_system = cmd.source_system;
	_mode_sent = false;
	_arm_sent = false;
	_state = State::Waiting;
	mavlink_log_info(&_mavlink_log_pub, "IDPS show: starting in %.1f s", (double)(ahead * 1e-6));
	return nullptr;
}

const char *IdpsShow::ready_on_pad(hrt_abstime now, int64_t t_pc)
{
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

	// tracked: the cameras see it now (the estimate alone stays valid for seconds without them)
	if (_vision_at == 0 || now > _vision_at + MAX_VISION_AGE) {
		return "not tracked (no IDPS position from the cameras)";
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

	return nullptr;
}

float IdpsShow::leg_time(float dist, float speed)
{
	return fmaxf(1.5f, 1.875f * fabsf(dist) / fmaxf(speed, 0.1f));
}

bool IdpsShow::show_point(float show_t, float p[3])
{
	trajectory_setpoint_s sp{};

	if (!evaluate(show_t, sp)) {
		return false;
	}

	for (int i = 0; i < 3; i++) {
		p[i] = sp.position[i];
	}

	return true;
}

const char *IdpsShow::launch(const vehicle_command_s &cmd, hrt_abstime now)
{
	int64_t t_pc = 0;

	if (!pc_time_us(cmd, t_pc)) {
		return "launch time missing (send COMMAND_INT)";
	}

	if (_state == State::Launching || _state == State::Holding) {
		return (t_pc == _launch_pc && cmd.source_system == _pc_system) ? nullptr : "already launched";
	}

	if (!_file.is_open() && load() != nullptr) {
		return "no valid show file";
	}

	const idps::TrjHeader &h = _file.header();

	if (h.batch == 0) {
		return "this show starts on the ground: start, not launch";
	}

	const char *err = ready_on_pad(now, t_pc);

	if (err) {
		return err;
	}

	float grid[3];

	if (!show_point(h.t0, grid)) {
		return "show file read error";
	}

	// take-off: straight up over the pad slot to the grid's height, CLEAR_M forward over the pad's
	// empty slots (a long leg starts slowly), then level to the grid place
	_legs.t0 = t_pc + (int64_t)h.batch_gap_ms * 1000 * (h.batch - 1);
	float fwd[3];
	forward(fwd);

	for (int i = 0; i < 3; i++) {
		_legs.p[0][i] = h.home[i];
		_legs.p[1][i] = i < 2 ? h.home[i] : grid[2];
		_legs.p[2][i] = _legs.p[1][i] + fwd[i] * CLEAR_M;
		_legs.p[3][i] = grid[i];
	}

	const float dn = grid[0] - _legs.p[2][0], de = grid[1] - _legs.p[2][1];
	_legs.T[0] = leg_time(grid[2] - h.home[2], h.climb_mm_s * 1e-3f);
	_legs.T[1] = leg_time(CLEAR_M, h.transfer_mm_s * 1e-3f);
	_legs.T[2] = leg_time(sqrtf(dn * dn + de * de), h.transfer_mm_s * 1e-3f);

	_launch_pc = t_pc;
	_go_pc = _legs.t0;
	_start_pc = 0;
	_pc_system = cmd.source_system;
	_mode_sent = false;
	_arm_sent = false;
	_airborne = false;
	_land_sent = false;
	_state = State::Launching;
	mavlink_log_info(&_mavlink_log_pub, "IDPS show: batch %u of %u, take-off in %.1f s", (unsigned)h.batch,
			 (unsigned)h.n_batches, (double)((_go_pc - pc_now_us(now)) * 1e-6));
	return nullptr;
}

const char *IdpsShow::start_in_air(int64_t t_pc, const vehicle_command_s &cmd, hrt_abstime now)
{
	if (_state == State::Running) {
		return (t_pc == _start_pc && cmd.source_system == _pc_system) ? nullptr : "already started";
	}

	if (_state != State::Launching && _state != State::Holding) {
		return "not in the air grid (launch first)";
	}

	if (cmd.source_system != _pc_system) {
		return "launched by another PC";
	}

	if (_start_pc != 0) {
		return t_pc == _start_pc ? nullptr : "already started";
	}

	if (!sync_valid(now)) {
		return "no clock sync from the PC";
	}

	const idps::TrjHeader &h = _file.header();
	const int64_t gap = (int64_t)h.batch_gap_ms * 1000;
	const int64_t leave = _launch_pc + gap * (h.n_batches - 1) + (int64_t)h.hold_timeout_s * 1000000
			      + gap * (h.n_batches - h.batch);

	if (t_pc < _legs.end() + 1000000) {
		return "start before this drone reaches the grid";
	}

	if (t_pc >= leave) {
		return "start after the hold timeout";
	}

	_start_pc = t_pc;
	mavlink_log_info(&_mavlink_log_pub, "IDPS show: show starts in %.1f s", (double)((t_pc - pc_now_us(now)) * 1e-6));
	return nullptr;
}

void IdpsShow::abort(const char *why)
{
	if (active()) {
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

int64_t IdpsShow::show_clock_us(hrt_abstime now)
{
	// A new clock sync point can move the PC's time by a few ms; stepping the show by that would
	// jump the setpoint. Run with the local clock (at the synced rate) and slew towards the sync.
	const int64_t target = pc_now_us(now);

	if (_show_pc_at == 0) {
		_show_pc = target;

	} else if (now <= _show_pc_at) {
		return _show_pc;                // no time has passed (the simulator's clock may stand still)

	} else {
		const double dt = (double)(now - _show_pc_at);
		const int64_t predicted = _show_pc + (int64_t)llround(dt * (1.0 - _sync_b));   // pc = local - offset
		const int64_t step = (int64_t)llround(dt * CLOCK_SLEW);
		_show_pc = predicted + math::constrain(target - predicted, -step, step);
	}

	_show_pc_at = now;
	return _show_pc;
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
		if (drone_id == _status.system_id && pc_time_us(cmd, t_pc) && (!active() || cmd.source_system == _pc_system)) {
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
		if (active()) {
			err = "busy";

		} else {
			err = load();
		}

		break;

	case START:
		err = start(cmd, now);
		break;

	case LAUNCH:
		err = launch(cmd, now);
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

		if (err && (action == START || action == LAUNCH) && !_airborne && _state != State::Running) {
			_state = State::Ready;
			_start_pc = 0;
		}
	}

	if (err) {
		mavlink_log_critical(&_mavlink_log_pub, "IDPS show: %s", err);
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);

	} else {
		ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
	}
}

void IdpsShow::read_params()
{
	const char *names[3] = {"IDPS_ACC_LEAD", "MPC_XY_VEL_D_ACC", "MPC_Z_VEL_D_ACC"};
	float *dest[3] = {&_acc_lead, &_vel_d_xy, &_vel_d_z};

	for (int i = 0; i < 3; i++) {
		param_t p = param_find(names[i]);
		*dest[i] = 0.f;

		if (p != PARAM_INVALID) {
			param_get(p, dest[i]);
		}
	}
}

void IdpsShow::publish_setpoint(hrt_abstime now, float show_t, bool running)
{
	trajectory_setpoint_s sp{};

	if (!evaluate(show_t, sp)) {
		abort("show file read error");
		return;
	}

	if (running) {
		// The show runs on the PC's clock: ds/dt = r show seconds per local second (1 on real
		// hardware to a few ppm; a few % in the simulator). Feed-forwards are per local second.
		const float r = (float)math::constrain(1.0 - _sync_b, 0.9, 1.1);
		trajectory_setpoint_s ahead{};
		const bool lead = _acc_lead > 0.f && evaluate(show_t + _acc_lead * r, ahead);

		for (int i = 0; i < 3; i++) {
			sp.velocity[i] *= r;
			// taken ahead by the airframe's lag; PX4's velocity D term acts on the measured
			// acceleration and so takes back D x the acceleration it was asked for
			const float acc = lead ? ahead.acceleration[i] : sp.acceleration[i];
			sp.acceleration[i] = acc * r * r * (1.f + (i < 2 ? _vel_d_xy : _vel_d_z));
		}
	}

	publish(sp, now);
}

void IdpsShow::publish(const trajectory_setpoint_s &sp_in, hrt_abstime now)
{
	trajectory_setpoint_s sp = sp_in;
	sp.timestamp = now;

	offboard_control_mode_s ocm{};
	ocm.position = true;
	ocm.velocity = true;
	ocm.acceleration = true;
	ocm.timestamp = now;
	_offboard_control_mode_pub.publish(ocm);
	_trajectory_setpoint_pub.publish(sp);
}

void IdpsShow::hold_setpoint(const float p[3], trajectory_setpoint_s &sp) const
{
	for (int i = 0; i < 3; i++) {
		sp.position[i] = p[i];
		sp.velocity[i] = 0.f;
		sp.acceleration[i] = 0.f;
		sp.jerk[i] = NAN;
	}

	sp.yaw = _file.header().yaw;
	sp.yawspeed = NAN;
}

void IdpsShow::path_setpoint(const Path &path, int64_t t_pc, trajectory_setpoint_s &sp) const
{
	float t = (float)((double)(t_pc - path.t0) * 1e-6);
	int leg = 0;

	while (leg < LEGS - 1 && t > path.T[leg]) {
		t -= path.T[leg];
		leg++;
	}

	const float T = path.T[leg];
	const float u = math::constrain(t / T, 0.f, 1.f);
	// minimum jerk: s = 10u^3 - 15u^4 + 6u^5
	const float s = u * u * u * (10.f - 15.f * u + 6.f * u * u);
	const float ds = 30.f * u * u * (1.f - u) * (1.f - u) / T;
	const float dds = (60.f * u - 180.f * u * u + 120.f * u * u * u) / (T * T);

	for (int i = 0; i < 3; i++) {
		const float d = path.p[leg + 1][i] - path.p[leg][i];
		sp.position[i] = path.p[leg][i] + d * s;
		sp.velocity[i] = d * ds;
		sp.acceleration[i] = d * dds;
		sp.jerk[i] = NAN;
	}

	sp.yaw = _file.header().yaw;
	sp.yawspeed = NAN;
}

void IdpsShow::forward(float f[3]) const
{
	const float a = math::radians((float)_file.header().pad_facing_cdeg * 0.01f);
	f[0] = cosf(a);
	f[1] = sinf(a);
	f[2] = 0.f;
}

void IdpsShow::begin_landing(const float from[3], int64_t t_leave)
{
	// level to CLEAR_M in front of the pad slot, back over it, then down onto it (at zero speed);
	// PX4 then lands and disarms
	const idps::TrjHeader &h = _file.header();
	_legs.t0 = t_leave;
	float fwd[3];
	forward(fwd);

	for (int i = 0; i < 3; i++) {
		_legs.p[0][i] = from[i];
		_legs.p[2][i] = i < 2 ? h.home[i] : from[2];
		_legs.p[1][i] = _legs.p[2][i] + fwd[i] * CLEAR_M;
		_legs.p[3][i] = h.home[i];
	}

	const float dn = _legs.p[1][0] - from[0], de = _legs.p[1][1] - from[1];
	_legs.T[0] = leg_time(sqrtf(dn * dn + de * de), h.transfer_mm_s * 1e-3f);
	_legs.T[1] = leg_time(CLEAR_M, h.transfer_mm_s * 1e-3f);
	_legs.T[2] = leg_time(_legs.p[3][2] - from[2], h.climb_mm_s * 1e-3f);
	_land_sent = false;
	_state = State::Landing;
}

void IdpsShow::run_pads(hrt_abstime now)
{
	const idps::TrjHeader &h = _file.header();
	const bool armed = _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool offboard = _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	trajectory_setpoint_s sp{};

	if (_state == State::Launching && !_airborne) {
		// on the pad until this batch's turn: stream the pad setpoint so PX4 accepts OFFBOARD
		_show_pc_at = 0;
		const int64_t to_go = _go_pc - pc_now_us(now);
		hold_setpoint(_legs.p[0], sp);
		publish(sp, now);

		if (!sync_valid(now)) {
			abort("lost clock sync with the PC");
			return;
		}

		if (!_mode_sent && to_go <= (int64_t)MODE_LEAD) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);   // custom, OFFBOARD
			_mode_sent = true;
		}

		if (!_arm_sent && to_go <= (int64_t)ARM_LEAD) {
			if (!offboard) {
				abort("OFFBOARD not accepted");
				return;
			}

			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
			_arm_sent = true;
		}

		if (to_go <= 0) {
			if (!armed || !offboard) {
				abort(!armed ? "did not arm" : "left OFFBOARD before take-off");
				return;
			}

			read_params();
			_airborne = true;
		}

		return;
	}

	if (!offboard || !armed) {
		_state = State::Aborted;
		mavlink_log_critical(&_mavlink_log_pub, "IDPS show stopped: left OFFBOARD");
		return;
	}

	const int64_t t = show_clock_us(now);

	if (_state == State::Launching) {
		path_setpoint(_legs, t, sp);

		if (t >= _legs.end()) {
			_state = State::Holding;
			mavlink_log_info(&_mavlink_log_pub, "IDPS show: in the grid, holding");
		}

	} else if (_state == State::Holding) {
		hold_setpoint(_legs.p[LEGS], sp);

		if (_start_pc != 0 && t >= _start_pc) {
			read_params();
			_state = State::Running;          // the show plays from the next cycle

		} else if (_start_pc == 0) {
			const int64_t gap = (int64_t)h.batch_gap_ms * 1000;
			const int64_t leave = _launch_pc + gap * (h.n_batches - 1) + (int64_t)h.hold_timeout_s * 1000000
					      + gap * (h.n_batches - h.batch);

			if (t >= leave) {
				mavlink_log_info(&_mavlink_log_pub, "IDPS show: no start, back to the pad");
				const float grid[3] {_legs.p[LEGS][0], _legs.p[LEGS][1], _legs.p[LEGS][2]};
				begin_landing(grid, leave);
			}
		}

	} else if (_state == State::Landing) {
		if (t < _legs.t0) {
			hold_setpoint(_legs.p[0], sp);

		} else {
			path_setpoint(_legs, t, sp);
		}

		if (t >= _legs.end() && !_land_sent) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
			_land_sent = true;
			_state = State::Done;
			mavlink_log_info(&_mavlink_log_pub, "IDPS show: over its pad, landing");
			return;
		}
	}

	const TrackPhase phase = _state == State::Launching ? TRACK_TAKEOFF : _state == State::Landing ? TRACK_LANDING : TRACK_HOLD;
	track(now, t, phase, sp.position, _start_pc != 0 ? h.t0 + (float)((double)(t - _start_pc) * 1e-6) : NAN);
	publish(sp, now);
}

void IdpsShow::track(hrt_abstime now, int64_t t_pc, TrackPhase phase, const float plan[3], float show_t)
{
	// each simulated drone's local frame starts at its home spot (h.home in the venue frame)
	if (!_have_gt) {
		return;
	}

	const idps::TrjHeader &h = _file.header();
	const float gt_pos[3] {_gt.x, _gt.y, _gt.z};
	debug_array_s v{};
	strncpy(v.name, "track", sizeof(v.name));

	for (int i = 0; i < 3; i++) {
		v.data[i] = gt_pos[i] + h.home[i] - plan[i];     // from where it should be, NED m
		v.data[7 + i] = plan[i];                           // where it should be, venue NED m
	}

	v.data[3] = show_t;                                    // NaN before the start is known
	v.data[4] = (float)phase;
	const int64_t sec = t_pc / 1000000;                    // the PC's clock, in two parts (float32)
	v.data[5] = (float)sec;
	v.data[6] = (float)((double)(t_pc - sec * 1000000) * 1e-6);
	v.timestamp = now;
	_track_pub.publish(v);
}

bool IdpsShow::evaluate(float show_t, trajectory_setpoint_s &sp)
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
		return false;
	}

	// Cubic Hermite through the samples' positions and velocities; acceleration linear.
	const float f2 = f * f, f3 = f2 * f;
	const float h00 = 2 * f3 - 3 * f2 + 1, h10 = f3 - 2 * f2 + f, h01 = -2 * f3 + 3 * f2, h11 = f3 - f2;
	const float d00 = 6 * f2 - 6 * f, d10 = 3 * f2 - 4 * f + 1, d01 = -6 * f2 + 6 * f, d11 = 3 * f2 - 2 * f;

	for (int i = 0; i < 3; i++) {
		sp.position[i] = h00 * a.pos[i] + h10 * h_s * a.vel[i] + h01 * b.pos[i] + h11 * h_s * b.vel[i];
		sp.velocity[i] = moving ? (d00 * a.pos[i] + d10 * h_s * a.vel[i] + d01 * b.pos[i] + d11 * h_s * b.vel[i]) / h_s : 0.f;
		sp.acceleration[i] = moving ? a.acc[i] + f * (b.acc[i] - a.acc[i]) : 0.f;
		sp.jerk[i] = NAN;
		_rgb[i] = (uint8_t)lroundf(a.rgb[i] + f * ((float)b.rgb[i] - (float)a.rgb[i]));
	}

	sp.yaw = h.yaw;
	sp.yawspeed = NAN;
	return true;
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
	vehicle_odometry_s vo;

	if (_visual_odometry_sub.update(&vo)) {
		_vision_at = hrt_absolute_time();
	}

	vehicle_land_detected_s land{};

	if (_vehicle_land_detected_sub.update(&land)) {
		_landed = land.landed;
	}

	vehicle_local_position_s gt;

	if (_groundtruth_sub.update(&gt)) {
		debug_vect_s v{};
		strncpy(v.name, "truth", sizeof(v.name));
		v.x = gt.x;
		v.y = gt.y;
		v.z = gt.z;
		v.timestamp = gt.timestamp;
		_truth_pub.publish(v);

		if (_sitl_vision) {
			sitl_vision(gt);
		}

		_gt = gt;
		_have_gt = true;
	}

	vehicle_command_s cmd{};

	while (_vehicle_command_sub.update(&cmd)) {
		if (cmd.from_external) {
			handle_command(cmd);
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!active()) {
		return;
	}

	if (_state == State::Launching || _state == State::Holding || _state == State::Landing) {
		run_pads(now);
		return;
	}

	const bool armed = _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool offboard = _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

	// µs until the show starts; while waiting the show clock follows the sync exactly
	const int64_t to_start = _start_pc - (_state == State::Running ? show_clock_us(now) : pc_now_us(now));

	if (_state == State::Waiting) {
		_show_pc_at = 0;
	}

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

			read_params();
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
	// evaluated half a period ahead: the controller uses each setpoint until the next one
	const float show_t = h.t0 + (float)((double)(-to_start + (int64_t)SETPOINT_PERIOD / 2) * 1e-6);
	publish_setpoint(now, show_t, true);

	if (_have_gt) {
		// simulator only: how far the drone is from the file at its own show time, unaffected by
		// how the PC and the simulator's clocks run
		trajectory_setpoint_s plan{};
		const float at = show_t - (float)SETPOINT_PERIOD * 0.5e-6f;

		if (evaluate(at, plan)) {
			track(now, _start_pc - to_start, TRACK_SHOW, plan.position, at);
		}
	}

	const float t_end = h.t0 + (float)(h.count - 1) / h.rate_hz;

	// From pads, landing takes over at the end (it holds the final grid until this batch leaves,
	// the first batch down right away); waiting longer put that batch's path partly behind it.
	if (show_t > t_end + (h.batch > 0 ? 0.f : 0.5f)) {
		if (h.batch > 0) {
			// back to the pad from the final grid, last up first down
			float last[3];

			if (!show_point(t_end, last)) {
				abort("show file read error");
				return;
			}

			const int64_t gap = (int64_t)h.batch_gap_ms * 1000;
			const int64_t end_pc = _start_pc + (int64_t)((t_end - h.t0) * 1e6f);
			begin_landing(last, end_pc + gap * (h.n_batches - h.batch));
			mavlink_log_info(&_mavlink_log_pub, "IDPS show: finished, back to the pad");

		} else {
			finish();
		}
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

	if (_file.is_open() && _file.header().batch > 0) {
		const idps::TrjHeader &h = _file.header();
		PX4_INFO("charging pad: batch %u of %u, gap %.1f s, climb %.2f m/s, transfer %.2f m/s, hold %u s",
			 (unsigned)h.batch, (unsigned)h.n_batches, (double)(h.batch_gap_ms * 1e-3f), (double)(h.climb_mm_s * 1e-3f),
			 (double)(h.transfer_mm_s * 1e-3f), (unsigned)h.hold_timeout_s);
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

void IdpsShow::set_sitl_vision(bool on, float delay_ms, float noise_mm, float rate_hz)
{
	const char *names[3] = {"EKF2_EV_POS_X", "EKF2_EV_POS_Y", "EKF2_EV_POS_Z"};

	for (int i = 0; i < 3; i++) {
		param_t p = param_find(names[i]);
		_ev_pos[i] = 0.f;

		if (p != PARAM_INVALID) {
			param_get(p, &_ev_pos[i]);
		}
	}

	_sitl_delay = (hrt_abstime)(fmaxf(delay_ms, 0.f) * 1000.f);
	_sitl_noise = fmaxf(noise_mm, 0.f) * 1e-3f;
	_sitl_period = rate_hz > 0.f ? (hrt_abstime)(1e6f / rate_hz) : 0;
	_vcount = 0;
	_sitl_vision = on;
	PX4_INFO("SITL vision %s, delay %.1f ms, noise %.1f mm, %.0f Hz", on ? "on" : "off", (double)delay_ms,
		 (double)noise_mm, (double)rate_hz);
}

void IdpsShow::sitl_vision(const vehicle_local_position_s &gt)
{
	_attitude_gt_sub.update(&_att_gt);
	const matrix::Dcmf R(matrix::Quatf(_att_gt.q));
	// each simulated drone starts at its own origin, which is its home spot in the venue
	matrix::Vector3f home{};

	if (_state != State::NoFile) {
		home = matrix::Vector3f(_file.header().home);
	}

	const matrix::Vector3f marker = matrix::Vector3f(gt.x, gt.y, gt.z) + home + R * matrix::Vector3f(_ev_pos);
	const hrt_abstime now = hrt_absolute_time();
	_vbuf[_vhead] = VisionSample{now, {marker(0), marker(1), marker(2)}};
	_vhead = (_vhead + 1) % VISION_BUF;
	_vcount = math::min(_vcount + 1, VISION_BUF);

	// the newest sample at least _sitl_delay old
	for (int k = 0; k < _vcount; k++) {
		const VisionSample &s = _vbuf[(_vhead - 1 - k + VISION_BUF) % VISION_BUF];

		if (now - s.t >= _sitl_delay) {
			if (now < _sitl_last + _sitl_period) {
				return;
			}

			_sitl_last = now;
			vehicle_odometry_s o{};
			o.timestamp = now;
			o.timestamp_sample = now;            // its arrival time; EKF2_EV_DELAY says how old it is
			o.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			for (int i = 0; i < 3; i++) {             // Gaussian noise (Box-Muller)
				const float u1 = ((float)rand() + 1.f) / ((float)RAND_MAX + 2.f);
				const float u2 = (float)rand() / (float)RAND_MAX;
				o.position[i] = s.p[i] + _sitl_noise * sqrtf(-2.f * logf(u1)) * cosf(2.f * M_PI_F * u2)
						+ (now < _kick_until ? _kick[i] : 0.f);
			}

			o.q[0] = 1.f;                        // heading 0 = North (D20)
			o.q[1] = o.q[2] = o.q[3] = 0.f;
			o.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN;

			for (int i = 0; i < 3; i++) {
				o.velocity[i] = NAN;
				o.angular_velocity[i] = NAN;
				o.position_variance[i] = 1e-6f;   // 1 mm
				o.orientation_variance[i] = 1e-3f;
				o.velocity_variance[i] = NAN;
			}

			o.quality = 100;
			_visual_odometry_pub.publish(o);
			return;
		}
	}
}

int IdpsShow::custom_command(int argc, char *argv[])
{
	if (argc >= 5 && strcmp(argv[0], "sitl_vision") == 0 && strcmp(argv[1], "kick") == 0 && is_running()) {
		// a step in the positions for a while (default 1 s): how the drone settles after a jolt
		IdpsShow *s = get_instance();

		for (int i = 0; i < 3; i++) {
			s->_kick[i] = (float)atof(argv[2 + i]) * 1e-3f;
		}

		s->_kick_until = hrt_absolute_time() + (hrt_abstime)((argc >= 6 ? atof(argv[5]) : 1000.0) * 1000.0);
		return 0;
	}

	if (argc >= 2 && strcmp(argv[0], "sitl_vision") == 0 && is_running()) {
		get_instance()->set_sitl_vision(strcmp(argv[1], "on") == 0, argc >= 3 ? (float)atof(argv[2]) : 0.f,
						argc >= 4 ? (float)atof(argv[3]) : 0.f, argc >= 5 ? (float)atof(argv[4]) : 0.f);
		return 0;
	}

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("sitl_vision", "simulator only: on [delay_ms [noise_mm [rate_hz]]] | off: true positions to EKF2; "
					 "kick N E D (mm) [ms]: offset them for a while");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int idps_show_main(int argc, char *argv[])
{
	return IdpsShow::main(argc, argv);
}
