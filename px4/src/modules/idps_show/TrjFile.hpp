/**
 * Reader for the IDPS per-drone show file (.trj), written by `idps-show export`
 * (format: idps/show/trj.py). Samples are read on demand through a small window, so a long
 * show needs no more RAM than a few kilobytes.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace idps
{

struct TrjHeader {
	char magic[8];
	uint16_t version;
	uint16_t header_size;
	uint16_t drone_id;
	uint16_t flags;
	float rate_hz;
	uint32_t count;
	float t0;
	float home[3];
	float yaw;
	uint32_t samples_crc;
	uint8_t batch;            // take-off batch (1..) from a charging pad; 0: the show starts on the ground
	uint8_t n_batches;
	uint16_t batch_gap_ms;
	uint16_t climb_mm_s;      // leg speeds (peak of a minimum-jerk move = 1.875 x mean)
	uint16_t transfer_mm_s;
	uint16_t hold_timeout_s;  // no start this long after the last batch took off: land
	uint16_t pad_facing_cdeg; // the pad's front, 0.01° clockwise from North (take-off steps that way)
	uint32_t header_crc;
};
static_assert(sizeof(TrjHeader) == 64, "TrjHeader must be 64 bytes");

struct TrjSample {
	float pos[3];
	float vel[3];
	float acc[3];
	uint8_t rgb[3];
	uint8_t pad;
};
static_assert(sizeof(TrjSample) == 40, "TrjSample must be 40 bytes");

class TrjFile
{
public:
	~TrjFile() { close(); }

	/** Open and verify the whole file (header CRC, sample CRC). Returns nullptr on success,
	 * else a short reason. file_crc gets the CRC of the whole file (as MAVLink FTP computes). */
	const char *open(const char *path, uint32_t &file_crc);
	void close();
	bool is_open() const { return _fd >= 0; }

	const TrjHeader &header() const { return _hdr; }

	/** Sample i (0..count-1), read through the window. False on a read error. */
	bool sample(uint32_t i, TrjSample &out);

private:
	static constexpr uint32_t WINDOW = 128;

	int _fd{-1};
	TrjHeader _hdr{};
	TrjSample _win[WINDOW] {};
	uint32_t _win_first{0};
	uint32_t _win_count{0};
};

} // namespace idps
