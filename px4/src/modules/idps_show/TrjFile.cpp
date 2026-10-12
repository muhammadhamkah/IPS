#include "TrjFile.hpp"

#include <crc32.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

namespace idps
{

const char *TrjFile::open(const char *path, uint32_t &file_crc)
{
	close();
	_fd = ::open(path, O_RDONLY);

	if (_fd < 0) {
		return "no show file";
	}

	if (::read(_fd, &_hdr, sizeof(_hdr)) != (ssize_t)sizeof(_hdr) || memcmp(_hdr.magic, "IDPSTRJ1", 8) != 0) {
		close();
		return "not an IDPS show file";
	}

	if (crc32part((const uint8_t *)&_hdr, 60, 0) != _hdr.header_crc) {
		close();
		return "header corrupt";
	}

	if (_hdr.version != 1 || _hdr.header_size != sizeof(TrjHeader) || _hdr.count < 2 || !(_hdr.rate_hz > 0.f)) {
		close();
		return "unsupported show file";
	}

	// Whole-file CRC (what the PC compares after the upload) and the sample-block CRC.
	file_crc = crc32part((const uint8_t *)&_hdr, sizeof(_hdr), 0);
	uint32_t samples_crc = 0;
	uint8_t buf[512];
	size_t total = 0;
	const size_t expected = (size_t)_hdr.count * sizeof(TrjSample);
	ssize_t n;

	while ((n = ::read(_fd, buf, sizeof(buf))) > 0) {
		file_crc = crc32part(buf, n, file_crc);
		size_t use = (total + n <= expected) ? (size_t)n : (total < expected ? expected - total : 0);
		samples_crc = crc32part(buf, use, samples_crc);
		total += n;
	}

	if (total != expected || samples_crc != _hdr.samples_crc) {
		close();
		return "samples truncated or corrupt";
	}

	_win_first = 0;
	_win_count = 0;
	return nullptr;
}

void TrjFile::close()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	_win_count = 0;
}

bool TrjFile::sample(uint32_t i, TrjSample &out)
{
	if (_fd < 0 || i >= _hdr.count) {
		return false;
	}

	if (i < _win_first || i >= _win_first + _win_count) {
		// Refill the window starting a little before i, so i+1 (interpolation) is inside too.
		_win_first = (i > 2) ? i - 2 : 0;
		uint32_t want = _hdr.count - _win_first;
		want = want > WINDOW ? WINDOW : want;
		off_t at = (off_t)sizeof(TrjHeader) + (off_t)_win_first * (off_t)sizeof(TrjSample);

		if (lseek(_fd, at, SEEK_SET) != at
		    || ::read(_fd, _win, want * sizeof(TrjSample)) != (ssize_t)(want * sizeof(TrjSample))) {
			_win_count = 0;
			return false;
		}

		_win_count = want;
	}

	out = _win[i - _win_first];
	return true;
}

} // namespace idps
