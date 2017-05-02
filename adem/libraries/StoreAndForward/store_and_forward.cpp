/*
storeAndForwardBuf.cpp - Circular buffer implementation
Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
This file is part of the esp8266 core for Arduino environment.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "store_and_forward.h"
#include <ctype.h>

#define DEBUG

#ifdef DEBUG
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif


storeAndForwardBuf::storeAndForwardBuf(size_t size) :
next(NULL), _size(size), _buf(new char[size]), _bufend(_buf + size), _begin(_buf), _end(_begin) {
	if (_buf == 0) _size = 0;
}

storeAndForwardBuf::~storeAndForwardBuf() {
	delete[] _buf;
}

size_t storeAndForwardBuf::resizeAdd(size_t addSize) {
	return resize(_size + addSize);
}

size_t storeAndForwardBuf::resize(size_t newSize) {

	size_t bytes_available = available();

	// not lose any data
	// if data can be lost use remove or flush before resize
	if ((newSize < bytes_available) || (newSize == _size)) {
		return _size;
	}

	char *newbuf = new char[newSize];
	char *oldbuf = _buf;

	if (!newbuf) {
		return _size;
	}

	if (_buf) {
		read(newbuf, bytes_available);
		memset((newbuf + bytes_available), 0x00, (newSize - bytes_available));
	}

	_begin = newbuf;
	_end = newbuf + bytes_available;
	_bufend = newbuf + newSize;
	_size = newSize;

	_buf = newbuf;
	delete[] oldbuf;

	return _size;
}

size_t storeAndForwardBuf::available() const {
	if (_end >= _begin) {
		return _end - _begin;
	}
	return _size - (_begin - _end);
}

size_t storeAndForwardBuf::size() {
	return _size;
}

size_t storeAndForwardBuf::room() const {
	if (_end >= _begin) {
		return _size - (_end - _begin) - 1;
	}
	return _begin - _end - 1;
}

int storeAndForwardBuf::peek() {
	if (empty())
		return -1;

	return static_cast<int>(*_begin);
}

size_t storeAndForwardBuf::peek(char *dst, size_t size) {
	size_t bytes_available = available();
	__LOG("store and forward peek : ");__LOG(bytes_available);__LOG(" for "); __LOG(size);
	size_t size_to_read = (size < bytes_available) ? size : bytes_available;
	size_t size_read = size_to_read;
	char * begin = _begin;
	if (_end < _begin && size_to_read >(size_t) (_bufend - _begin)) {
		size_t top_size = _bufend - _begin;
		memcpy(dst, _begin, top_size);
		begin = _buf;
		size_to_read -= top_size;
		dst += top_size;
	}
	memcpy(dst, begin, size_to_read);
	_next = wrap_if_bufend(_begin + size_to_read);
	__LOG(" and read ");__LOGLN(size_read);
	return size_read;
}

int storeAndForwardBuf::read() {
	if (empty())
		return -1;
    __LOG("----- store and forward "); __LOGLN( _end - _begin);
	char result = *_begin;
	_begin = wrap_if_bufend(_begin + 1);
	return static_cast<int>(result);
}

size_t storeAndForwardBuf::read(char* dst, size_t size) {
	__LOG("----- store and forward ");__LOG( _end - _next);__LOG(" / ");__LOGLN( _end - _begin);
	size_t bytes_available = available();
	size_t size_to_read = (size < bytes_available) ? size : bytes_available;
	size_t size_read = size_to_read;
	if (_end < _begin && size_to_read >(size_t) (_bufend - _begin)) {
		size_t top_size = _bufend - _begin;
		memcpy(dst, _begin, top_size);
		_begin = _buf;
		size_to_read -= top_size;
		dst += top_size;
	}
	memcpy(dst, _begin, size_to_read);
	_next = wrap_if_bufend(_begin + size_to_read);
	
	__LOG("+++++ store and forward : "); __LOG( _end - _next);__LOG(" / ");__LOGLN( _end - _begin);
	return size_read;
}

size_t storeAndForwardBuf::write(char c) {
	if (full())
		return 0;
    __LOG("+++++ store and forward ");__LOG( _end - _next);__LOG(" / ");__LOGLN( _end - _begin);
	*_end = c;
	_end = wrap_if_bufend(_end + 1);
	return 1;
}

size_t storeAndForwardBuf::write(const char* src, size_t size) {
	__LOG("+++++ store and forward ");__LOG( _end - _next);__LOG(" / ");__LOGLN( _end - _begin);
	size_t bytes_available = room();
	size_t size_to_write = (size < bytes_available) ? size : bytes_available;
	size_t size_written = size_to_write;
	if (_end >= _begin && size_to_write >(size_t) (_bufend - _end)) {
		size_t top_size = _bufend - _end;
		memcpy(_end, src, top_size);
		_end = _buf;
		size_to_write -= top_size;
		src += top_size;
	}
	memcpy(_end, src, size_to_write);
	_end = wrap_if_bufend(_end + size_to_write);
	return size_written;
}

void storeAndForwardBuf::ack() {
	_begin = _next;
    __LOG("+++++ store and forward : "); __LOG( _end - _next);__LOG(" / ");__LOGLN( _end - _begin);

}

void storeAndForwardBuf::nack() {
	_next = _begin;
    __LOG("+++++ store and forward : "); __LOG( _end - _next);__LOG(" / ");__LOGLN( _end - _begin);

}

void storeAndForwardBuf::flush() {
	_begin = _buf;
	_end = _buf;
	_next = _buf;
}

size_t storeAndForwardBuf::remove(size_t size) {
	size_t bytes_available = available();
	if (size >= bytes_available) {
		flush();
		return 0;
	}
	size_t size_to_remove = (size < bytes_available) ? size : bytes_available;
	if (_end < _begin && size_to_remove >(size_t) (_bufend - _begin)) {
		size_t top_size = _bufend - _begin;
		_begin = _buf;
		size_to_remove -= top_size;
	}
	_begin = wrap_if_bufend(_begin + size_to_remove);
	_next = _begin;
	return available();
}