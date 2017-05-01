/*
* This file is part of the ADEM project.
*
* ADEM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License,�
* (at your option) any later version.
*
* ADEM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ADEM.  If not, see <http://www.gnu.org/licenses/>.
*
* Copyright 2016 Lieven Blancke, Koen Verstringe
*
*/

/* 
storeAndForwardBuf
is based on cbuf - circular buffer - impelementation with a few additions
it has a pointer to the next position to read after the peek, which can be
used in the forward mode to get the data and forward it to an external server.
Once tha ack (acknowledge) is received the ack function moves the begin pointer 
to the next pointer. nack reset the next pointer to the begin pointer.
The rest of the functionality is as in cbuf.

The class has been renamed from cbuf to storeAndForwardBuf
*/

/*
cbuf.h - Circular buffer implementation
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


#ifndef __storeAndForwardBuf_h
#define __storeAndForwardBuf_h

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

class storeAndForwardBuf {
public:
	storeAndForwardBuf(size_t size);
	~storeAndForwardBuf();

	size_t resizeAdd(size_t addSize);
	size_t resize(size_t newSize);
	size_t available() const;
	size_t size();

	size_t room() const;

	inline bool empty() const {
		return _begin == _end;
	}

	inline bool full() const {
		return (wrap_if_bufend(_end + 1) == _begin) || (_size == 0);
	}

	int peek();
	size_t peek(char *dst, size_t size);

	int read();
	size_t read(char* dst, size_t size);

	size_t write(char c);
	size_t write(const char* src, size_t size);

	void ack();
	void nack();

	void flush();
	size_t remove(size_t size);

	storeAndForwardBuf *next;

	//void printbuf();
	//void dump();

private:
	inline char* wrap_if_bufend(char* ptr) const {
		return (ptr == _bufend) ? _buf : ptr;
	}

	//char buffer[500];
	size_t _size;// = 500;
	char* _buf;
	const char* _bufend;
	char* _begin;
	char* _end;
	char* _next;
	char* _tmptr;
};

#endif//__storeAndForwardBuf_h