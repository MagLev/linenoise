/* linenoise.h -- guerrilla line editing library against the idea that a
 * line editing lib needs to be 20,000 lines of C code.
 *
 * See linenoise.c for more information.
 *
 * Copyright (c) 2010, Salvatore Sanfilippo <antirez at gmail dot com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Redis nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LINENOISE_H
#define __LINENOISE_H

class LineReaderStateType ;

// All Functions have no effect, or return NULL on Windows

int LineRead(LineReaderStateType *state, const char *prompt, char *dest, 
	   size_t destSize, int echoing = 1);
  // function result 0 for success, -1 for error,
  //  -2 for EINTR as from ctl-C on interactive input,
  //  -3 for EOF on stdin
  // result includes '\n' as last character if users typed CR character to
  //   terminate the input of a line .
  // echoing==0 means don't echo what user types (as for password entry)
  
int LineReaderHistoryAdd(LineReaderStateType *state, const char *line);
int LineReaderHistorySetMaxLen(LineReaderStateType *state, int len);
int LineReaderHistoryMaxLen(LineReaderStateType *state);

int LineReaderHistoryLength(LineReaderStateType *state);

const char* LineReaderHistoryAt(LineReaderStateType *state, int idx);

void LineReadShutdown(LineReaderStateType *st);

LineReaderStateType* LineReaderAllocate();

#endif /* __LINENOISE_H */
