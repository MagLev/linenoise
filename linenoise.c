/* linenoise.c -- guerrilla line editing library against the idea that a
 * line editing lib needs to be 20,000 lines of C code.
 *
 * You can find the latest source code at:
 * 
 *   http://github.com/antirez/linenoise
 *
 *   Gemstone note: (this file started with linenoise.c from above URL 
                     as of 24 March 2010 , and has been modified)
 *
 * Does a number of crazy assumptions that happen to be true in 99.9999% of
 * the 2010 UNIX computers around.
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
 *
 * References:
 * - http://invisible-island.net/xterm/ctlseqs/ctlseqs.html
 * - http://www.3waylabs.com/nw/WWW/products/wizcon/vt220.html
 *
 * Todo list:
 * - Switch to gets() if $TERM is something we can't support.
 * - Filter bogus Ctrl+<char> combinations.
 * - Win32 support
 *
 * Bloat:
 * - Completion?
 * - History search like Ctrl+r in readline?
 *
 * List of escape sequences used by this program, we do everything just
 * with three sequences. In order to be so cheap we may have some
 * flickering effect with some slow terminal, but the lesser sequences
 * the more compatible.
 *
 * CHA (Cursor Horizontal Absolute)
 *    Sequence: ESC [ n G
 *    Effect: moves cursor to column n
 *
 * EL (Erase Line)
 *    Sequence: ESC [ n K
 *    Effect: if n is 0 or missing, clear from cursor to end of line
 *    Effect: if n is 1, clear from beginning of line to cursor
 *    Effect: if n is 2, clear entire line
 *
 * CUF (CUrsor Forward)
 *    Sequence: ESC [ n C
 *    Effect: moves cursor forward of n chars
 * 
 */
#include "flag.ht"
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int checkEintr();

#if defined(FLG_UNIX)
static int getColumns(void) 
{
    struct winsize ws;

    int winSz = TIOCGWINSZ; // macro uses  'unsigned long' on Mac
    for (;;) {
      if (ioctl(1, winSz, &ws) == -1) {
        int errNum = errno;
        if (errNum != EINTR) {
          return 80;
        }
        int status = checkEintr();
        if (status == -2) {
          return -2; // SIGINT received
        }
        // retry after EINTR
      } else {
        return ws.ws_col;
      }
    }
}
#endif

#include "global.ht"
// from this point forwards  'long' is not allowed , use int or in64

#include "linenoise.h"

#if defined(FLG_UNIX)
#include "utlansi.hf"
#include "unixio.hf"
#include "host.hf"
#include "hostshr.hf"

class LineReaderStateType {
 public: 
  struct termios orig_termios; /* in order to restore at exit */
  struct termios rawin_out_termios;
  struct termios rawin_termios;
  int orig_termios_valid;
  int raw_inmode ; 
  int raw_outmode;
  // int atexit_registered ; 
  int history_max_len ;
  int history_len ;
  int unsupportedTerm;
  int readBufLimit;
  int readBufIdx;
  char **history ;
  char readBuf[20480];
  
  void initialize() {
    raw_inmode = 0;
    raw_outmode = 0;
    orig_termios_valid = 0;
    //atexit_registered = 0;
    history_max_len = 100;
    history_len = 0;
    history = NULL; 
    unsupportedTerm = -1;
    readBufLimit = 0;
    readBufIdx = 0;
  }
};

LineReaderStateType* LineReaderAllocate()
{
  LineReaderStateType *st = (LineReaderStateType*)UtlMallocF(sizeof(*st), __LINE__, __FILE__);
  st->initialize();
  return st;
}

static const char* unsupported_term[] = {"dumb","cons25",NULL};

static int isUnsupportedTerm(void) 
{
    char *term = getenv("TERM");
    int j;

    if (term == NULL) return 0;
    for (j = 0; unsupported_term[j]; j++)
        if (!strcasecmp(term,unsupported_term[j])) return 1;
    return 0;
}

static void freeHistory(LineReaderStateType *st) 
{
    if (st->history) {
        int j;

        for (j = 0; j < st->history_len; j++)
            UtlFree(st->history[j]);
        UtlFree(st->history);
    }
}

static char* lineDup(const char* str)
{
  intptr_t len = strlen(str);
  char* res = (char*)UtlMalloc_(len + 1, __LINE__);
  if (res == NULL) {
    return res;
  }
  memcpy(res, str, len);
  res[len] = '\0';
  return res;
}

static int checkEintr()
{
  // function result 0 if caller should retry IO operation after EINTR,
  //  -2 for EINTR as from ctl-C on interactive input
  //  -1 for other error

  int errNum = errno;
  if (errNum == EINTR && ! HostGetSigTerm()) {
    if (UnixIoIsInterrupted()) { 
      return -2;  // SIGINT received
    }
    errno = 0;
    return 0; // caller should retry
  }
  return -1;
}

static int writeFd(int fd, const char* buf, size_t len)
{
  // function result 0 for success,
  //  -1 for error,
  //  -2 for EINTR as from ctl-C on interactive input

  const char *src = buf;
  ssize_t numLeftToWrite = len;
  while (numLeftToWrite > 0) {
    errno = 0;
    ssize_t writeResult = write(fd, src, numLeftToWrite);
    if (writeResult == -1) {
      int status = checkEintr();
      if (status < 0) {
        return status;
      } 
      continue;
    }
    numLeftToWrite -= writeResult;
    src += writeResult ;
  } while (numLeftToWrite > 0);

  return 0;
}

#if defined(FLG_DEBUG)
// array to capture sizes of large reads for debugging flow control issues
enum { READ_HIST_SIZE = 20 };
static int readHist[READ_HIST_SIZE];
static int readHistLen = 0;
#endif

static int readChar(LineReaderStateType *st, int fd, char *dest)
{
  // function result 1 for success, -3 for EOF,
  //  -1 for error,
  //  -2 for EINTR as from ctl-C on interactive input

  // read as much data as is available to handle a paste of
  // many lines without loosing input.
  int idx = st->readBufIdx;
  if (idx >= st->readBufLimit) {
    for (;;) {
      errno = 0;
      ssize_t readResult = read(fd, st->readBuf, sizeof(st->readBuf) );
#if defined(FLG_DEBUG)
      if (readResult == 1) {
        readHistLen = 0;
      }
      if (readHistLen < READ_HIST_SIZE - 1) {
        readHist[readHistLen] = readResult;
        readHistLen += 1;
        readHist[readHistLen] = 0;
      }
#endif
      if (readResult <= 0) {
        if (readResult == 0)
          return -3; // EOF
        int status = checkEintr();
        if (status < 0)
          return status;
      } else {
        idx = 0;
        st->readBufLimit = readResult;
        break;
      }
    }
  }
  *dest = st->readBuf[idx];
  st->readBufIdx = idx + 1;
  return 1;
}

static int saveOrigMode(LineReaderStateType *st, int fd)
{
  if (! st->orig_termios_valid) {
    for (;;) {
      if (tcgetattr(fd, &st->orig_termios) == 0)
        break;
      int status = checkEintr();
      if (status < 0)
        return status;
    }
    st->orig_termios_valid = 1;
  }
  return 0;
}

static int setRawOutMode(LineReaderStateType *st, int fd, int val) 
{
  if (st->raw_outmode != val) {
    UTL_ASSERT( st->orig_termios_valid);
    for (;;) {
      if (isatty(STDIN_FILENO))
        break;
      int status = checkEintr();
      if (status < 0) {
	if (status == -1)
	  errno = ENOTTY;
	return status;
      }
    }
    struct termios raw;
    raw = st->orig_termios;  /* modify the original mode */
    if (val) {
      raw = st->rawin_out_termios;
    } else {
      raw = st->rawin_termios;
    }
    for (;;) {
      if (tcsetattr(fd, TCSADRAIN, &raw) == 0)
        break;
      int status = checkEintr();
      if (status < 0) 
	return status;
    }
    st->raw_outmode = val;
  }
  return 0;
}

static int enableRawIO(LineReaderStateType *st, int fd) 
{
  if (! st->raw_inmode) {
    UTL_ASSERT( st->orig_termios_valid);
    for (;;) {
      if (isatty(STDIN_FILENO))
        break;
      int status = checkEintr();
      if (status < 0) {
	if (status == -1)
	  errno = ENOTTY;
	return status;
      }
    }
    struct termios raw;
    raw = st->orig_termios;  /* modify the original mode */
    /* input modes: no break, no CR to NL, no parity check, no strip char,
     * no start/stop output control. */
    raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    /* control modes - set 8 bit chars */
    raw.c_cflag |= (CS8);
    /* local modes - choing off, canonical off, no extended functions*/
    // do not disable the signal chars (^Z,^C) (do not include ISIG in mask 
    tcflag_t mask = ECHO | ICANON | IEXTEN ;
    raw.c_lflag &= ~ mask ;
    /* control chars - set return condition: min number of bytes and timer.
     * We want read to return every single byte, without timeout. */
    raw.c_cc[VMIN] = 1; raw.c_cc[VTIME] = 0; /* 1 byte, no timer */

    st->rawin_termios = raw;

    raw.c_oflag &= ~(OPOST); // disable post processing

    st->rawin_out_termios = raw;
    /* put terminal in raw mode after flushing output */
    for (;;) {
      if (tcsetattr(fd, TCSADRAIN, &raw) == 0)
        break;
      int status = checkEintr();
      if (status < 0) 
	return status;
    }
    st->raw_inmode = 1;
    st->raw_outmode = 1;
    return 0;
  } else {
    return setRawOutMode(st, fd, 1);
  }
}

static int restoreIOModes(LineReaderStateType *st, int fd)
{
  // used after a SIGCONT detected
  struct termios raw;  
  if (st->raw_inmode && st->raw_outmode) {
    raw = st->rawin_out_termios;
  } else if (st->raw_inmode) {
    raw = st->rawin_termios;
  } else {
    return 0;
  }
  for (;;) {
    if (tcsetattr(fd, TCSADRAIN, &raw) == 0)
      break;
    int status = checkEintr();
    if (status < 0)
      return status;
  }
  return 0;
}

static int disableRawIO(LineReaderStateType *st, int fd)  
{
  if (st->orig_termios_valid) {
    for (;;) {
      if (tcsetattr(fd, TCSADRAIN, &st->orig_termios) == 0) 
        break;
      int status = checkEintr();
      if (status < 0) 
        return status;
    } 
    st->raw_outmode = 0;
    st->raw_inmode = 0;
  } 
  return 0;
}

void LineReadShutdown(LineReaderStateType *st)
{
  disableRawIO(st, STDIN_FILENO);
  freeHistory(st);
}


static int refreshLine(int fd, const char *prompt, char *buf, size_t len, size_t pos, 
				size_t cols) 
{
    size_t plen = strlen(prompt);
    
    while((plen+pos) >= cols) {
        buf++;
        len--;
        pos--;
    }
    while (plen+len > cols) {
        len--;
    }

    /* Cursor to left edge */
    char leftEdge[64];
    snprintf(leftEdge, sizeof(leftEdge),"\x1b[0G");
    int leftEdgeLen = strlen(leftEdge);
    int status = writeFd(fd, leftEdge, leftEdgeLen);
    if (status < 0) 
      return status;
    
    /* Write the prompt and the current buffer content */
    if (plen > 0) {
      status = writeFd(fd,prompt, plen);
      if (status < 0) 
        return status;
    }

    status = writeFd(fd,buf,len);
    if (status < 0)
      return status;

    /* Erase to right */
    char seq[64];
    snprintf(seq, sizeof(seq),"\x1b[0K");
    status = writeFd(fd,seq,strlen(seq));
    if (status < 0)
      return status;

    /* Move cursor to original position. */
    if (pos+plen > 0) {
      snprintf(seq, sizeof(seq),"\x1b[0G\x1b[%dC", (int)(pos+plen));
      int seqLen = strlen(seq);
      return writeFd(fd,seq, seqLen);
    } else {
      return writeFd(fd, leftEdge, leftEdgeLen); 
    }
}

static int linenoisePrompt(LineReaderStateType *st,
		int fd, char *buf, size_t buflen, const char *prompt) 
{
  // function result 0 for success or -1 for error,
  //  -2 for EINTR as from ctl-C on interactive input
  //  -3 for EOF on stdin , 
  // -4 for line too long

    size_t plen = strlen(prompt);
    size_t pos = 0;
    size_t len = 0;
    size_t cols = getColumns();
    if (cols < 0) {
      int status = cols;
      return status;
    } 
    int history_index = 0;

    buf[0] = '\0';
    buflen--; /* Make sure there is always space for the nulterm */

    /* The latest history entry is always our current buffer, that
     * initially is just an empty string. */
    LineReaderHistoryAdd(st, "");
    
    if (plen > 0) {
      int status = writeFd(fd,prompt,plen);
      if (status < 0) 
         return status;
   }
    
    while(1) {
        char c;
        char seq[2];

        int status = readChar(st, fd, &c);
        if (status <= 0) 
          return status; // error or EOF
        
        switch(c) {
        case 10:   // ascii LF , can occur after a ctrl-Z / fg transition.
                   // could not easily make a SIGCONT handler work.
                   // handle as if it was ASCII CR
            status = restoreIOModes(st, fd);  
            if (status < 0)
              return status;
            st->history_len--;
            return 0;
        case 13:    /* enter (ascii CR) */
            st->history_len--;
            return 0;
        case 4:     /* ctrl-d */
            st->history_len--;
            return (len == 0) ? -1 : 0;
        case 3:     /* ctrl-c */
            if (HostGetSigTerm()) {
              return -1;
            }
            errno = EAGAIN;
            return -2;
        case 127:   /* backspace */
        case 8:     /* ctrl-h */
            if (pos > 0 && len > 0) {
                memmove(buf+pos-1,buf+pos,len-pos);
                pos--;
                len--;
                buf[len] = '\0';
                status = refreshLine(fd,prompt,buf,len,pos,cols);
                if (status < 0) 
                  return status;
            }
            break;
        case 20:    /* ctrl-t */
            if (pos > 0 && pos < len) {
                int aux = buf[pos-1];
                buf[pos-1] = buf[pos];
                buf[pos] = aux;
                if (pos != len-1) pos++;
                status = refreshLine(fd,prompt,buf,len,pos,cols);
                if (status < 0) 
                  return status;
            }
            break;
        case 2:     /* ctrl-b */
            goto left_arrow;
        case 6:     /* ctrl-f */
            goto right_arrow;
        case 16:    /* ctrl-p */
            seq[1] = 65;
            goto up_down_arrow;
        case 14:    /* ctrl-n */
            seq[1] = 66;
            goto up_down_arrow;
            break;
        case 27:    /* escape sequence */
            status = readChar(st, fd, &seq[0]);
            if (status < 0)
              return status;
            status = readChar(st, fd, &seq[1]);
            if (status < 0)
              return status;
            if (seq[0] == 91 && seq[1] == 68) {
left_arrow:
                /* left arrow */
                if (pos > 0) {
                    pos--;
                    status = refreshLine(fd,prompt,buf,len,pos,cols);
                    if (status < 0) 
                      return status;
                }
            } else if (seq[0] == 91 && seq[1] == 67) {
right_arrow:
                /* right arrow */
                if (pos != len) {
                    pos++;
                    status = refreshLine(fd,prompt,buf,len,pos,cols);
                    if (status < 0) 
                      return status;
                }
            } else if (seq[0] == 91 && (seq[1] == 65 || seq[1] == 66)) {
up_down_arrow:
                /* up and down arrow: history */
                if (st->history_len > 1) {
                    /* Update the current history entry before 
                     * overwriting it with the next one. */
                    UtlFree(st->history[st->history_len -1 -history_index]);
                    st->history[st->history_len -1 -history_index] = lineDup(buf);
                    /* Show the new entry */
                    history_index += (seq[1] == 65) ? 1 : -1;
                    if (history_index < 0) {
                        history_index = 0;
                        break;
                    } else if (history_index >= st->history_len) {
                        history_index = st->history_len -1;
                        break;
                    }
                    strncpy(buf, st->history[st->history_len -1 -history_index], buflen);
                    buf[buflen] = '\0';
                    len = pos = strlen(buf);
                    status = refreshLine(fd,prompt,buf,len,pos,cols);
                    if (status < 0) 
                      return status;
                }
            }
            break;
        default:
            if (len < buflen) {
                if (len == pos) {
                    buf[pos] = c;
                    pos++;
                    len++;
                    buf[len] = '\0';
                    if (plen+len < cols) {
                        /* Avoid a full update of the line in the
                         * trivial case. */
                        status = writeFd(fd,&c,1);
                        if (status < 0)
                          return status;
                    } else {
                        status = refreshLine(fd,prompt,buf,len,pos,cols);
                        if (status < 0) 
                          return status;
                    }
                } else {
                    memmove(buf+pos+1,buf+pos,len-pos);
                    buf[pos] = c;
                    len++;
                    pos++;
                    buf[len] = '\0';
                    status = refreshLine(fd,prompt,buf,len,pos,cols);
                    if (status < 0) 
                      return status;
                }
            } else {
              buf[len] = '\0'; 
              return -4; // line too long
            }
            break;
        case 21: /* Ctrl+u, delete the whole line. */
            buf[0] = '\0';
            pos = len = 0;
            status = refreshLine(fd,prompt,buf,len,pos,cols);
            if (status < 0) 
               return status;
            break;
        case 11: /* Ctrl+k, delete from current to end of line. */
            buf[pos] = '\0';
            len = pos;
            status = refreshLine(fd,prompt,buf,len,pos,cols);
            if (status < 0) 
               return status;
            break;
        case 1: /* Ctrl+a, go to the start of the line */
            pos = 0;
            status = refreshLine(fd,prompt,buf,len,pos,cols);
            if (status < 0) 
               return status;
            break;
        case 5: /* ctrl+e, go to the end of the line */
            pos = len;
            status = refreshLine(fd,prompt,buf,len,pos,cols);
            if (status < 0) 
               return status;
            break;
        }
    }
    return 0;
}

static int linenoiseRaw(LineReaderStateType *st, 
			char *buf, size_t buflen, const char *prompt) 
{
  // function result 0 for success or -1 for error,
  //  -2 for EINTR as from ctl-C on interactive input
  //  -3 for EOF on stdin
  // -4 for line too long

    int fd = STDIN_FILENO;

    if (buflen <= 0) {
      errno = EINVAL;
      return -1;
    }
    int status = saveOrigMode(st, fd);
    if (status < 0)
      return status;

    status = enableRawIO(st, fd);
    if (status < 0)
      return status;

    status = linenoisePrompt(st, fd, buf, buflen, prompt);
    int dStatus = setRawOutMode(st, fd, 0);
    if (status == 0) {
      status = dStatus;
    }
    printf("\n");
    return status;
}

int LineRead(LineReaderStateType *st, const char *prompt, char *dest, size_t destSize)
{
  // function result 0 for success,  -1 for error,
  //  -2 for EINTR as from ctl-C on interactive input
  // -3 for EOF on stdin
  // -4 for line too long

    if (st->unsupportedTerm < 0) {
      st->unsupportedTerm = isUnsupportedTerm();
    }
    if (st->unsupportedTerm == 1) {
        printf("%s",prompt);
        fflush(stdout);
        // if (fgets(dest, destSize, stdin) == NULL) return NULL;
        int status = UnixIoFgets_(dest, destSize, stdin);
        if (status == 0) { 
          size_t len = strlen(dest);
          while(len && (dest[len-1] == '\n' || dest[len-1] == '\r')) {
            len--;
            dest[len] = '\0';
          }
        }
        return status;
    } else {
        return linenoiseRaw(st, dest, destSize ,prompt);
    }
}

/* Using a circular buffer is smarter, but a bit more complex to handle. */
int LineReaderHistoryAdd(LineReaderStateType *st, const char *line) 
{

    if (st->history_max_len == 0) 
      return 0;

    if (st->history == 0) {
        st->history = (char**)UtlMalloc_(sizeof(char*) * st->history_max_len, __LINE__);
        if (st->history == NULL) return 0;
        memset(st->history, 0, (sizeof(char*) * st->history_max_len));
    }
    char* linecopy = lineDup(line);
    if (! linecopy) return 0;
    if (st->history_len == st->history_max_len) {
        memmove(st->history, st->history+1, 
                sizeof(char*) * (st->history_max_len-1));
        st->history_len--;
    }
    st->history[st->history_len] = linecopy;
    st->history_len++;
    return 1;
}

int LineReaderHistoryMaxLen(LineReaderStateType *st)
{
  return st->history_max_len;
}

int LineReaderHistorySetMaxLen(LineReaderStateType *st, int len) 
{
    if (len < 1) return 0;
    if (st->history && len != st->history_max_len) {
        int tocopy = st->history_len;

        char** nHist = (char**)UtlMalloc_(sizeof(char*)*len, __LINE__ );
        if (nHist == NULL) return 0;
        if (len < tocopy) tocopy = len;
        memcpy(nHist, 
              st->history + (st->history_max_len - tocopy), 
               sizeof(char*)*tocopy);
        UtlFree(st->history);
        st->history = nHist;
    }
    st->history_max_len = len;
    if (st->history_len > st->history_max_len)
        st->history_len = st->history_max_len;
    return 1;
}

#else
// -------------------------  Windows implementations
class LineReaderStateType;

LineReaderStateType* LineReaderAllocate()
{
  return NULL;
}

int LineRead(LineReaderStateType *state, const char *prompt, char *dest, size_t destSize)
{
  return -1;
}

int LineReaderHistoryAdd(LineReaderStateType *state, const char *line)
{
  return 0;
} 

int LineReaderHistorySetMaxLen(LineReaderStateType *state, int len)
{
  return 0;
}
int LineReaderHistoryMaxLen(LineReaderStateType *state)
{
  return 0;
}

void LineReadShutdown(LineReaderStateType *st)
{
  return;
}
#endif /* FLG_UNIX*/
