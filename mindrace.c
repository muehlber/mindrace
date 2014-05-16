
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include "ThinkGearStreamParser.h"


// #define  MINDWAVEPORT "/dev/ttyUSB1"
#define MINDWAVEPORT "/dev/rfcomm0"
#define BUFSIZE      1024

#define MWERROR      "e"
#define MWSTRAIGHT   "s"
#define MWRIGHT      "r"
#define MWLEFT       "l"


static unsigned char attention  = 0;
static unsigned char meditation = 0;
static unsigned char error      = 0;


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf (stderr, "error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                fprintf (stderr, "error %d from tcsetattr\n", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf (stderr, "error %d from tggetattr\n", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf (stderr, "error %d setting term attributes\n", errno);
}


void control (void)
{
  unsigned char dif;

  if (error) {
    printf (MWERROR);
  }

  if (attention >= meditation) {
    dif = attention - meditation;
    if (dif <= 10) {
      printf (MWSTRAIGHT);
    } else {
      printf (MWRIGHT);
    }
  } else
  if (meditation >= attention) {
    dif = meditation - attention;
    if (dif <= 10) {
      printf (MWSTRAIGHT);
    } else {
      printf (MWLEFT);
    }
  }

  fflush (stdout);

  return;
}


void mindhandler(unsigned char extendedCodeLevel,
                 unsigned char code, unsigned char numBytes,
                 const unsigned char *value, void *customData)
{

  switch (code) {
    case PARSER_CODE_BATTERY: {
      fprintf (stderr, "warning: device battery low\n");
      error = 100;
      break;
    }
    case PARSER_CODE_POOR_QUALITY: {
      if (numBytes == 1) {
#ifdef __DEBUG
        fprintf (stderr, "quality %i\n", value[0]);
#endif
        if (value[0] > 25) {
          error = 100;
          fprintf (stderr, "warning: poor signal quality\n");
        } else {
          error = 0;
        }
      }
      break;
    }
    case PARSER_CODE_ATTENTION: {
      if (numBytes == 1) {
        attention = value[0];
#ifdef __DEBUG
        fprintf (stderr, "attention %i\n", value[0]);
#endif
      }
      break;
    }
    case PARSER_CODE_MEDITATION: {
      if (numBytes == 1) {
        meditation = value[0];
#ifdef __DEBUG
        fprintf (stderr, "meditation %i\n", value[0]);
#endif
      }
      break;
    }
    default: {
      break;
    }
  }

  return;
}

int main (void)
{
  int res, i, n;
  unsigned char bytes[BUFSIZE];

  int fd = open (MINDWAVEPORT, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    fprintf (stderr, "error %d opening %s: %s\n", errno, MINDWAVEPORT,
      strerror (errno));
    return (1);
  }

  set_interface_attribs (fd, B57600, 0);   // 57,600 bps, 8n1 (no parity)
  set_blocking (fd, 0);                    // set non blocking


  ThinkGearStreamParser ctx;
  res = THINKGEAR_initParser(&ctx, PARSER_TYPE_PACKETS, &mindhandler, NULL);
  if (res) {
    fprintf (stderr, "error initialising THINKGEAR parser: %i\n", res);
    return (1);
  }

  while (1) {
    n = read (fd, bytes, BUFSIZE); // read up to BUFSIZE bytes from the device

    if (n > 0) {
#ifdef __DEBUG
      fprintf (stderr, "%i bytes read\n", n);
#endif
      for (i = 0; i < n; i++) {
        res = THINKGEAR_parseByte(&ctx, bytes[i]);
        if (res < 0) {
          fprintf (stderr, "error parsing byte: %i\n", res);
          return (1);
        }
        control();  // output robot control char
      }
    } else
    if (n < 0) {
      fprintf (stderr, "error %d reading %s: %s\n", errno, MINDWAVEPORT,
        strerror (errno));
      return (1);
    }

    usleep (n * 100);  // sleep approx 100 uS per char transmit
  }

  return (0);
}

