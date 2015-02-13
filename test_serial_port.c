#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <malloc.h>
#include <limits.h>
#include <poll.h>
#include <string.h>
#include <stropts.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>

#define DEBUG    1
#define pr_debug(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

#define  portname "/dev/ttyS2"

static void print_buf(void *buf, int size)
{
    int i;
    unsigned char *ptr = (unsigned char*)buf;
    for(i=0; i<size; i++) {
        printf("%02X ", ptr[i]);
        if(0 != i && 0 == ((i+1) % 16))
            printf("\n");
    }
    printf("\n");
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                pr_debug ("error %d from tcgetattr", errno);
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

        /* Make raw, this is the same as "minicom -H" */
        cfmakeraw(&tty);
        /* flush before setting it */
        tcflush(fd, TCIFLUSH);

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                pr_debug ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                pr_debug ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                pr_debug ("error %d setting term attributes", errno);
}

int main() 
{
    char buf[256];
    int n;

    int fd = open (portname, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        pr_debug ("error %d opening %s: %s", errno, portname, strerror (errno));
        return 0;
    }

    set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
    set_blocking (fd, 1);                // set blocking

    //write (fd, "hello!\n", 7);           // send 7 character greeting

    //usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
                                         // receive 25:  approx 100 uS per char transmit
    while(1) {
        n = read (fd, buf, sizeof(buf));  
        if (n>0) {
            pr_debug("read %i bytes\n",n);
            print_buf(buf,n);
        }
    }
    return 0;
}
