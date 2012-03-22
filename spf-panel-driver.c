#include <stdio.h>		/* Standard input/output definitions */
#include <string.h>		/* String function definitions */
#include <unistd.h>		/* UNIX standard function definitions */
#include <fcntl.h>		/* File control definitions */
#include <errno.h>		/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <stdlib.h>
#include <linux/input.h>
#include <linux/uinput.h>

/*
This is attempt 0.1.1 at makiing a uinput based SFP driver.


Latest changes:
0.1.1
- Added no-drag timer.
  - Only the first movement is used for the first 500ms, to prevent clicks from being misinterpretted as dragging. To drag, hold down and then drag.
  - I plan to experiment with this value to find a reasonable default value, and make it parametized.

Todo:
- Figure out how to probe monitor while in touch scan mode
  - (use a long select timeout on the read, to test monitor every 5-10 minutes or so)
- Detect many 0's in a row, indicated panel has been reset, and re-initialize it
- Better parameter handling (eg: $0 --port /dev/ttyS0 --timeout 250 )

*/



#define die(str, args...) do { \
        perror(str); \
        exit(EXIT_FAILURE); \
    } while(0)

int fd_serial, fd_uinput;
struct uinput_user_dev uidev;
struct termios options;


int
main (int argc, char *argv[])
{
  int x, y;
  struct input_event ev[2];
  struct input_event ev_button[2];
  struct input_event ev_sync;

  if (argc < 2)
  {
	printf("Usage: %s serial-device [uinput-device]\n",argv[0]);
	exit(-1);
  }

  open_serial_port (argv[1]);		// Open serial port
  set_serial_ops ();	//configure serial port optionS


// configure uinput
if (argc > 3)
  setup_uinput_dev(argv[2]);
else
  setup_uinput();


// Try 5 times to initialize screen:
  int n = 1;
  printf ("Attempting to initialize screen...\n");
  while (initialize_panel ())
    {
      n++;
      if (n > 5)
	{
	  printf ("Too many failures, exiting\n");
	  exit (1);
	}
    }

// input sync signal:
  memset (&ev_sync, 0, sizeof (struct input_event));
  ev_sync.type = EV_SYN;
  ev_sync.code = 0;
  ev_sync.value = 0;

// button press signals:
  memset (&ev_button, 0, sizeof (ev_button));
  ev_button[0].type = EV_KEY;
  ev_button[0].code = BTN_LEFT;
  ev_button[0].value = 0;
  ev_button[1].type = EV_KEY;
  ev_button[1].code = BTN_LEFT;
  ev_button[1].value = 1;


  struct timeval tv_start_click;
  struct timeval tv_current;

  char click_state = 0;
  char first_click = 0;
  unsigned char buffer[4];
  while (1)
    {

      read (fd_serial, &buffer, sizeof (buffer));
      if ((buffer[0] >= 0xFD) && (buffer[3] != 0xFF))
	continue;		// make sure its a valid position command.

      x = (int) ((buffer[1]) * 1024.0 / 0x5F);	// max is 5F
      y = (int) ((buffer[2]) * 1024.0 / 0x48);	// max is 48
      int old_click_state = click_state;
      if (buffer[0] == 0xFD)
	click_state = 0;
      else
	click_state = 1;

// If this is the first panel event, track time for no-drag timer
if (click_state != old_click_state && click_state == 1)
{
	first_click = 1;
	gettimeofday(&tv_start_click,NULL);
}
else
first_click=0;


// load X,Y into input_events
      memset (ev, 0, sizeof (ev));	//resets object
      ev[0].type = EV_ABS;
      ev[0].code = ABS_X;
      ev[0].value = x;
      ev[1].type = EV_ABS;
      ev[1].code = ABS_Y;
      ev[1].value = y;

// send X,Y
gettimeofday(&tv_current,NULL); // Only move to posision of click for first while - prevents accidental dragging.
if (time_elapsed_ms(&tv_start_click,&tv_current,500) || first_click)
{
      if (write (fd_uinput, &ev[0], sizeof (struct input_event)) < 0)
	die ("error: write");
      if (write (fd_uinput, &ev[1], sizeof (struct input_event)) < 0)
	die ("error: write");
}

// clicking
      if (click_state != old_click_state)
	  if (write(fd_uinput, &ev_button[click_state],sizeof (struct input_event)) < 0)
	    die ("error: write");
// Sync
      if (write (fd_uinput, &ev_sync, sizeof (struct input_event)) < 0)
	die ("error: write");
      usleep (100);
    }				// while 1

  if (ioctl (fd_uinput, UI_DEV_DESTROY) < 0)
    die ("error: ioctl");

  close (fd_uinput);

  return 0;
}


int time_elapsed_ms(struct timeval *start, struct timeval *end, int ms){
int difference = (end->tv_usec + end->tv_sec * 1000000 ) - (start->tv_usec + start ->tv_sec * 1000000);
if (difference > ms*1000)
	return 1;
return 0;
}

int setup_uinput(void){
  fd_uinput = open ("/dev/uinput", O_WRONLY | O_NONBLOCK);
  if (fd_uinput < 0)
    {				// Different platforms have diff locations. There's a proper way to check, im too lazy.
      fd_uinput = open ("/dev/input/uinput", O_WRONLY | O_NONBLOCK);
      if (fd_uinput < 0)
	die ("error: uinput");
    }

return configure_uinput ();
}

int setup_uinput_dev(char *ui_dev)
{
      fd_uinput = open (ui_dev, O_WRONLY | O_NONBLOCK);
      if (fd_uinput < 0)
	die ("error: uinput");
return configure_uinput();
}

int
configure_uinput (void)
{
  if (ioctl (fd_uinput, UI_SET_EVBIT, EV_KEY) < 0)
    die ("error: ioctl");

  if (ioctl (fd_uinput, UI_SET_KEYBIT, BTN_LEFT) < 0)
    die ("error: ioctl");

  if (ioctl (fd_uinput, UI_SET_EVBIT, EV_ABS) < 0)
    die ("error: ioctl");

  if (ioctl (fd_uinput, UI_SET_ABSBIT, ABS_X) < 0)
    die ("error: ioctl");

  if (ioctl (fd_uinput, UI_SET_ABSBIT, ABS_Y) < 0)
    die ("error: ioctl");


  memset (&uidev, 0, sizeof (uidev));
  snprintf (uidev.name, UINPUT_MAX_NAME_SIZE, "elo-sfp-touchpanel");
  uidev.id.bustype = BUS_RS232;
  uidev.id.vendor = 0x1;
  uidev.id.product = 0x1;
  uidev.id.version = 1;
  uidev.absmin[ABS_X] = 0;
  uidev.absmax[ABS_X] = 1023;
  uidev.absmin[ABS_Y] = 0;
  uidev.absmax[ABS_Y] = 1023;

  if (write (fd_uinput, &uidev, sizeof (uidev)) < 0)
    die ("error: write");

  if (ioctl (fd_uinput, UI_DEV_CREATE) < 0)
    die ("error: ioctl");

  return 0;
}

int
set_serial_ops (void)
{

  tcgetattr (fd_serial, &options);
  cfsetispeed (&options, B4800);
  cfsetospeed (&options, B4800);

  // Local, enabled, 8n1 

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;	// Mask the character size bits
  options.c_cflag |= CS8;	// Select 8 data bits
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 3;

  /*
   * Set the new options for the port...
   */
  tcsetattr (fd_serial, TCSANOW, &options);
  return 0;
}

int
open_serial_port (char *fd_device)
{
  fd_serial = open (fd_device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_serial == -1)
    {
      /*
       * Could not open the port.
       */

      perror ("open_port: Unable to open serial port");
      exit (1);
    }
  else
    fcntl (fd_serial, F_SETFL, 0);

  return 0;
}

/**
* This function sends the initalization commands to the panel.
*/
int
initialize_panel (void)
{
  int i;
  ioctl (fd_serial, TCSBRK, 0);	// Sends break

  usleep (1000000);		// 1s wait
  char cr[] = { 0x0D };

  for (i = 0; i < 10; i++)
    {
      usleep (100000);		// 100ms wait
      write (fd_serial, cr, 1);	//send CR
    }

  char reset[] = { 0x3C };	// Software reset
  char genreport[] = { 0x32 };	// Generate error report
  char transreport[] = { 0x44 };	// Transfer report

  usleep (100000);		// 100ms wait
  write (fd_serial, reset, 1);


// Error report on boot is not optional.
  usleep (100000);
  write (fd_serial, genreport, 1);

// Screen sent many 0's, flush input buffer:
  tcflush (fd_serial, TCIFLUSH);

  usleep (100000);
  write (fd_serial, transreport, 1);

// Receive report
//usleep(100000);
  unsigned char report[3] = { 0, 0, 0 };
  struct timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  fd_set serial;
  FD_ZERO (&serial);
  FD_SET (fd_serial, &serial);

// Use select to use timeout...
  if (select (fd_serial + 1, &serial, 0, 0, &tv) <= 0)
    {
      printf ("Timeout waiting for report...\n");
      return 1;
    }
  if (!FD_ISSET (fd_serial, &serial))
    {
      printf ("Did not find data to read...\n");
      return 1;
    }
  read (fd_serial, &report, 3);

  printf ("Report: %2X %2X %2X\n", report[0], report[1], report[2]);

// Test report
  if (report[0] != 0xF8 || report[1] != 0x00 || report[2] != 0xFF)
    {
      printf ("Failed to receive error report, or failure!\n");
      return 1;
    }


// Set main options
  char coord[] = { 0x23 };	// coordinate reporting
  char cont[] = { 0x27 };	// continous reporting
  char exit_mod[] = { 0x29 };	// Add_exit_point_modifier
  char touch_scan[] = { 0x2A };	// Touch scanning on


  usleep (100000);
  write (fd_serial, coord, 1);
  usleep (100000);
  write (fd_serial, cont, 1);
  usleep (100000);
  write (fd_serial, exit_mod, 1);
  usleep (100000);
  write (fd_serial, touch_scan, 1);


  printf ("Panel initialized!\n");
  return 0;
}
