/*
    This file is part of ttymidi.

    ttymidi is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ttymidi is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ttymidi.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <argp.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <alsa/asoundlib.h>
#include <linux/serial.h>

#define MAX_DEV_STR_LEN		32
#define DEFAULT_SERIAL		"/dev/ttyUSB0"
#define DEFAULT_NAME		"ttymidi"
#define MIDI_NOTE_OFF		0x80
#define MIDI_NOTE_ON		0x90
#define MIDI_POLY_KEY_PRESSURE	0xA0
#define MIDI_CONTROL_CHANGE	0xB0
#define MIDI_PROGRAM_CHANGE	0xC0
#define MIDI_MONO_KEY_PRESSURE	0xD0
#define MIDI_PITCH_BEND		0xE0
#define MAX_MSG_SIZE		1024

static struct arguments {
	int silent;
	int verbose;
	int printonly;
	int baudrate;
	char serialdevice[MAX_DEV_STR_LEN];
	char name[MAX_DEV_STR_LEN];
} arguments = {
	.baudrate = B115200,
	.serialdevice = DEFAULT_SERIAL,
	.name = DEFAULT_NAME,
};

static struct argp_option options[] = {
	{"serialdevice" , 's', "DEV" , 0, "Serial device to use. Default = /dev/ttyUSB0" },
	{"baudrate"     , 'b', "BAUD", 0, "Serial port baud rate. Default = 115200" },
	{"verbose"      , 'v', 0     , 0, "For debugging: Produce verbose output" },
	{"printonly"    , 'p', 0     , 0, "Super debugging: Print values read from serial -- and do nothing else" },
	{"quiet"        , 'q', 0     , 0, "Don't produce any output, even when the print command is sent" },
	{"name"		, 'n', "NAME", 0, "Name of the Alsa MIDI client. Default = ttymidi" },
	{ 0 }
};

static char doc[] = "ttymidi - Connect serial port devices to ALSA MIDI programs!";

static bool run = true;

static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
	/* Get the input argument from argp_parse, which we
	   know is a pointer to our arguments structure. */
	struct arguments *arguments = state->input;
	int baud_temp;

	switch (key) {
		case 'p':
			arguments->printonly = 1;
			break;
		case 'q':
			arguments->silent = 1;
			break;
		case 'v':
			arguments->verbose = 1;
			break;
		case 's':
			if (!arg)
				break;
			strncpy(arguments->serialdevice, arg, MAX_DEV_STR_LEN);
			break;
		case 'n':
			if (!arg)
				break;
			strncpy(arguments->name, arg, MAX_DEV_STR_LEN);
			break;
		case 'b':
			if (!arg)
				break;
			baud_temp = strtol(arg, NULL, 0);
			if (baud_temp != EINVAL && baud_temp != ERANGE)
				switch (baud_temp) {
					case 1200:
						arguments->baudrate = B1200;
						break;
					case 2400:
						arguments->baudrate = B2400;
						break;
					case 4800:
						arguments->baudrate = B4800;
						break;
					case 9600:
						arguments->baudrate = B9600;
						break;
					case 19200:
						arguments->baudrate = B19200;
						break;
					case 38400:
						arguments->baudrate = B38400;
						break;
					case 57600:
						arguments->baudrate = B57600;
						break;
					case 115200:
						arguments->baudrate = B115200;
						break;
					default:
						printf("Baud rate %i is not supported.\n",baud_temp);
						exit(1);
				}

		case ARGP_KEY_ARG:
		case ARGP_KEY_END:
			break;

		default:
			return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

static struct argp argp = {
	.options = options,
	.parser = parse_opt,
	.doc = doc,
};

static int open_seq(snd_seq_t** seq)
{
	int port_id;

	if (snd_seq_open(seq, "default", SND_SEQ_OPEN_DUPLEX, 0) < 0) {
		fprintf(stderr, "Error opening ALSA sequencer.\n");
		return -1;
	}

	snd_seq_set_client_name(*seq, arguments.name);

	if ((port_id = snd_seq_create_simple_port(*seq, "MIDI out",
						  SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
						  SND_SEQ_PORT_TYPE_APPLICATION)) < 0)
		fprintf(stderr, "Error creating sequencer port.\n");

	return port_id;
}

static void build_midi_termios(struct termios *newtio)
{
	/*
	 * BAUDRATE : Set bps rate. You could also use cfsetispeed and cfsetospeed
	 * CS8      : 8n1 (8bit, no parity, 1 stopbit)
	 * CLOCAL   : local connection, no modem contol
	 * CREAD    : enable receiving characters
	 */
	newtio->c_cflag = arguments.baudrate | CS8 | CLOCAL | CREAD;

	/* IGNPAR  : ignore bytes with parity errors */
	newtio->c_iflag = IGNPAR;

	/* Raw output */
	newtio->c_oflag = 0;

	/*
	 * ICANON  : enable canonical input
	 * disable all echo functionality, and don't send signals to calling program
	 */
	newtio->c_lflag = 0; /* non-canonical */

	/* set up: we'll be reading 4 bytes at a time */
	newtio->c_cc[VTIME] = 0;	/* inter-character timer unused */
	newtio->c_cc[VMIN] = 1;		/* blocking read until n character arrives */

}

static void exit_cli(int sig) {
	run = false;
	printf("\rttymidi closing down ... ");
}

static void parse_midi_command(snd_seq_t* seq, int port_out_id, char *buf)
{
	snd_seq_event_t ev;
	int operation, channel, param1, param2;

	snd_seq_ev_clear(&ev);
	snd_seq_ev_set_direct(&ev);
	snd_seq_ev_set_source(&ev, port_out_id);
	snd_seq_ev_set_subs(&ev);

	/*
	   MIDI COMMANDS
	   -------------------------------------------------------------------
	   name                 status      param 1          param 2
	   -------------------------------------------------------------------
	   note off             0x80+C       key #            velocity
	   note on              0x90+C       key #            velocity
	   poly key pressure    0xA0+C       key #            pressure value
	   control change       0xB0+C       control #        control value
	   program change       0xC0+C       program #        --
	   mono key pressure    0xD0+C       pressure value   --
	   pitch bend           0xE0+C       range (LSB)      range (MSB)
	   system               0xF0+C       manufacturer     model
	   -------------------------------------------------------------------
	   C is the channel number, from 0 to 15;
	   -------------------------------------------------------------------
	   source: http://ftp.ec.vanderbilt.edu/computermusic/musc216site/MIDI.Commands.html
	
	   In this program the pitch bend range will be transmitter as 
	   one single 8-bit number. So the end result is that MIDI commands 
	   will be transmitted as 3 bytes, starting with the operation byte:
	
	   buf[0] --> operation/channel
	   buf[1] --> param1
	   buf[2] --> param2        (param2 not transmitted on program change or key press)
	 */
	operation = buf[0] & 0xF0;
	channel = buf[0] & 0x0F;
	param1 = buf[1];
	param2 = buf[2];

	switch (operation) {
		case MIDI_NOTE_OFF:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Note off           %03u %03u %03u\n", operation, channel, param1, param2);
			snd_seq_ev_set_noteoff(&ev, channel, param1, param2);
			break;
			
		case MIDI_NOTE_ON:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Note on            %03u %03u %03u\n", operation, channel, param1, param2);
			snd_seq_ev_set_noteon(&ev, channel, param1, param2);
			break;
			
		case MIDI_POLY_KEY_PRESSURE:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Pressure change    %03u %03u %03u\n", operation, channel, param1, param2);
			snd_seq_ev_set_keypress(&ev, channel, param1, param2);
			break;

		case MIDI_CONTROL_CHANGE:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Controller change  %03u %03u %03u\n", operation, channel, param1, param2);
			snd_seq_ev_set_controller(&ev, channel, param1, param2);
			break;

		case MIDI_PROGRAM_CHANGE:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Program change     %03u %03u\n", operation, channel, param1);
			snd_seq_ev_set_pgmchange(&ev, channel, param1);
			break;

		case MIDI_MONO_KEY_PRESSURE:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Channel change     %03u %03u\n", operation, channel, param1);
			snd_seq_ev_set_chanpress(&ev, channel, param1);
			break;

		case MIDI_PITCH_BEND:
			param1 = (param1 & 0x7F) + ((param2 & 0x7F) << 7);
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Pitch bend         %03u %05i\n", operation, channel, param1);
			snd_seq_ev_set_pitchbend(&ev, channel, param1 - 8192); /* in alsa MIDI we want signed int */
			break;

		/* Not implementing system commands (0xF0) */
		default:
			if (!arguments.silent) 
				printf("0x%x Unknown MIDI cmd   %03u %03u %03u\n", operation, channel, param1, param2);
			break;
	}

	snd_seq_event_output_direct(seq, &ev);
	snd_seq_drain_output(seq);
}

static inline bool is_status(char buf)
{
	return buf >> 7 != 0;
}

static void read_midi_from_serial_port(snd_seq_t* seq, int port_out_id, int serial)
{
	char buf[3];
	
	if (arguments.printonly)
		printf("Super debug mode: Only printing the signal to screen. Nothing else.\n");
	else /* Lets first fast forward to first status byte... */
		do
			read(serial, &buf[0], 1);
		while (!is_status(buf[0]));

	while (run) {
		int i;

		/* super-debug mode: only print to screen whatever comes through the serial port */
		if (arguments.printonly) {
			read(serial, &buf[0], 1);
			printf("%x\t", (int)buf[0] & 0xFF);
			fflush(stdout);
			continue;
		}

		/* let's start at the beginning of a midi command */
		for (i = 1; ; ) {
			read(serial, &buf[i], 1);

			if (is_status(buf[i])) {
				/* Status byte received and will always be first bit! */
				buf[0] = buf[i];
				i = 1;
			} else {
				/* Data byte received */
				if (i == 2) {
					/* It was 2nd data byte so we have a MIDI event process! */
					break;
				} else {
					/* Lets figure out are we done or should we read one more byte */
					if ((buf[0] & 0xF0) == 0xC0 || (buf[0] & 0xF0) == 0xD0)
						break;
					else
						i = 2;
				}
			}
		}

		/* print comment message (the ones that start with 0xFF 0x00 0x00 */
		if (buf[0] == (char)0xFF && buf[1] == (char)0x00 && buf[2] == (char)0x00) {
			char msg[MAX_MSG_SIZE];
			int msglen;

			read(serial, buf, 1);
			msglen = buf[0];
			if (msglen > MAX_MSG_SIZE - 1)
				msglen = MAX_MSG_SIZE - 1;

			read(serial, msg, msglen);

			if (arguments.silent)
				continue;

			/* make sure the string ends with a null character */
			msg[msglen] = 0;

			puts("0xFF Non-MIDI message: ");
			puts(msg);
			putchar('\n');
			fflush(stdout);
		} else {
			parse_midi_command(seq, port_out_id, buf);
		}
	}
}

int main(int argc, char** argv)
{
	struct termios oldtio;
	static struct termios newtio;
	snd_seq_t *seq;
	int port_out_id;
	int serial;

	argp_parse(&argp, argc, argv, 0, 0, &arguments);

	/* Open MIDI output port */
	port_out_id = open_seq(&seq);
	if (port_out_id < 0)
		exit(-1);

	/* 
	 * Open modem device for reading and not as controlling tty
	 * because we don't want to get killed if linenoise sends CTRL-C
	 */
	serial = open(arguments.serialdevice, O_RDWR | O_NOCTTY ); 
	if (serial < 0) {
		perror(arguments.serialdevice); 
		exit(-1); 
	}

	/* save current serial port settings */
	if (tcgetattr(serial, &oldtio)) {
		perror("tcgetattr");
		exit(-1);
	}

	build_midi_termios(&newtio);

	/* now clean the modem line and activate the settings for the port */
	if (tcflush(serial, TCIFLUSH)) {
		perror("tcflush");
		exit(-1);
	}
	if (tcsetattr(serial, TCSANOW, &newtio)) {
		perror("tcsetattr");
		exit(-1);
	}

#if 0
	{
		struct serial_struct ser_info;
		/* Linux-specific: enable low latency mode (FTDI "nagling off") */
		ioctl(serial, TIOCGSERIAL, &ser_info);
		ser_info.flags |= ASYNC_LOW_LATENCY;
		ioctl(serial, TIOCSSERIAL, &ser_info);
	}
#endif

	if (signal(SIGINT, exit_cli) == SIG_ERR) {
		perror("signal");
		exit(-1);
	}
	if (signal(SIGTERM, exit_cli) == SIG_ERR) {
		perror("signal");
		exit(-1);
	}

	read_midi_from_serial_port(seq, port_out_id, serial);

	/* restore the old port settings */
	if (tcsetattr(serial, TCSANOW, &oldtio)) {
		perror("tcsetattr");
		exit(-1);
	}

	printf("\ndone!\n");
}
