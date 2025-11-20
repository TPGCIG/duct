/*
 * Copyright 2025, The University of Queensland
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Simple test tool for duct(4)
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <err.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <errno.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <dev/ductvar.h>

static __dead void
usage(void)
{
	extern char *__progname;
	fprintf(stderr, "usage: %s [-i] [-d DEVICE]\n", __progname);
	fprintf(stderr, "       - shows duct device version and address\n");
	fprintf(stderr, "       %s -s ADDRHEX [-d DEVICE]\n", __progname);
	fprintf(stderr, "       - sends a packet from stdin to a "
	    "given address\n");
	fprintf(stderr, "       %s -r [-d DEVICE]\n", __progname);
	fprintf(stderr, "       - receives a packet and prints it to "
	    "stdout\n");
	exit(1);
}

enum mode {
	MODE_UNSET = 0,
	MODE_INFO,
	MODE_SEND,
	MODE_RECV
};
static enum mode mode = MODE_UNSET;
static char devpath[PATH_MAX];
static struct duct_packet_hdr hdr;

#define	DUCT_DEVFMT	"/dev/duct%d"
#define	DUCT_MAXDEV	8

static void	do_send(int);
static void	do_recv(int);

int
main(int argc, char *argv[])
{
	int ch;
	const char *errstr;
	char *ep;
	int fd;
	uint i;
	struct duct_info_arg info;

	bzero(&hdr, sizeof(hdr));
	snprintf(devpath, sizeof(devpath), DUCT_DEVFMT, 0);

	while ((ch = getopt(argc, argv, "id:s:r")) != -1) {
		switch (ch) {
		case 'i':
			if (mode != MODE_UNSET) {
				fprintf(stderr, "error: -i, -s and -r are "
				    "mutually exclusive\n");
				usage();
			}
			mode = MODE_INFO;
			break;
		case 'r':
			if (mode != MODE_UNSET) {
				fprintf(stderr, "error: -i, -s and -r are "
				    "mutually exclusive\n");
				usage();
			}
			mode = MODE_RECV;
			break;
		case 's':
			if (mode != MODE_UNSET) {
				fprintf(stderr, "error: -i, -s and -r are "
				    "mutually exclusive\n");
				usage();
			}
			mode = MODE_SEND;
			errno = 0;
			hdr.dpkt_destination = strtoul(optarg, &ep, 16);
			if (*ep != '\0' || errno != 0) {
				fprintf(stderr, "error: -s addr is invalid\n");
				usage();
			}
			break;
		case 'd':
			if (optarg[0] == '/') {
				strlcpy(devpath, optarg, sizeof(devpath));
				break;
			}
			if (isdigit(optarg[0])) {
				i = strtonum(optarg, 0, DUCT_MAXDEV, &errstr);
				if (errstr != NULL) {
					warnx("-d arg starts with a number "
					    "but is %s", errstr);
					usage();
				}
				snprintf(devpath, sizeof(devpath), DUCT_DEVFMT,
				    i);
				break;
			}
			strlcpy(devpath, "/dev/", sizeof(devpath));
			strlcat(devpath, optarg, sizeof(devpath));
			break;
		default:
			usage();
		}
	}

	fd = open(devpath, O_RDWR);
	if (fd < 0)
		err(1, "open(%s)", devpath);

	bzero(&info, sizeof(info));
	if (ioctl(fd, DUCTIOC_GET_INFO, &info))
		err(1, "ioctl(%s, DUCTIOC_GET_INFO)", devpath);

	switch (mode) {
	case MODE_UNSET:
	case MODE_INFO:
		fprintf(stderr, "%s: duct v%u.%u, hwaddr: %08x\n", devpath,
		    info.duct_major, info.duct_minor, info.duct_hwaddr);
		break;

	case MODE_SEND:
		do_send(fd);
		break;

	case MODE_RECV:
		do_recv(fd);
		break;
	}

	close(fd);
	return (0);
}

static void
do_send(int fd)
{
	struct iovec iov[2];
	ssize_t done;
	char *data;
	size_t pos, len;

	len = 16384;
	data = malloc(len);
	if (data == NULL)
		err(1, "malloc");

	pos = 0;
	while (pos < len) {
		done = read(STDIN_FILENO, &data[pos], len - pos);
		if (done < 0)
			err(1, "read(stdin)");
		if (done == 0)
			break;
		pos += done;
	}

	hdr.dpkt_length = pos;

	iov[0].iov_base = &hdr;
	iov[0].iov_len = sizeof(hdr);

	iov[1].iov_base = data;
	iov[1].iov_len = pos;

	done = writev(fd, iov, 2);
	if (done < 0)
		err(1, "writev");

	free(data);
}

static void
do_recv(int fd)
{
	struct iovec iov[2];
	ssize_t done;
	char *data;
	size_t pos, len;

	len = 16384;
	data = malloc(len);
	if (data == NULL)
		err(1, "malloc");

	iov[0].iov_base = &hdr;
	iov[0].iov_len = sizeof(hdr);
	iov[1].iov_base = data;
	iov[1].iov_len = len;

	done = readv(fd, iov, 2);
	if (done < 0)
		err(1, "readv");
	if ((size_t)done < sizeof(hdr))
		errx(1, "short packet");

	done -= sizeof(hdr);
	fprintf(stderr, "rx: %08x => %08x (%zd bytes)\n", hdr.dpkt_source,
	    hdr.dpkt_destination, done);

	len = done;
	pos = 0;
	while (pos < len) {
		done = write(STDOUT_FILENO, &data[pos], len - pos);
		if (done < 0)
			err(1, "write(stdout)");
		if (done == 0)
			break;
		pos += done;
	}

	free(data);
}
