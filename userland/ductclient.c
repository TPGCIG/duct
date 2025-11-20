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
	fprintf(stderr, "usage: %s -e [-d DEVICE]\n", __progname);
	fprintf(stderr, "       - line-based client for the echo service\n");
	fprintf(stderr, "       %s -q [-d DEVICE]\n", __progname);
	fprintf(stderr, "       - client for the quote-of-the-day service\n");
	exit(1);
}

enum mode {
	MODE_UNSET = 0,
	MODE_ECHO,
	MODE_QOTD
};
static enum mode mode = MODE_UNSET;
static char devpath[PATH_MAX];

#define	DUCT_DEVFMT	"/dev/duct%d"
#define	DUCT_MAXDEV	8

static void	do_echo(int);
static void	do_qotd(int);

int
main(int argc, char *argv[])
{
	int ch;
	const char *errstr;
	int fd;
	uint i;

	snprintf(devpath, sizeof(devpath), DUCT_DEVFMT, 0);

	while ((ch = getopt(argc, argv, "d:eq")) != -1) {
		switch (ch) {
		case 'e':
			if (mode != MODE_UNSET) {
				fprintf(stderr, "error: -e and -q are "
				    "mutually exclusive\n");
				usage();
			}
			mode = MODE_ECHO;
			break;
		case 'q':
			if (mode != MODE_UNSET) {
				fprintf(stderr, "error: -e and -q are "
				    "mutually exclusive\n");
				usage();
			}
			mode = MODE_QOTD;
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

	if (mode == MODE_UNSET) {
		warnx("need either -e or -q option");
		usage();
	}

	fd = open(devpath, O_RDWR);
	if (fd < 0)
		err(1, "open(%s)", devpath);

	if (mode == MODE_ECHO) {
		do_echo(fd);
	} else if (mode == MODE_QOTD) {
		do_qotd(fd);
	}

	close(fd);
	return (0);
}

static void
do_echo(int fd)
{
	struct iovec iov[2];
	ssize_t done;
	char *data;
	size_t len;
	struct duct_packet_hdr hdr;

	len = 16384;
	data = malloc(len);
	if (data == NULL)
		err(1, "malloc");

	while (1) {
		if (fgets(data, len, stdin) == NULL)
			break;

		bzero(&hdr, sizeof(hdr));
		hdr.dpkt_destination = 0xEC60;
		hdr.dpkt_length = strlen(data);

		iov[0].iov_base = &hdr;
		iov[0].iov_len = sizeof(hdr);

		iov[1].iov_base = data;
		iov[1].iov_len = hdr.dpkt_length;

		done = writev(fd, iov, 2);
		if (done < 0)
			err(1, "writev");

		while (hdr.dpkt_source != 0xEC60) {
			done = readv(fd, iov, 2);
			if (done < 0)
				err(1, "readv");
			if ((size_t)done < sizeof(hdr))
				errx(1, "short packet");
		}
		done -= sizeof(hdr);
		data[done] = '\0';

		printf("%s", data);
	}

	free(data);
}

static void
do_qotd(int fd)
{
	struct iovec iov[2];
	ssize_t done;
	char *data;
	size_t len;
	struct duct_packet_hdr hdr;

	len = 16384;
	data = malloc(len);
	if (data == NULL)
		err(1, "malloc");

	bzero(&hdr, sizeof(hdr));
	hdr.dpkt_destination = 0xBEEF;
	hdr.dpkt_length = 0;

	iov[0].iov_base = &hdr;
	iov[0].iov_len = sizeof(hdr);

	done = writev(fd, iov, 1);
	if (done < 0)
		err(1, "writev");

	iov[1].iov_base = data;
	iov[1].iov_len = len;

	while (hdr.dpkt_source != 0xBEEF) {
		done = readv(fd, iov, 2);
		if (done < 0)
			err(1, "readv");
		if ((size_t)done < sizeof(hdr))
			errx(1, "short packet");
	}
	done -= sizeof(hdr);
	data[done] = '\0';

	printf("%s\n", data);

	free(data);
}
