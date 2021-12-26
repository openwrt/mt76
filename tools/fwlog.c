// SPDX-License-Identifier: ISC
/* Copyright (C) 2022 Felix Fietkau <nbd@nbd.name> */
#define _GNU_SOURCE

#include <sys/types.h>
#include <sys/uio.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>
#include <signal.h>
#include "mt76-test.h"

bool done = false;

static const char *debugfs_path(const char *phyname, const char *file)
{
	static char path[256];

	snprintf(path, sizeof(path), "/sys/kernel/debug/ieee80211/%s/mt76/%s", phyname, file);

	return path;
}

static int mt76_set_fwlog_en(const char *phyname, bool en)
{
	FILE *f = fopen(debugfs_path(phyname, "fw_debug_bin"), "w");

	if (!f) {
		fprintf(stderr, "Could not open fw_debug_bin file\n");
		return 1;
	}

	fprintf(f, "7");
	fclose(f);

	return 0;
}

int read_retry(int fd, void *buf, int len)
{
	int out_len = 0;
	int r;

	while (len > 0) {
		if (done)
			return -1;

		r = read(fd, buf, len);
		if (r < 0) {
			if (errno == EINTR || errno == EAGAIN)
				continue;

			return -1;
		}

		if (!r)
			return 0;

		out_len += r;
		len -= r;
		buf += r;
	}

	return out_len;
}

static void handle_signal(int sig)
{
	done = true;
}

int mt76_fwlog(const char *phyname, int argc, char **argv)
{
	struct sockaddr_in local = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = INADDR_ANY,
	};
	struct sockaddr_in remote = {
		.sin_family = AF_INET,
		.sin_port = htons(55688),
	};
	char buf[1504];
	int ret = 0;
	int yes = 1;
	int s, fd;

	if (argc < 1) {
		fprintf(stderr, "need destination address\n");
		return 1;
	}

	if (!inet_aton(argv[0], &remote.sin_addr)) {
		fprintf(stderr, "invalid destination address\n");
		return 1;
	}

	s = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s < 0) {
		perror("socket");
		return 1;
	}

	setsockopt(s, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
	if (bind(s, (struct sockaddr *)&local, sizeof(local)) < 0) {
		perror("bind");
		return 1;
	}

	if (mt76_set_fwlog_en(phyname, true))
		return 1;

	fd = open(debugfs_path(phyname, "fwlog_data"), O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "Could not open fwlog_data file: %s\n", strerror(errno));
		ret = 1;
		goto out;
	}

	signal(SIGTERM, handle_signal);
	signal(SIGINT, handle_signal);
	signal(SIGQUIT, handle_signal);

	while (1) {
		struct pollfd pfd = {
			.fd = fd,
			.events = POLLIN | POLLHUP | POLLERR,
		};
		uint32_t len;
		int r;

		if (done)
			break;

		poll(&pfd, 1, -1);

		r = read_retry(fd, &len, sizeof(len));
		if (r < 0)
			break;

		if (!r)
			continue;

		if (len > sizeof(buf)) {
			fprintf(stderr, "Length error: %d > %d\n", len, (int)sizeof(buf));
			ret = 1;
			break;
		}

		if (done)
			break;

		r = read_retry(fd, buf, len);
		if (done)
			break;

		if (r != len) {
			fprintf(stderr, "Short read: %d < %d\n", r, len);
			ret = 1;
			break;
		}

		/* send buf */
		sendto(s, buf, len, 0, (struct sockaddr *)&remote, sizeof(remote));
	}

	close(fd);

out:
	mt76_set_fwlog_en(phyname, false);

	return ret;
}
