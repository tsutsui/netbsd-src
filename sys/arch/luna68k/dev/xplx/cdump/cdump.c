#include <err.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int
main(int ac, char *av[])
{
	uint8_t buf[1024];

	if (ac < 2) {
		errx(EXIT_FAILURE, "too few argument");
	}

	int fd = open(av[1], O_RDONLY);
	if (fd == -1) {
		err(EXIT_FAILURE, "open");
	}

	int len = 0;
	do {
		int n = read(fd, buf, sizeof(buf));
		if (n == 0) break;
		if (n == -1) {
			err(EXIT_FAILURE, "read");
		}
		for (int i = 0; i < n; i++) {
			printf("0x%02x,%c", (uint8_t)buf[i],
				(len % 8 == 7) ? '\n' : ' ');
			len++;
		}
	} while (1);

	return 0;
}
