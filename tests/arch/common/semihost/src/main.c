/*
 * Copyright (c) 2022, Commonwealth Scientific and Industrial Research
 * Organisation (CSIRO) ABN 41 687 119 230.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/arch/common/semihost.h>

/**
 * @brief Verify that the semihosting file functions open, write, read, seek and
 * close a file on the host.
 *
 * @details
 * The test uses the file ./test.bin on the host. It writes 32 bytes, reads them
 * back and reads past the end of the file. It also verifies that an open in
 * write mode erases the file.
 *
 * Test steps:
 * - Open the file with SEMIHOST_OPEN_WB.
 * - Write the 16-byte buffer two times. After each write, get the file length.
 * - Read from the file while it is open in write mode.
 * - Close the file. Then open it again with SEMIHOST_OPEN_RB.
 * - Read 16 bytes two times. Then read 16 bytes past the end of the file.
 * - Seek to the offset 1. Then read 15 bytes.
 * - Close the file. Then open it with SEMIHOST_OPEN_WB again and close it.
 *
 * Expected result:
 * - Each semihost_open() call returns a handle that is more than 0.
 * - After an open in write mode, semihost_flen() returns 0.
 * - semihost_write() returns 0. After each write, semihost_flen() returns the
 *   total number of bytes written.
 * - The read in write mode and the read past the end of the file return -EIO.
 * - After the open in read mode, semihost_flen() returns 32.
 * - The other reads return the requested number of bytes and the written data.
 * - semihost_seek() and semihost_close() return 0.
 *
 * @see semihost_open(), semihost_flen(), semihost_write(), semihost_read(),
 * semihost_seek(), semihost_close()
 */
ZTEST(semihost, test_file_ops)
{
	const char *test_file = "./test.bin";
	uint8_t w_buffer[16] = { 1, 2, 3, 4, 5 };
	uint8_t r_buffer[16];
	long read, fd;

	/* Open in write mode */
	fd = semihost_open(test_file, SEMIHOST_OPEN_WB);
	zassert_true(fd > 0, "Bad handle (%ld)", fd);
	zassert_equal(semihost_flen(fd), 0, "File not empty");

	/* Write some data */
	zassert_equal(semihost_write(fd, w_buffer, sizeof(w_buffer)), 0, "Write failed");
	zassert_equal(semihost_flen(fd), sizeof(w_buffer), "Size not updated");
	zassert_equal(semihost_write(fd, w_buffer, sizeof(w_buffer)), 0, "Write failed");
	zassert_equal(semihost_flen(fd), 2 * sizeof(w_buffer), "Size not updated");

	/* Reading should fail in this mode */
	read = semihost_read(fd, r_buffer, sizeof(r_buffer));
	zassert_equal(read, -EIO, "Read from write-only file");

	/* Close the file */
	zassert_equal(semihost_close(fd), 0, "Close failed");

	/* Open the same file again for reading */
	fd = semihost_open(test_file, SEMIHOST_OPEN_RB);
	zassert_true(fd > 0, "Bad handle (%ld)", fd);
	zassert_equal(semihost_flen(fd), 2 * sizeof(w_buffer), "Data not preserved");

	/* Check reading data */
	read = semihost_read(fd, r_buffer, sizeof(r_buffer));
	zassert_equal(read, sizeof(r_buffer), "Read failed %ld", read);
	zassert_mem_equal(r_buffer, w_buffer, sizeof(r_buffer), "Data not read");
	read = semihost_read(fd, r_buffer, sizeof(r_buffer));
	zassert_equal(read, sizeof(r_buffer), "Read failed");
	zassert_mem_equal(r_buffer, w_buffer, sizeof(r_buffer), "Data not read");

	/* Read past end of file */
	read = semihost_read(fd, r_buffer, sizeof(r_buffer));
	zassert_equal(read, -EIO, "Read past end of file");

	/* Seek to file offset */
	zassert_equal(semihost_seek(fd, 1), 0, "Seek failed");

	/* Read from offset */
	read = semihost_read(fd, r_buffer, sizeof(r_buffer) - 1);
	zassert_equal(read, sizeof(r_buffer) - 1, "Read failed");
	zassert_mem_equal(r_buffer, w_buffer + 1, sizeof(r_buffer) - 1, "Data not read");

	/* Close the file */
	zassert_equal(semihost_close(fd), 0, "Close failed");

	/* Opening again in write mode should erase the file */
	fd = semihost_open(test_file, SEMIHOST_OPEN_WB);
	zassert_true(fd > 0, "Bad handle (%ld)", fd);
	zassert_equal(semihost_flen(fd), 0, "File not empty");
	zassert_equal(semihost_close(fd), 0, "Close failed");
}

ZTEST_SUITE(semihost, NULL, NULL, NULL, NULL, NULL);
