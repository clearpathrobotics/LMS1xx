/*
 * lms_buffer.h
 *
 *  Author: Mike Purvis <mpurvis@clearpathrobotics.com>
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "LMS1xx/lms_buffer.h"
#include "gtest/gtest.h"

#include <errno.h>
#include <unistd.h>


class BufferTest : public :: testing :: Test
{
  virtual void SetUp()
  {
    ASSERT_NE(-1, pipe(fds_));
    ASSERT_NE(0, fds_[0]);
    ASSERT_NE(0, fds_[1]);
  }

  virtual void TearDown()
  {
    close(fds_[0]);
    close(fds_[1]);
  }

protected:
  LMSBuffer buf_;
  int fds_[2];
};


TEST_F(BufferTest, single)
{
  ASSERT_NE(-1, write(fds_[1], "\x02ghij\x03", 6)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);

  // Final char should have been transformed to null terminator.
  ASSERT_STREQ("\x02ghij", buf_.getNextBuffer());
}

TEST_F(BufferTest, split)
{
  ASSERT_NE(-1, write(fds_[1], "\x02ghij", 5)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);
  EXPECT_EQ(NULL, buf_.getNextBuffer());

  ASSERT_NE(-1, write(fds_[1], "abcde", 5)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);
  EXPECT_EQ(NULL, buf_.getNextBuffer());

  ASSERT_NE(-1, write(fds_[1], "fghijk\x03", 7)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);
  EXPECT_STREQ("\x02ghijabcdefghijk", buf_.getNextBuffer());
}

TEST_F(BufferTest, extra_bytes)
{
  ASSERT_NE(-1, write(fds_[1], "abc\x02ghijk\x03stuv", 14)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);
  EXPECT_STREQ("\x02ghijk", buf_.getNextBuffer());
  buf_.popLastBuffer();

  ASSERT_NE(-1, write(fds_[1], "abc\x02ghjk\x03stuv", 14)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);
  EXPECT_STREQ("\x02ghjk", buf_.getNextBuffer());
}

TEST_F(BufferTest, multiple)
{
  ASSERT_NE(-1, write(fds_[1], "\x02gh\x03\x02jk\x03xyz\x02ty\x03", 15)) << "Error code: " << errno;
  buf_.readFrom(fds_[0]);

  EXPECT_STREQ("\x02gh", buf_.getNextBuffer());
  buf_.popLastBuffer();
  EXPECT_STREQ("\x02jk", buf_.getNextBuffer());
  buf_.popLastBuffer();
  EXPECT_STREQ("\x02ty", buf_.getNextBuffer());
  buf_.popLastBuffer();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
