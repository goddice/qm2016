#include <gtest/gtest.h>
#include "../Point.h"

TEST(POINT_CLASS_TEST, ADDITION_TEST) {
	EXPECT_EQ(Point(2, 2, 2), Point(1, 1, 1) + Point(1, 1, 1));
	EXPECT_EQ(Point(0, 0, 0), Point(1, 1, 1) + Point(-1, -1, -1));
	EXPECT_EQ(Point(10001, 10001, -9999), Point(10000, 10000, -10000) + Point(1, 1, 1));
}

TEST(POINT_CLASS_TEST, DOC_PRODUCT_TEST) {
	EXPECT_EQ(0, Point(1, 0, 0) * Point(0, 1, 0));
}