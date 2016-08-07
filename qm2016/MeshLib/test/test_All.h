#include <gtest/gtest.h>

void test_all(int &argc, char ** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	RUN_ALL_TESTS();
}