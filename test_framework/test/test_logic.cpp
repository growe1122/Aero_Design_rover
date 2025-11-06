#include <unity.h>
#include "../src/rover_logic.h"
#include "../src/rover_logic.cpp" 

void setUp(void) {}
void tearDown(void) {}

void test_pulse() {
    TEST_ASSERT_EQUAL_INT(-255, pulseToSpeed(1000));
    TEST_ASSERT_EQUAL_INT(0,    pulseToSpeed(1500));
    TEST_ASSERT_EQUAL_INT(255,  pulseToSpeed(2000));
}

void test_mix() {
    int L, R;
    computeDriveSpeeds(2000, 1500, L, R);
    TEST_ASSERT(L >= 0);
    TEST_ASSERT(R >= 0);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pulse);
    RUN_TEST(test_mix);
    return UNITY_END();
}
