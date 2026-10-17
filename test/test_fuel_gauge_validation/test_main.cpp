#include <unity.h>
#include "fuel_gauge_validate.h"

void test_soc_rejects_invalid_reads() {
  TEST_ASSERT_TRUE(fuelGaugeSocValid(0));
  TEST_ASSERT_TRUE(fuelGaugeSocValid(55));
  TEST_ASSERT_TRUE(fuelGaugeSocValid(100));
  TEST_ASSERT_FALSE(fuelGaugeSocValid(-1));
  TEST_ASSERT_FALSE(fuelGaugeSocValid(101));
  TEST_ASSERT_FALSE(fuelGaugeSocValid(255));    // 0xFFFF short read
  TEST_ASSERT_FALSE(fuelGaugeSocValid(65535));
}

void test_voltage_rejects_invalid_reads() {
  TEST_ASSERT_TRUE(fuelGaugeVoltageValid(3700));
  TEST_ASSERT_TRUE(fuelGaugeVoltageValid(2000));
  TEST_ASSERT_TRUE(fuelGaugeVoltageValid(5000));
  TEST_ASSERT_FALSE(fuelGaugeVoltageValid(65535));  // 65.535 V failure mode
  TEST_ASSERT_FALSE(fuelGaugeVoltageValid(1999));
  TEST_ASSERT_FALSE(fuelGaugeVoltageValid(0));
}

void test_current_power_reject_garbage() {
  TEST_ASSERT_TRUE(fuelGaugeCurrentValid(0));
  TEST_ASSERT_TRUE(fuelGaugeCurrentValid(-1200));
  TEST_ASSERT_FALSE(fuelGaugeCurrentValid(32767));
  TEST_ASSERT_FALSE(fuelGaugeCurrentValid(-32768));
  TEST_ASSERT_TRUE(fuelGaugePowerValid(-2500));
  TEST_ASSERT_FALSE(fuelGaugePowerValid(32767));
}

void test_capacity_soh_temp_reject_invalid() {
  TEST_ASSERT_TRUE(fuelGaugeCapacityValid(700));
  TEST_ASSERT_FALSE(fuelGaugeCapacityValid(65535));
  TEST_ASSERT_TRUE(fuelGaugeFullCapacityValid(634));
  TEST_ASSERT_FALSE(fuelGaugeFullCapacityValid(65535));
  TEST_ASSERT_TRUE(fuelGaugeSohValid(91));
  TEST_ASSERT_FALSE(fuelGaugeSohValid(0));
  TEST_ASSERT_FALSE(fuelGaugeSohValid(255));
  TEST_ASSERT_TRUE(fuelGaugeTempValid(2981));
  TEST_ASSERT_FALSE(fuelGaugeTempValid(0x7FFF));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_soc_rejects_invalid_reads);
  RUN_TEST(test_voltage_rejects_invalid_reads);
  RUN_TEST(test_current_power_reject_garbage);
  RUN_TEST(test_capacity_soh_temp_reject_invalid);
  return UNITY_END();
}
