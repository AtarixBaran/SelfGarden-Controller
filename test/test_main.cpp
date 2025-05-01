#include <Arduino.h>
#include <unity.h>
#include "iot_parser.h"

void test_parse_C0_valid() {
  String input = "C0,120,126700,617,120,60,120,120,200";
  ConfigData d = parseC0Packet(input);
  TEST_ASSERT_EQUAL(120, d.heartbeatInterval);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1267.00, d.targetPH);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 6.17, d.targetEC);
  TEST_ASSERT_EQUAL(120, d.lightOpenSec);
  TEST_ASSERT_EQUAL(60, d.lightCloseSec);
  TEST_ASSERT_EQUAL(120, d.pumpOpenSec);
  TEST_ASSERT_EQUAL(120, d.pumpCloseSec);
  TEST_ASSERT_EQUAL_UINT64(200, d.timestamp);
}

void test_parse_H0_valid() {
  String input = "H0,700,800,90,1,200";
  HeartbeatData d = parseH0Packet(input);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 7.00, d.phLevel);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 8.00, d.ecLevel);
  TEST_ASSERT_EQUAL(90, d.waterLevelPercent);
  TEST_ASSERT_TRUE(d.lightsOpen);
  TEST_ASSERT_EQUAL_UINT64(200, d.timestamp);
}

void test_parse_N0_valid() {
  String input = "N0,4,200";
  NotificationData d = parseN0Packet(input);
  TEST_ASSERT_EQUAL(4, d.type);
  TEST_ASSERT_EQUAL_UINT64(200, d.timestamp);
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_parse_C0_valid);
  RUN_TEST(test_parse_H0_valid);
  RUN_TEST(test_parse_N0_valid);
  UNITY_END();
}

void loop() {
  // boş
}