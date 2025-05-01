#pragma once
#include <Arduino.h>

struct ConfigData {
  int heartbeatInterval;
  float targetPH;
  float targetEC;
  int lightOpenSec;
  int lightCloseSec;
  int pumpOpenSec;
  int pumpCloseSec;
  uint64_t timestamp;
};

struct HeartbeatData {
  float phLevel;
  float ecLevel;
  int waterLevelPercent;
  bool lightsOpen;
  uint64_t timestamp;
};

struct NotificationData {
  int type;
  uint64_t timestamp;
};

ConfigData parseC0Packet(const String& packet) {
  ConfigData data;
  String parts[10];
  int index = 0;
  int last = 0;

  for (int i = 0; i < packet.length(); i++) {
    if (packet[i] == ',' || i == packet.length() - 1) {
      int end = (packet[i] == ',') ? i : i + 1;
      parts[index++] = packet.substring(last, end);
      last = i + 1;
    }
  }

  if (parts[0] != "C0") return data;

  data.heartbeatInterval = parts[1].toInt();
  data.targetPH = parts[2].toFloat() / 100.0;
  data.targetEC = parts[3].toFloat() / 100.0;
  data.lightOpenSec = parts[4].toInt();
  data.lightCloseSec = parts[5].toInt();
  data.pumpOpenSec = parts[6].toInt();
  data.pumpCloseSec = parts[7].toInt();
  data.timestamp = parts[8].toInt();

  return data;
}

HeartbeatData parseH0Packet(const String& packet) {
  HeartbeatData data;
  String parts[10];
  int index = 0;
  int last = 0;

  for (int i = 0; i < packet.length(); i++) {
    if (packet[i] == ',' || i == packet.length() - 1) {
      int end = (packet[i] == ',') ? i : i + 1;
      parts[index++] = packet.substring(last, end);
      last = i + 1;
    }
  }

  if (parts[0] != "H0") return data;

  data.phLevel = parts[1].toFloat() / 100.0;
  data.ecLevel = parts[2].toFloat() / 100.0;
  data.waterLevelPercent = parts[3].toInt();
  data.lightsOpen = (parts[4].toInt() == 1);
  data.timestamp = parts[5].toInt();

  return data;
}

NotificationData parseN0Packet(const String& packet) {
  NotificationData data;
  String parts[5];
  int index = 0;
  int last = 0;

  for (int i = 0; i < packet.length(); i++) {
    if (packet[i] == ',' || i == packet.length() - 1) {
      int end = (packet[i] == ',') ? i : i + 1;
      parts[index++] = packet.substring(last, end);
      last = i + 1;
    }
  }

  if (parts[0] != "N0") return data;

  data.type = parts[1].toInt();
  data.timestamp = parts[2].toInt();

  return data;
}
