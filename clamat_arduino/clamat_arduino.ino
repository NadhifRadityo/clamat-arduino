#include <Thread.h>
#include <ThreadController.h>
#include <AsyncTimer.h>
#include <CRC.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// byte: 1 byte
// short: 2 bytes
// int: 4 bytes
// long: 8 bytes
// float: 4 bytes
// double: 8 bytes
#define t_byte char
#define t_short int
#define t_int long
#define t_long long long
#define t_ubyte unsigned char
#define t_ushort unsigned int
#define t_uint unsigned long
#define t_ulong unsigned long long
#define t_float float
#define t_double double

// #define WARN(...) Serial.print(__VA_ARGS__)
// #define DEBUG(...) Serial.print(__VA_ARGS__)
#define WARN(...)
#define DEBUG(...)

#define BLUETOOTH_RECEIVE_BUFFER_SIZE 256
CRC16 commCrcCalculator;

t_ushort commCrc(const t_ubyte *buffer, t_uint length, t_ushort previous) {
  commCrcCalculator.setInitial(previous);
  commCrcCalculator.restart();
  commCrcCalculator.add(buffer, length);
  return commCrcCalculator.calc();
}

t_ubyte commReceiveBuffer[BLUETOOTH_RECEIVE_BUFFER_SIZE] = {};
t_uint commReceiveIndex = 0;
t_uint commReceiveLength = 0;
t_uint commReceiveCrcValue = 0;
t_uint commReceiveBailoutIndex = 0;
t_uint commReceiveBailoutLength = 0;
t_uint commReceiveBailoutCrcValue = 0;

void commReceive_consume() {
  t_uint original = commReceiveLength;
  while(Serial.available() && commReceiveLength < BLUETOOTH_RECEIVE_BUFFER_SIZE) {
    commReceiveBuffer[commReceiveIndex] = Serial.read();
    commReceiveIndex++;
    commReceiveLength++;
    if(commReceiveIndex >= BLUETOOTH_RECEIVE_BUFFER_SIZE) {
      commReceiveIndex = 0;
      DEBUG("OFFSET WRAP");
    }
  }
  if(original != commReceiveLength) {
    DEBUG("RECEIVED: ");
    DEBUG(commReceiveLength - original);
    DEBUG(" bytes\n");
  }
}
void commReceive_captureBailout() {
  commReceiveBailoutIndex = commReceiveIndex;
  commReceiveBailoutLength = commReceiveLength;
  commReceiveBailoutCrcValue = commReceiveCrcValue;
}
void commReceive_applyBailout() {
  commReceiveIndex = commReceiveBailoutIndex;
  commReceiveLength = commReceiveBailoutLength;
  commReceiveCrcValue = commReceiveBailoutCrcValue;
}
boolean commReceive_readBytes(t_ubyte *buffer, t_uint length) {
  if(commReceiveLength < length)
    return false;
  if(commReceiveIndex >= commReceiveLength) {
    t_uint index = commReceiveIndex - commReceiveLength;
    memcpy(&buffer[0], &commReceiveBuffer[index], length);
    commReceiveCrcValue = commCrc(&commReceiveBuffer[index], length, commReceiveCrcValue);
    commReceiveLength -= length;
    return true;
  }
  t_uint rightIndex = (BLUETOOTH_RECEIVE_BUFFER_SIZE + commReceiveIndex) - commReceiveLength;
  t_uint leftIndex = 0;
  t_uint rightLength = BLUETOOTH_RECEIVE_BUFFER_SIZE - rightIndex;
  t_uint leftLength = length - rightLength;
  if(length <= rightLength) {
    memcpy(&buffer[0], &commReceiveBuffer[rightIndex], length);
    commReceiveCrcValue = commCrc(&commReceiveBuffer[rightIndex], length, commReceiveCrcValue);
    commReceiveLength -= length;
    return true;
  }
  memcpy(&buffer[0], &commReceiveBuffer[rightIndex], rightLength);
  memcpy(&buffer[rightLength], &commReceiveBuffer[leftIndex], leftLength);
  commReceiveCrcValue = commCrc(&commReceiveBuffer[rightIndex], rightLength, commReceiveCrcValue);
  commReceiveCrcValue = commCrc(&commReceiveBuffer[leftIndex], leftLength, commReceiveCrcValue);
  commReceiveLength -= length;
  return true;
}
boolean commReceive_readUByte(t_ubyte *out) {
  if(!commReceive_readBytes(out, 1))
    return false;
  return true;
}
// bit shift array starts from highest index because arduino is little endian.
boolean commReceive_readUShort(t_ushort *out) {
  static t_ubyte buffer[2] = {};
  if(!commReceive_readBytes(buffer, 2))
    return false;
  *out = (((t_ushort) buffer[1] & 0b11111111) << 8) | ((t_ushort) buffer[0] & 0b11111111);
  return true;
}
boolean commReceive_readUInt(t_uint *out) {
  static t_ubyte buffer[4] = {};
  if(!commReceive_readBytes(buffer, 4))
    return false;
  *out = (((t_uint) buffer[3] & 0b11111111) << 24) | (((t_uint) buffer[2] & 0b11111111) << 16) | (((t_uint) buffer[1] & 0b11111111) << 8) | ((t_uint) buffer[0] & 0b11111111);
  return true;
}
boolean commReceive_readULong(t_ulong *out) {
  static t_ubyte buffer[8] = {};
  if(!commReceive_readBytes(&buffer[0], 8))
    return false;
  *out = (((t_ulong) buffer[7] & 0b11111111) << 56) | (((t_ulong) buffer[6] & 0b11111111) << 48) | (((t_ulong) buffer[5] & 0b11111111) << 40) | (((t_ulong) buffer[4] & 0b11111111) << 32) |
         (((t_ulong) buffer[3] & 0b11111111) << 24) | (((t_ulong) buffer[2] & 0b11111111) << 16) | (((t_ulong) buffer[1] & 0b11111111) << 8) | ((t_ulong) buffer[0] & 0b11111111);
  return true;
}
boolean commReceive_readByte(t_byte *out) {
  return commReceive_readUByte((t_ubyte*) out);
}
boolean commReceive_readShort(t_short *out) {
  return commReceive_readUShort((t_ushort*) out);
}
boolean commReceive_readInt(t_int *out) {
  return commReceive_readUInt((t_uint*) out);
}
boolean commReceive_readLong(t_long *out) {
  return commReceive_readULong((t_ulong*) out);
}
boolean commReceive_readFloat(t_float *out) {
  return commReceive_readUInt((t_uint*) out);
}
boolean commReceive_readDouble(t_double *out) {
  return commReceive_readULong((t_ulong*) out);
}
boolean commReceive_readCRC(t_ushort *out) {
  return commReceive_readUShort(out);
}
inline void commReceive_captureBailout() __attribute__((always_inline));
inline void commReceive_applyBailout() __attribute__((always_inline));
inline void commReceive_readUByte() __attribute__((always_inline));
inline void commReceive_readUShort() __attribute__((always_inline));
inline void commReceive_readUInt() __attribute__((always_inline));
inline void commReceive_readULong() __attribute__((always_inline));
inline void commReceive_readByte() __attribute__((always_inline));
inline void commReceive_readShort() __attribute__((always_inline));
inline void commReceive_readInt() __attribute__((always_inline));
inline void commReceive_readLong() __attribute__((always_inline));
inline void commReceive_readFloat() __attribute__((always_inline));
inline void commReceive_readDouble() __attribute__((always_inline));
inline void commReceive_readCRC() __attribute__((always_inline));

t_uint commSendCrcValue = 0;
void commSend_writeBytes(const t_ubyte *buffer, t_uint length) {
  Serial.write(buffer, length);
  commSendCrcValue = commCrc(buffer, length, commSendCrcValue);
  DEBUG("SENT: ");
  DEBUG(length);
  DEBUG(" bytes\n");
}
void commSend_writeUByte(t_ubyte in) {
  commSend_writeBytes((t_ubyte*) &in, 1);
}
void commSend_writeUShort(t_ushort in) {
  commSend_writeBytes((t_ubyte*) &in, 2);
}
void commSend_writeUInt(t_uint in) {
  commSend_writeBytes((t_ubyte*) &in, 4);
}
void commSend_writeULong(t_ulong in) {
  commSend_writeBytes((t_ubyte*) &in, 8);
}
void commSend_writeByte(t_byte in) {
  commSend_writeUByte(*((t_ubyte*) &in));
}
void commSend_writeShort(t_short in) {
  commSend_writeUShort(*((t_ushort*) &in));
}
void commSend_writeInt(t_int in) {
  commSend_writeUInt(*((t_uint*) &in));
}
void commSend_writeLong(t_long in) {
  commSend_writeULong(*((t_ulong*) &in));
}
void commSend_writeFloat(t_float in) {
  commSend_writeUInt(*((t_uint*) &in));
}
void commSend_writeDouble(t_double in) {
  commSend_writeULong(*((t_ulong*) &in));
}
void commSend_writeString(const char *in) {
  t_uint length = strlen(in);
  commSend_writeUShort(length);
  commSend_writeBytes((t_ubyte*) in, length);
}
void commSend_writeCRC() {
  commSend_writeUShort(commSendCrcValue);
  commSendCrcValue = 0;
}
inline void commSend_writeBytes() __attribute__((always_inline));
inline void commSend_writeUByte() __attribute__((always_inline));
inline void commSend_writeUShort() __attribute__((always_inline));
inline void commSend_writeUInt() __attribute__((always_inline));
inline void commSend_writeULong() __attribute__((always_inline));
inline void commSend_writeByte() __attribute__((always_inline));
inline void commSend_writeShort() __attribute__((always_inline));
inline void commSend_writeInt() __attribute__((always_inline));
inline void commSend_writeLong() __attribute__((always_inline));
inline void commSend_writeFloat() __attribute__((always_inline));
inline void commSend_writeDouble() __attribute__((always_inline));
inline void commSend_writeString() __attribute__((always_inline));
inline void commSend_writeCRC() __attribute__((always_inline));

#define COMMAND_PING 0b00000000
#define COMMAND_PONG 0b00000001
#define COMMAND_SYSTEM_STARTSEQUENCE_PRINT 0b00010000
#define COMMAND_SYSTEM_STARTSEQUENCE_PRINT_ACK 0b00010001
#define COMMAND_SYSTEM_IDENTITY_GET 0b00010010
#define COMMAND_SYSTEM_IDENTITY_GET_ACK 0b00010011
#define COMMAND_LED_SET 0b00100000
#define COMMAND_LED_SET_ACK 0b00100001
#define COMMAND_LED_GET 0b00100010
#define COMMAND_LED_GET_ACK 0b00100011
#define COMMAND_ECG_READ_SET 0b00110000
#define COMMAND_ECG_READ_SET_ACK 0b00110001
#define COMMAND_ECG_READ_GET 0b00110010
#define COMMAND_ECG_READ_GET_ACK 0b00110011
#define COMMAND_ECG_READ_VALUE 0b00110100
#define COMMAND_OXY_READ_SET 0b01000000
#define COMMAND_OXY_READ_SET_ACK 0b01000001
#define COMMAND_OXY_READ_GET 0b01000010
#define COMMAND_OXY_READ_GET_ACK 0b01000011
#define COMMAND_OXY_READ_VALUE 0b01000100
#define COMMAND_BODYTEMP_READ_SET 0b01010000
#define COMMAND_BODYTEMP_READ_SET_ACK 0b01010001
#define COMMAND_BODYTEMP_READ_GET 0b01010010
#define COMMAND_BODYTEMP_READ_GET_ACK 0b01010011
#define COMMAND_BODYTEMP_READ_VALUE 0b01010100
#define COMMAND_ENVTEMP_READ_SET 0b01100000
#define COMMAND_ENVTEMP_READ_SET_ACK 0b01100001
#define COMMAND_ENVTEMP_READ_GET 0b01100010
#define COMMAND_ENVTEMP_READ_GET_ACK 0b01100011
#define COMMAND_ENVTEMP_READ_VALUE 0b01100100
#define COMMAND_ENVHUMID_READ_SET 0b01110000
#define COMMAND_ENVHUMID_READ_SET_ACK 0b01110001
#define COMMAND_ENVHUMID_READ_GET 0b01110010
#define COMMAND_ENVHUMID_READ_GET_ACK 0b01110011
#define COMMAND_ENVHUMID_READ_VALUE 0b01110100

t_uint now = 0;
AsyncTimer microtask;

const char* const identity = "LKP9UABMMGHAW2GX@v0.1.1-patch3";
const t_ubyte startSequence[] = {
	121, 210, 15, 129, 98, 30, 111, 175,
	122, 239, 156, 151, 121, 157, 204, 157,
	106, 39, 39, 174, 198, 3, 80, 135,
	123, 94, 76, 247, 13, 38, 124, 174,
	76, 162, 111, 108, 228, 37, 43, 0,
	202, 101, 70, 189, 145, 200, 121, 156,
	207, 85, 243, 217, 126, 39, 197, 128,
	124, 100, 71, 95, 206, 182, 74, 17
};
t_uint lastActive = 0;

t_ubyte ledValue = 0;
t_ubyte ecgReadValue = 0;
t_ubyte oxyReadValue = 0;
t_ubyte bodyTempReadValue = 0;
t_ubyte envTempReadValue = 0;
t_ubyte envHumidReadValue = 0;

t_uint comm_assertCRC() {
  t_ushort receivedCrc = commReceiveCrcValue;
  t_ushort packetCrc;
  if(!commReceive_readCRC(&packetCrc))
    return 1;
  commReceiveCrcValue = 0;
  if(receivedCrc == packetCrc)
    return 0;
  WARN("CRC: got ");
  WARN(packetCrc);
  WARN(" expecting ");
  WARN(receivedCrc);
  WARN("\n");
  return 2;
}
t_uint comm_process(boolean recovery) {
  t_ubyte command;
  if(!commReceive_readUByte(&command))
    return 1;
  DEBUG("CMD: ");
  DEBUG(command);
  DEBUG("\n");
  t_ubyte user;
  if(!commReceive_readUByte(&user))
    return 1;
  DEBUG("USR: ");
  DEBUG(user);
  DEBUG("\n");
  t_uint crcStatus;
  if(command == COMMAND_PING) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_PING\n");
    commSend_writeUByte(COMMAND_PONG);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_SYSTEM_STARTSEQUENCE_PRINT) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_SYSTEM_STARTSEQUENCE_PRINT\n");
    commSend_writeUByte(COMMAND_SYSTEM_STARTSEQUENCE_PRINT_ACK);
    commSend_writeUByte(user);
    commSend_writeBytes(startSequence, sizeof(startSequence));
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_SYSTEM_IDENTITY_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_SYSTEM_IDENTITY_GET\n");
    commSend_writeUByte(COMMAND_SYSTEM_IDENTITY_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeString(identity);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_LED_SET) {
    if(!commReceive_readUByte(&ledValue))
      return 1;
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_LED_SET\n");
    commSend_writeUByte(COMMAND_LED_SET_ACK);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_LED_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_LED_GET\n");
    commSend_writeUByte(COMMAND_LED_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeUByte(ledValue);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_ECG_READ_SET) {
    if(!commReceive_readUByte(&ecgReadValue))
      return 1;
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_ECG_READ_SET\n");
    commSend_writeUByte(COMMAND_ECG_READ_SET_ACK);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_ECG_READ_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_ECG_READ_GET\n");
    commSend_writeUByte(COMMAND_ECG_READ_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeUByte(ecgReadValue);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_OXY_READ_SET) {
    if(!commReceive_readUByte(&oxyReadValue))
      return 1;
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_OXY_READ_SET\n");
    commSend_writeUByte(COMMAND_OXY_READ_SET_ACK);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_OXY_READ_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_OXY_READ_GET\n");
    commSend_writeUByte(COMMAND_OXY_READ_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeUByte(oxyReadValue);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_BODYTEMP_READ_SET) {
    if(!commReceive_readUByte(&bodyTempReadValue))
      return 1;
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_BODYTEMP_READ_SET\n");
    commSend_writeUByte(COMMAND_BODYTEMP_READ_SET_ACK);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_BODYTEMP_READ_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_BODYTEMP_READ_GET\n");
    commSend_writeUByte(COMMAND_BODYTEMP_READ_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeUByte(bodyTempReadValue);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_ENVTEMP_READ_SET) {
    if(!commReceive_readUByte(&envTempReadValue))
      return 1;
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_ENVTEMP_READ_SET\n");
    commSend_writeUByte(COMMAND_ENVTEMP_READ_SET_ACK);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_ENVTEMP_READ_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_ENVTEMP_READ_GET\n");
    commSend_writeUByte(COMMAND_ENVTEMP_READ_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeUByte(envTempReadValue);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_ENVHUMID_READ_SET) {
    if(!commReceive_readUByte(&envHumidReadValue))
      return 1;
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_ENVHUMID_READ_SET\n");
    commSend_writeUByte(COMMAND_ENVHUMID_READ_SET_ACK);
    commSend_writeUByte(user);
    commSend_writeCRC();
    return 0;
  }
  if(command == COMMAND_ENVHUMID_READ_GET) {
    if((crcStatus = comm_assertCRC()) != 0)
      return crcStatus;
    DEBUG("Command: COMMAND_ENVHUMID_READ_GET\n");
    commSend_writeUByte(COMMAND_ENVHUMID_READ_GET_ACK);
    commSend_writeUByte(user);
    commSend_writeUByte(envHumidReadValue);
    commSend_writeCRC();
    return 0;
  }
  if((crcStatus = comm_assertCRC()) != 0)
    return crcStatus;
  if(!recovery) {
    WARN("Command: UNKNOWN (");
    WARN(command);
    WARN(" ");
    WARN(user);
    WARN(")\n");
  }
  return 2;
}

#define LED_PIN 13
#define ECG_PIN A0
#define ECG_DELAY 60
#define ECG_SAMPLE 3
#define ECG_GAP 20
#define OXY_PIN A1
#define OXY_DELAY 60
#define OXY_SAMPLE 3
#define OXY_GAP 20
#define BODYTEMP_PIN A2
#define BODYTEMP_DELAY 60
#define BODYTEMP_SAMPLE 3
#define BODYTEMP_GAP 20
#define ENVTEMP_PIN 2
#define ENVTEMP_DELAY 60
#define ENVTEMP_SAMPLE 3
#define ENVTEMP_GAP 20
#define ENVHUMID_PIN 2
#define ENVHUMID_DELAY 60
#define ENVHUMID_SAMPLE 3
#define ENVHUMID_GAP 20

DHT_Unified dht(ENVTEMP_PIN, DHT22);

boolean readingEcg = false;
boolean readingOxy = false;
boolean readingBodyTemp = false;
boolean readingEnvTemp = false;
boolean readingEnvHumid = false;
t_uint lastEcgRead = 0;
t_uint lastOxyRead = 0;
t_uint lastBodyTempRead = 0;
t_uint lastEnvTempRead = 0;
t_uint lastEnvHumidRead = 0;

void read_ecg_handle(t_ubyte i, t_uint measurings = 0) {
  readingEcg = true;
  i++;
  measurings += analogRead(ECG_PIN);
  if(i < ECG_SAMPLE) {
    microtask.setTimeout([i, measurings]() {
      read_ecg_handle(i, measurings);
    }, ECG_GAP);
    return;
  }
  t_float voltage = measurings * 5.f / ECG_SAMPLE / 1024;
  commSend_writeUByte(COMMAND_ECG_READ_VALUE);
  commSend_writeUByte(rand() & 0b11111111);
  commSend_writeFloat(voltage);
  commSend_writeCRC();
  readingEcg = false;
}
void read_oxy_handle(t_ubyte i, t_uint measurings = 0) {
  readingOxy = true;
  i++;
  measurings += analogRead(OXY_PIN);
  if(i < OXY_SAMPLE) {
    microtask.setTimeout([i, measurings]() {
      read_oxy_handle(i, measurings);
    }, OXY_GAP);
    return;
  }
  t_float voltage = measurings * 5.f / OXY_SAMPLE / 1024;
  commSend_writeUByte(COMMAND_OXY_READ_VALUE);
  commSend_writeUByte(rand() & 0b11111111);
  commSend_writeFloat(voltage);
  commSend_writeCRC();
  readingOxy = false;
}
void read_bodytemp_handle(t_ubyte i, t_uint measurings = 0) {
  readingBodyTemp = true;
  i++;
  measurings += analogRead(BODYTEMP_PIN);
  if(i < BODYTEMP_SAMPLE) {
    microtask.setTimeout([i, measurings]() {
      read_bodytemp_handle(i, measurings);
    }, BODYTEMP_GAP);
    return;
  }
  t_float voltage = measurings * 5.f / BODYTEMP_SAMPLE / 1024;
  commSend_writeUByte(COMMAND_BODYTEMP_READ_VALUE);
  commSend_writeUByte(rand() & 0b11111111);
  commSend_writeFloat(voltage);
  commSend_writeCRC();
  readingBodyTemp = false;
}
void read_envtemp_handle(t_ubyte i, t_float measurings = 0) {
  readingEnvTemp = true;
  i++;
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  measurings += event.temperature;
  if(i < ENVTEMP_SAMPLE) {
    microtask.setTimeout([i, measurings]() {
      read_envtemp_handle(i, measurings);
    }, ENVTEMP_GAP);
    return;
  }
  t_float value = measurings / ENVTEMP_SAMPLE;
  commSend_writeUByte(COMMAND_ENVTEMP_READ_VALUE);
  commSend_writeUByte(rand() & 0b11111111);
  commSend_writeFloat(value);
  commSend_writeCRC();
  readingEnvTemp = false;
}
void read_envhumid_handle(t_ubyte i, t_float measurings = 0) {
  readingEnvHumid = true;
  i++;
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  measurings += event.relative_humidity;
  if(i < ENVHUMID_SAMPLE) {
    microtask.setTimeout([i, measurings]() {
      read_envhumid_handle(i, measurings);
    }, ENVHUMID_GAP);
    return;
  }
  t_float value = measurings / ENVTEMP_SAMPLE;
  commSend_writeUByte(COMMAND_ENVHUMID_READ_VALUE);
  commSend_writeUByte(rand() & 0b11111111);
  commSend_writeFloat(value);
  commSend_writeCRC();
  readingEnvHumid = false;
}

ThreadController threadController;
Thread microtaskThread;
Thread mainThread;

void microtask_task() {
  microtaskThread.enabled = false;
  microtask.handle();
  microtaskThread.enabled = true;
}
void main_task() {
  mainThread.enabled = false;
  commReceive_consume();
  t_uint status;
  do {
    commReceive_captureBailout();
    status = comm_process(false);
    if(status == 0)
      lastActive = now;
    else if(status == 1)
      commReceive_applyBailout();
    else if(status == 2) {
      commReceive_applyBailout();
      boolean recovered = false;
      for(t_ubyte i = 0; i < 8 && i < commReceiveLength; i++) {
        commReceive_captureBailout();
        commReceiveIndex += i + 1;
        commReceiveLength -= i + 1;
        status = comm_process(true);
        if(status != 0) {
          commReceive_applyBailout();
          continue;
        }
        recovered = true;
        WARN("Recovered with offset ");
        WARN(i + 1);
        WARN("\n");
        break;
      }
      if(!recovered) {
        commReceiveIndex++;
        commReceiveLength--;
      }
    }
  } while(status == 0);
  if(now - lastActive >= 30000) {
    lastActive = now;
    commReceiveIndex = 0;
    commReceiveLength = 0;
    commReceiveCrcValue = 0;
    commReceiveBailoutIndex = 0;
    commReceiveBailoutLength = 0;
    commReceiveBailoutCrcValue = 0;
    ecgReadValue = 0;
    oxyReadValue = 0;
    bodyTempReadValue = 0;
    envTempReadValue = 0;
    envHumidReadValue = 0;
    WARN("Reset\n");
  }
  if(ecgReadValue && !readingEcg && (now - lastEcgRead >= ECG_DELAY)) {
    lastEcgRead = now;
    read_ecg_handle(0, 0);
  }
  if(oxyReadValue && !readingOxy && (now - lastOxyRead >= OXY_DELAY)) {
    lastOxyRead = now;
    read_oxy_handle(0, 0);
  }
  if(bodyTempReadValue && !readingBodyTemp && (now - lastBodyTempRead >= BODYTEMP_DELAY)) {
    lastBodyTempRead = now;
    read_bodytemp_handle(0, 0);
  }
  if(envTempReadValue && !readingEnvTemp && (now - lastEnvTempRead >= ENVTEMP_DELAY)) {
    lastEnvTempRead = now;
    read_envtemp_handle(0, 0);
  }
  if(envHumidReadValue && !readingEnvHumid && (now - lastEnvHumidRead >= ENVHUMID_DELAY)) {
    lastEnvHumidRead = now;
    read_envhumid_handle(0, 0);
  }
  mainThread.enabled = true;
}

void setup() {
  Serial.begin(9600);
  
  microtaskThread.onRun(microtask_task);
  mainThread.onRun(main_task);
  threadController.add(&microtaskThread);
  threadController.add(&mainThread);

  dht.begin();
}

void loop() {
  now = millis();
  threadController.run();
}
