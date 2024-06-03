#define DEBUG

#ifdef DEBUG
#define DEBUG_begin(x) Serial.begin(x)
#define DEBUG_print(x) Serial.print(x)
#define DEBUG_println(x) Serial.println(x)
#define DEBUG_wait() while(!Serial)yield()
#define DEBUG_printBufferReverse(x,y,z) Serial.printBufferReverse(x,y,z)
#define DEBUG_printHEX(x) Serial.print(x,HEX)
#define DEBUG_printlnHEX(x) Serial.println(x,HEX)
#else
#define DEBUG_begin(x)
#define DEBUG_print(x)
#define DEBUG_println(x)
#define DEBUG_wait()
#define DEBUG_printBufferReverse(x,y,z)
#define DEBUG_printHEX(x)
#define DEBUG_printlnHEX(x)
#endif

#include <bluefruit.h>

BLEClientService        CyclingPower(UUID16_SVC_CYCLING_POWER); //0x1818
BLEClientCharacteristic CyclingPowerMeasurement(UUID16_CHR_CYCLING_POWER_MEASUREMENT); //0x2A63

char PrphName[] = "ASSIOMA33173U";

void setup() {
  DEBUG_begin(115200);
  DEBUG_wait();

  DEBUG_println("BLE Cycling Power Central");

  Bluefruit.autoConnLed(false);
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4); //maximum for nRF52832
  Bluefruit.setName("Cycling Power Central");

  CyclingPower.begin();
  CyclingPowerMeasurement.setNotifyCallback(notify_callback);
  CyclingPowerMeasurement.begin();

  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);

  //Scan Setting
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);
  //ScanResponce for ActiveScan is rejected when filtering by UUID.
  //Because ScanResponce does not contain service UUID information.

  DEBUG_println("Active Scanning ...");
}

void loop() {
  // do nothing
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  if (report->type.scan_response)
  {
    DEBUG_print("ScanResponce received from : ");
    DEBUG_printBufferReverse(report->peer_addr.addr, 6, ' '); //MAC is in little endian --> print reverse
    DEBUG_println();

    DEBUG_print("RSSI : ");
    DEBUG_print(report->rssi);
    DEBUG_println(" dBm");

    if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
    {
      DEBUG_print("NAME : ");
      for (int i = 0; i < sizeof(buffer); i++) {
        DEBUG_print(char(buffer[i]));
      }
      DEBUG_println();

      if (!memcmp(buffer, PrphName, sizeof(PrphName))) //PrphName配列の長さが変わるならここも編集する
      {
        if (Bluefruit.Central.connect(report))
        {
          DEBUG_println("NAME matched. Connection established.");
          //memset(buffer, 0, sizeof(buffer));
          return; //Stop Scanning.
        }
        else
        {
          DEBUG_println("NAME matched. Connection failed.");
        }
      }
      else
      {
        DEBUG_println("NAME does not match.");
      }
    }
    else
    {
      DEBUG_println("No COMPLETE LOCAL NAME found.");
    }
  }
  else
  {
    //do nothing for advertise.
  }
  //memset(buffer, 0, sizeof(buffer));
  //parseReportByTypeを別の種類で実行する場合はbufferをクリアする.この場合は不要.
  Bluefruit.Scanner.resume();
}

void connect_callback(uint16_t conn_handle)
{
  DEBUG_println("Connection established.");

  DEBUG_print("Discovering Cycling Power Service ... ");
  if (!CyclingPower.discover(conn_handle))
  {
    DEBUG_println("Found NONE");
    Bluefruit.disconnect(conn_handle);
    DEBUG_println("Disconnected");
    return;
  }
  DEBUG_println("Found it"); //Once Cycling Power service is found, we continue to discover its characteristic.

  DEBUG_print("Discovering Cycling Power Measurement characteristic ... ");
  if (!CyclingPowerMeasurement.discover() )
  {
    DEBUG_println("Found NONE");
    Bluefruit.disconnect(conn_handle);
    DEBUG_println("Disconnected");
    return;
  }
  DEBUG_println("Found it");

  if (!CyclingPowerMeasurement.enableNotify())
  {
    DEBUG_println("Couldn't enable notify. Increase DEBUG LEVEL for troubleshooting");
    Bluefruit.disconnect(conn_handle);
    DEBUG_println("Disconnected");
    return;
  }
  DEBUG_println("Enable notify. Ready to receive value.");

}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  DEBUG_print("Disconnected, reason = 0x"); DEBUG_printlnHEX(reason);
}

uint16_t frag = 0;
int16_t power = 0;
uint16_t revolution = 0;
uint16_t revolution_last = 0;
uint16_t timestamp = 0;
uint16_t timestamp_last = 0;

uint16_t cadence = 0;

void notify_callback(BLEClientCharacteristic * chr, uint8_t* data, uint16_t len)
{
  /*
    DEBUG_print("Measurement : ");
    for (int i = 0; i < len; i++)
    {
    DEBUG_print(data[i]);
    DEBUG_print(" ");
    }
    DEBUG_println();
  */

  timestamp = word(data[7], data[6]); //単位は1/2048[s]じゃないっぽい
  revolution = word(data[5], data[4]);
  power = word(data[3], data[2]);
  frag = word(data[1], data[0]);

  if (revolution > revolution_last)
  {
    //cadence = 60.0 / ( ((timestamp - timestamp_last) / 1024.0) / (revolution - revolution_last) );
    cadence = 60 * 1024 * (revolution - revolution_last) / (timestamp - timestamp_last);
  }

  //DEBUG_print("timestamp : ");
  //DEBUG_println(timestamp);
  //DEBUG_print("revolutions : ");
  //DEBUG_println(revolution);
  DEBUG_print("power : ");
  DEBUG_println(power);
  //DEBUG_print("frags : ");
  //DEBUG_println(frag);
  DEBUG_print("cadence : ");
  DEBUG_println(cadence);

  timestamp_last = timestamp;
  revolution_last = revolution;
}
