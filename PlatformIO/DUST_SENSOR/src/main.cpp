#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "ESP8266WebServer.h"
#include "EEPROM.h"

#define CONNECT_WIFI      '0'
#define CHECK_TCP         '1'
#define SEND_DATA         '2'
#define DISCONNECT_WIFI   '3'
#define RESET             '4'
#define SOFT_AP           '5'
#define CHECK_WIFI        '6'
#define CHECK_TCP_USER    '7'

#define DEBUG   0

typedef struct 
{
  float PM2_5;
  float PM4_0;
  float PM1_0;
  float PM10;
} DUST_t;

String _user_SSID = "";
String _user_PASSWORD = "";
String _user_IP = "";
String _user_PORT = "";
String st;
String content;
int statusCode;
const char* ssid_AP = "IMET_DUST_PM";
const char* password_AP = "";

char ssid[30] = "";
char pass[30] = "";
String IP = "";
uint16_t port = 0;

ESP8266WebServer server(80);

WiFiClient client;

char buffer[100];
byte idx = 0;
bool isReceiving = false;

uint16_t num = 0;
bool timeOut = false;

void processCommand(void);
bool testWifi(void);
void launchWeb(void);
void setupAP(void);
void getUserWifi(void);
void createWebServer(void);
boolean smartConfig(void);

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  client.stopAll();

#if DEBUG == 1
  Serial.println();
  Serial.println("STARTED!");
#endif

  getUserWifi();

  WiFi.begin(_user_SSID.c_str(), _user_PASSWORD.c_str());

  if (testWifi())
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("FAIL");
  }

  delay(2000);
}

void loop()
{

  while (Serial.available() > 0)
  {
    byte c = Serial.read();
    if (!isReceiving && (c == 0xAA) && (idx == 0))   isReceiving =  true;
    if ((idx < sizeof(buffer)) && isReceiving)
    {
      buffer[idx++] = c;
      if (((c == 0xBB) && (idx != 1)) || (idx >= sizeof(buffer)))
      {
#if DEBUG == 1
        Serial.println(buffer);
#endif
        processCommand();
        memset(buffer, '\0', sizeof(buffer));
        idx = 0;
        isReceiving = false;
      }
    }
  }

//   if (WiFi.status() != WL_CONNECTED && isKeepWifiConnect)
//   {
// #if DEBUG == 1
//       Serial.println("Reconnecting...");
// #endif
//     WiFi.disconnect();
//     delay(500);
//     WiFi.mode(WIFI_STA);
//     WiFi.disconnect();

//     WiFi.begin(ssid, pass);
//     int j = 0;
//     while ((WiFi.status() != WL_CONNECTED) && (j <= 30)) 
//     {
//       delay(500);
//       j++;
// #if DEBUG == 1
//       Serial.print(".");
// #endif
//     }
//   }

}


void processCommand(void)
{
  if ((buffer[0] == 0xAA) && (buffer[idx-1] == 0xBB))
  {
    switch (buffer[1])
    {
      case CONNECT_WIFI:
      {
#if DEBUG == 1
        Serial.print("Old ssid: ");
        Serial.println(ssid);
#endif
        memset(ssid, '\0', sizeof(ssid));
        int i = 2;
        while (buffer[i] != ',')
        {
          ssid[i-2] = buffer[i];
          i++;
        }
#if DEBUG == 1
        Serial.print("New ssid: ");
        Serial.println(ssid);
#endif
        i++;
        int temp = i;
#if DEBUG == 1
        Serial.print("Old pass: ");
        Serial.println(pass);
#endif
        memset(pass, '\0', sizeof(pass));
        while (buffer[i] != 0xBB)
        {
          pass[i - temp] = buffer[i];
          i++;
        }
#if DEBUG == 1
        Serial.print("New pass: ");
        Serial.println(pass);
#endif
        WiFi.disconnect();
        delay(500);
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        WiFi.begin(ssid, pass);
        i = 0;
        while ((WiFi.status() != WL_CONNECTED) && (i <= 30)) 
        {
          delay(500);
          i++;
#if DEBUG == 1
          Serial.print(".");
#endif
        }
        if ((WiFi.status() != WL_CONNECTED) || (i >= 30))
        {
          Serial.println("ERROR");
          WiFi.mode(WIFI_STA);
          WiFi.disconnect();
          break;
        }
        else
        {
          Serial.println("OK");
        }
#if DEBUG == 1
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
#endif
        // isKeepWifiConnect = true;
        break;
      }
      
      case CHECK_TCP:
      {
#if DEBUG == 1
        Serial.print("Old IP: ");
        Serial.println(IP.c_str());
#endif
        int i = 2;
        while (buffer[i] != ',')
        {
          IP[i-2] = buffer[i];
          i++;
        }
#if DEBUG == 1
        Serial.print("New IP String: ");
        Serial.println(IP);
#endif
        i++;
#if DEBUG == 1
        Serial.print("Old Port: ");
        Serial.println(port);
#endif
        port = 0;
        while (buffer[i] != 0xBB)
        {
          port = port*10 + (buffer[i] - '0');
          i++;
        }
#if DEBUG == 1
        Serial.print("New Port: ");
        Serial.println(port);
#endif
        if (!client.connect(IP.c_str(), port))
        {
          Serial.println("FAIL");
        }
        else if (client.connected())
        {
          Serial.println("OK");
          client.stop();
        }
        break;
      }
      
      case SEND_DATA:
      {
        char data[20] = {0};
        int i = 2;
        while (buffer[i] != 0xBB)
        {
          data[i-2] = buffer[i];
          i++;
        }
        data[i-2] = '\0';
#if DEBUG == 1
        Serial.println(data);
#endif
        if (!client.connect(IP.c_str(), port))
        {
          Serial.println("FAIL");
        }
        else if (client.connected())
        {
          client.write(data, (i-2));
          Serial.println("OK");
          client.stop();
        }
        break;
      }

      case DISCONNECT_WIFI:
      {
        WiFi.disconnect();
        delay(500);

        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        Serial.println("OK");

        break;
      }

      case RESET:
      {
        WiFi.disconnect();
        delay(500);
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        Serial.println("OK");
        ESP.restart();
        break;
      }

      case SOFT_AP:
      {
        WiFi.disconnect();
        delay(500);

        setupAP();
        Serial.println("OK");
        memset(buffer, '\0', sizeof(buffer));
        idx = 0;
        while (1)
        {
          server.handleClient();
          while (Serial.available() > 0)
          {
            byte c = Serial.read();
            if (!isReceiving && (c == 0xAA) && (idx == 0))   isReceiving =  true;
            if ((idx < sizeof(buffer)) && isReceiving)
            {
              buffer[idx++] = c;
              if (((c == 0xBB) && (idx != 1)) || (idx >= sizeof(buffer)))
              {
#if DEBUG == 1
                Serial.println(buffer);
#endif
                processCommand();
                memset(buffer, '\0', sizeof(buffer));
                idx = 0;
              }
            }
          }
        }
        break;
      }

      case CHECK_WIFI:
      {
#if DEBUG == 1
        Serial.println("Check WiFi connection");
#endif
        if (WiFi.status() == WL_CONNECTED)
        {
          Serial.println("OK");
        }
        else
        {
          Serial.println("FAIL");
        }
        break;
      }

      case CHECK_TCP_USER:
      {
        if (!client.connect(IP.c_str(), port))
        {
          Serial.println("FAIL");
        }
        else if (client.connected())
        {
          Serial.println("OK");
          client.stop();
        }
        break;
      }

      default:
      {
        Serial.println("ERROR");
        break;
      }

    }
  }
}


/**
 * @brief Check the connection of wifi.
 * 
 * @return True if the device was successfully connected, otherwise false.
 */
bool testWifi(void)
{
  int c = 0;

#if DEBUG == 1
  Serial.println("Waiting for Wifi to connect");
#endif

  while (c < 30)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      return true;
    }
    delay(500);
#if DEBUG == 1
    Serial.print(".");
#endif
    c++;
  }
#if DEBUG == 1
  Serial.println("");
  Serial.println("Connect timed out!");
#endif
  return false;
}

/**
 * @brief Launch WebServer of ESP32 in AP mode.
 * 
 * @return None.
 */
void launchWeb()
{
#if DEBUG == 1
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("WiFi connected");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
#endif
  createWebServer();
  // Start the server
  server.begin();
#if DEBUG == 1
  Serial.println("Server started");
#endif
}

/**
 * @brief Set up ESP32 in AP mode and Scan wifi available.
 * 
 * @return None.
 */
void setupAP(void)
{
  WiFi.mode(WIFI_AP);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
#if DEBUG == 1
  Serial.println("scan done");
  if (n == 0)
  {
    Serial.println("no networks found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");

    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      //Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
#endif
  st = "<ol>";
  for (int i = 0; i < n; ++i)
  {
    // Print SSID and RSSI for each network found
    st += "<li>";
    st += WiFi.SSID(i);
    st += " (";
    st += WiFi.RSSI(i);

    st += ")";
    //st += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*";
    st += "</li>";
  }
  st += "</ol>";
  delay(100);
  WiFi.softAP(ssid_AP, password_AP);
  launchWeb();
}

/**
 * @brief Read User's SSID and PASSWORD from EEPROM.
 * 
 * @return None.
 */
void getUserWifi()
{
  EEPROM.begin(256);
  for (int i = 0; i < 32; ++i)
  {
    _user_SSID += char(EEPROM.read(i));
  }
#if DEBUG == 1
  Serial.println();
  Serial.println("Reading EEPROM SSID");
  Serial.print("SSID: ");
  Serial.println(_user_SSID);
#endif
  for (int i = 32; i < 96; ++i)
  {
    _user_PASSWORD += char(EEPROM.read(i));
  }
#if DEBUG == 1
  Serial.println("Reading EEPROM PASS");
  Serial.print("PASS: ");
  Serial.println(_user_PASSWORD);
#endif
  for (int i = 96; i < 150; ++i)
  {
    _user_IP += char(EEPROM.read(i));
  }
#if DEBUG == 1
  Serial.println("Reading EEPROM IP");
  Serial.print("IP: ");
  Serial.println(_user_IP);
#endif
  for (int i = 150; i < 160; ++i)
  {
    _user_PORT += char(EEPROM.read(i));
  }
#if DEBUG == 1
  Serial.println("Reading EEPROM PORT");
  Serial.print("PORT: ");
  Serial.println(_user_PORT);
#endif
  for (int i = 0; _user_IP[i] != '\0'; i++)
  {
    IP += _user_IP[i];
  }
#if DEBUG == 1
  Serial.print("IP: ");
  Serial.println(IP);
#endif
  port = 0;
  byte countPort = 0;
  while (_user_PORT[countPort] != '\0')
  {
    port = port*10 + (_user_PORT[countPort] - '0');
    countPort++;
  }
#if DEBUG == 1
  Serial.print("PORT: ");
  Serial.println(port);
#endif

  EEPROM.end();
}

/**
 * @brief Create UI and Page of Webserver .
 * 
 * @return None.
 */
void createWebServer()
{
  {
    /* ---------------Main Page -----------------*/
    server.on("/", []() {

      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
      content = "<!DOCTYPE HTML>\r\n<html>Welcome to Wifi Credentials Update page";
      content += "<form action=\"/scan\" method=\"POST\"><input type=\"submit\" value=\"scan\"></form>";
      content += ipStr;
      content += "<br><br><label style = \"margin-left : 23px\" > WIFI AVAILABLE </lable>";
      content += "<p>";
      content += st;
      content += "</p><form method='get' action='setting'><label>SSID: </label><input name='ssid' length=32 style = \"margin-left : 100px\"> <br>";
      content += "<label>PASSWORD: </label><input name='pass' length=64 style = \"margin-left : 100px\"> <br>";
      content += "<label>IP: </label><input name='ip' length=12 style = \"margin-left : 100px\"> <br>";
      content += "<label>PORT: </label><input name='port' length=12 style = \"margin-left : 100px\"> <br>";
      content += "<input type='submit'></form>";
      content += "</html>";
      server.send(200, "text/html", content);
    });

    /* ---------------Setting Page -----------------
    
     ** Refresh wifi available.
     ** Back and reload page to perform new available wifi.
    
    */
    server.on("/scan", []() {
      setupAP();
      IPAddress ip = WiFi.softAPIP();
      String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);

      content = "<!DOCTYPE HTML>\r\n<html>go back";
      server.send(200, "text/html", content);
    });

    /* ---------------Setting Page -----------------
    
     ** Reading User's SSID and PASSWORD submitted from Webserver.
    
    */
    server.on("/setting", []() {
      String qsid = server.arg("ssid");
      String qpass = server.arg("pass");
      String qip = server.arg("ip");
      String qport = server.arg("port");

      EEPROM.begin(256);
      if (qsid.length() > 0 && qpass.length() > 0)
      {
#if DEBUG == 1
        Serial.println("clearing eeprom");
#endif
        for (int i = 0; i < 160; ++i) 
        {
          EEPROM.write(i, 0);
        }
#if DEBUG == 1
        Serial.println(qsid);
        Serial.println(qpass);
        Serial.println(qip);
        Serial.println(qport);
        Serial.println("writing eeprom ssid:");
#endif
        for (unsigned int i = 0; i < qsid.length(); ++i)
        {
          EEPROM.write(i, qsid[i]);
#if DEBUG == 1
          Serial.print("Wrote: ");
          Serial.println(qsid[i]);
#endif
        }
#if DEBUG == 1
        EEPROM.write(qsid.length(), 0);
        Serial.println("writing eeprom pass:");
#endif
        for (unsigned int i = 0; i < qpass.length(); ++i)
        {
          EEPROM.write(32 + i, qpass[i]);
#if DEBUG == 1
          Serial.print("Wrote: ");
          Serial.println(qpass[i]);
#endif
        }
#if DEBUG == 1
        Serial.println("writing ip:");
#endif
        for (unsigned int i = 0; i < qip.length(); ++i)
        {
          EEPROM.write(96 + i, qip[i]);
#if DEBUG == 1
          Serial.print("Wrote: ");
          Serial.println(qip[i]);
#endif
        }
#if DEBUG == 1
        Serial.println("writing port:");
#endif
        for (unsigned int i = 0; i < qport.length(); ++i)
        {
          EEPROM.write(150 + i, qport[i]);
#if DEBUG == 1
          Serial.print("Wrote: ");
          Serial.println(qport[i]);
#endif
        }
        // EEPROM.write(qpass.length()+32, 0);
        EEPROM.commit();
        EEPROM.end();

        content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
        statusCode = 200;
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(statusCode, "application/json", content);
        ESP.restart();
      } 
      else 
      {
        content = "{\"Error\":\"404 not found\"}";
        statusCode = 404;
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(statusCode, "application/json", content);
        // Serial.println("Sending 404");
      }
    });
  }
}


boolean smartConfig(void)
{
  WiFi.disconnect();
  delay(10);

  getUserWifi();

  WiFi.begin(_user_SSID.c_str(), _user_PASSWORD.c_str());

  if (testWifi()) {return true;}
  else
  {
#if DEBUG == 1
    Serial.println("Connecting Status Negative");
    Serial.println("Turning the HotSpot On");
#endif
    setupAP();
    while (1)
    {
      server.handleClient();
    }
  }

}


#if DEBUG == 1

#endif
