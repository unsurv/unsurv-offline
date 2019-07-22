/*  Adapted from ESP32 Arduino Wifi scan example
 *  This sketch demonstrates how to scan WiFi networks.
 *  The API is almost the same as with the WiFi Shield library,
 *  the most obvious difference being the different file you need to include:
 *  
 *  Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.BSSIDstr(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
 *  
 *  
 */
 
#include "WiFi.h"
#include <map>  


class CapturedWifi{
  public:

  String ssid;
  String bssid[20];
  int uniqueBssids;
  unsigned long firstSeen;
  unsigned long lastSeen;

  String toString(){
    String bssids;
    for (int i=0; i < 20; i++) {
      
      if (bssid[i] != ""){
        bssids += bssid[i] + "\n";
      }
      
    }
    
    String representation = 
    ssid + "\n" + 
    bssids +  // newline already at the end of bssids
    String(firstSeen) + "\n" +
    String(lastSeen) + "\n";
    
    return representation;
  }
  
};

std::map <String, CapturedWifi> observedNetworks = {};


void setup()
{
    Serial.begin(115200);

    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("Setup done");
}

void loop()
{
  
    //Serial.println(ESP.getFreeHeap());

    Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            

            

            //observedNetworks[WiFi.SSID(i)] = wifiCapture;

            processWifiCapture(i);
           
            
            delay(10);
        }
        
    }
    
    Serial.println(observedNetworks.size());
    Serial.println("");

    // Wait a bit before scanning again
    delay(5000);
}

void processWifiCapture(int index){
  String ssid = WiFi.SSID(index);
  String bssid = WiFi.BSSIDstr(index);
  std::map <String, CapturedWifi>::iterator it = observedNetworks.find(ssid);
  
  if (it != observedNetworks.end()) {  // if network seen before, 
    //what if network is at last index??
    
    //TODO check if bssid in array, add to array if not, if yes alarm
    //TODO if array of size 20 full, delete first element and add to end
    //TODO use firstSeen lastSee

    // get previous capture
    CapturedWifi seenWifi = observedNetworks[ssid];

    boolean newBssidForSsid = true;

    for (int i=0; i < seenWifi.uniqueBssids; i++) {
      if (bssid == seenWifi.bssid[i]){
        newBssidForSsid = false;
      }
    }

    if (newBssidForSsid){
      seenWifi.bssid[seenWifi.uniqueBssids] = bssid;
      
      seenWifi.uniqueBssids += 1;
      seenWifi.lastSeen = millis();
      // notifyUser();
      Serial.println("Found SSID with multiple APs");
      Serial.print(seenWifi.toString());
      //update original map
      observedNetworks[ssid] = seenWifi;
    }
    
  } else {
    CapturedWifi unseenWifiCapture;

    unseenWifiCapture.ssid = WiFi.SSID(index);
    unseenWifiCapture.bssid[0]= WiFi.BSSIDstr(index);
    unseenWifiCapture.uniqueBssids = 1;
    unseenWifiCapture.firstSeen = millis();
    observedNetworks[WiFi.SSID(index)] = unseenWifiCapture;
    
    Serial.println(WiFi.SSID(index) + " not seen before, adding to db");
    // Serial.print(unseenWifiCapture.toString());
  }
  
};
