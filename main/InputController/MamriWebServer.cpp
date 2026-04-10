#include "MamriWebServer.h"

// ============================================================
// TEMP: Arduino / AsyncWebServer implementation disabled
// ============================================================
// Original implementation depended on:
// - LittleFS
// - WiFi
// - ESPAsyncWebServer
// - Arduino String / Serial / millis / delay
//
// These are commented out for the current pure ESP-IDF port.
// Restore or replace later with:
// - esp_wifi / esp_netif
// - esp_http_server
// - esp_littlefs
// - ESP_LOGI / std::string / native timing APIs

MamriWebServer::MamriWebServer()
    // : _server(80)
{
}

bool MamriWebServer::begin() {
    // ========================================================
    // ORIGINAL ARDUINO IMPLEMENTATION
    // ========================================================
    /*
    if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
        Serial.println("LittleFS Mount Failed");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setHostname(MAMRI_HOSTNAME);

    Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Connecting to %s...\n", MAMRI_SSID);

    WiFi.begin(MAMRI_SSID, MAMRI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connect failed");
        return false;
    }

    Serial.println("\nIP: " + WiFi.localIP().toString());

    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/root.html")) {
            request->send(LittleFS, "/root.html", "text/html");
        } else {
            request->send(404, "text/plain", "root.html not found.");
        }
    });

    _server.serveStatic("/", LittleFS, "/");

    _server.on("/robotstate", HTTP_GET, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/robotconfig", HTTP_GET, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/inputmode", HTTP_GET, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/ferristare", HTTP_POST, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/ferrisresettare", HTTP_POST, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/controljoints", HTTP_POST, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/controlposition", HTTP_POST, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/robotframes", HTTP_GET, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/robotframessensor", HTTP_GET, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/iktargets", HTTP_POST, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.on("/movetotargets", HTTP_POST, [this](AsyncWebServerRequest *request) {
        ...
    });

    _server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
    });

    _server.begin();
    Serial.println("HTTP async server started on port 80");
    return true;
    */

    // ========================================================
    // TEMP: webserver disabled during ESP-IDF port
    // ========================================================
    return false;
}

void MamriWebServer::attachRobotController(RobotController* rc) {
    _robotController = rc;
}