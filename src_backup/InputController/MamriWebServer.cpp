#include "MamriWebServer.h"

MamriWebServer::MamriWebServer() : _server(80) {
}

bool MamriWebServer::begin() {
    if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {// the default constructor uses "spiffs" label, but this triggers a warning
        Serial.println("LittleFS Mount Failed");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    //WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
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

    // Serve root.html
    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/root.html")) {
            request->send(LittleFS, "/root.html", "text/html");
        } else {
            request->send(404, "text/plain", "root.html not found.");
        }
    });

    // This automatically handles requests like /julian/index.html or /tom/index.html
    // It looks inside the LittleFS root for the corresponding path.
    _server.serveStatic("/", LittleFS, "/");

    // Robot state JSON endpoint
    _server.on("/robotstate", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(200, "application/json", "{\"message\":\"RobotController not attached\"}");
            return;
        }

        RobotState robotState = _robotController->getRobotState();

        String json = "{";
        json += "\"jointSteps\":[";
        for (size_t i = 0; i < robotState.jointSteps.size(); i++) {
            json += String(robotState.jointSteps[i]);
            if (i < robotState.jointSteps.size() - 1) json += ",";
        }
        json += "],";

        json += "\"targetJointSteps\":[";
        for (size_t i = 0; i < robotState.targetJointSteps.size(); i++) {
            json += String(robotState.targetJointSteps[i]);
            if (i < robotState.targetJointSteps.size() - 1) json += ",";
        }
        json += "],";

        json += "\"currentPosition\":[";
        for (size_t i = 0; i < robotState.currentPosition.size(); i++) {
            json += String(robotState.currentPosition[i], 4);
            if (i < robotState.currentPosition.size() - 1) json += ",";
        }
        json += "],";

        json += "\"targetPosition\":[";
        for (size_t i = 0; i < robotState.targetPosition.size(); i++) {
            json += String(robotState.targetPosition[i], 4);
            if (i < robotState.targetPosition.size() - 1) json += ",";
        }
        json += "],";

        json += "\"rawFerrisWheelValues\":[";
        for (size_t i = 0; i < robotState.rawFerrisValues.size(); i++) {
            json += String(robotState.rawFerrisValues[i], 4);
            if (i < robotState.rawFerrisValues.size() - 1) json += ",";
        }
        json += "],";

        // Sensor-corrected joint angles in radians (from ferris wheel encoder data)
        std::vector<float> sensorJointAngles = _robotController->stepsToRad(robotState.sensorCorrectedJointSteps);
        
        json += "\"sensorJointSteps\":[";
        for (size_t i = 0; i < robotState.sensorCorrectedJointSteps.size(); i++) {
            json += String(robotState.sensorCorrectedJointSteps[i]);
            if (i < robotState.sensorCorrectedJointSteps.size() - 1) json += ",";
        }
        json += "],";

        json += "\"sensorJointAngles\":[";
        for (size_t i = 0; i < sensorJointAngles.size(); i++) {
            json += String(sensorJointAngles[i], 4);
            if (i < sensorJointAngles.size() - 1) json += ",";
        }
        json += "]";
        json += "}";

        request->send(200, "application/json", json);
    });

    _server.on("/robotconfig", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(200, "application/json", "{\"message\":\"RobotController not attached\"}");
            return;
        }

        RobotConfig robotConfig = _robotController->getRobotConfig();

        // Provide both an ID and a human-readable label for the robot
        int robotId = (int)robotConfig.name;
        String robotLabel;
        switch (robotConfig.name) {
            case PINK_MAMRI: robotLabel = "PINK_MAMRI"; break;
            case PURPLE_MAMRI: robotLabel = "PURPLE_MAMRI"; break;
            // case SUNRAM: robotLabel = "SUNRAM"; break;
            case STEPPER_TESTER: robotLabel = "STEPPER_TESTER"; break;
            case STEPPER_AND_FERRIS: robotLabel = "STEPPER_AND_FERRIS"; break;
            case NO_ROBOT: robotLabel = "NO_ROBOT"; break;

            default: robotLabel = "UNKNOWN"; break;
        }

        float freq = _robotController->getFrequency(1);

        String json = "{";
        json += "\"robotId\":" + String(robotId) + ",";
        json += "\"robotName\":\"" + robotLabel + "\",";
        json += "\"numOfSteppers\":" + String(robotConfig.numOfSteppers) + ",";
        json += "\"numOfFerrisWheels\":" + String(robotConfig.numOfFerrisWheels) + ",";
        json += "\"ferrisWheelsReady\":" + String(_ferrisWheelsReady ? "true" : "false") + ",";
        // motor frequency (pot 1)
        json += "\"motorFrequency\":" + String(freq, 2);
        json += "}";

        request->send(200, "application/json", json);
    });

    _server.on("/inputmode", HTTP_GET, [this](AsyncWebServerRequest *request) {
        String modeStr;
        switch (_currentInputMode) {
            case InputModes::BUTTON_JOINT_MODE:
                modeStr = "BUTTONS CONTROL JOINTS";
                break;
            case InputModes::BUTTON_EE_CONTROL_MODE:
                modeStr = "BUTTONS CONTROL POSITION";
                break;
            // case InputModes::WEBPAGE_MODE:
            //     modeStr = "WEBPAGE CONTROL";
            //     break;
            // case InputModes::TOUCHSCREEN_MODE:
            //     modeStr = "TOUCHSCREEN CONTROL";
            //     break;
            // case InputModes::HANDTOOL_MODE:
            //     modeStr = "HANDTOOL CONTROL";
            //     break;
            // case InputModes::SPACEMOUSE_MODE:
            //     modeStr = "SPACEMOUSE CONTROL";
            //     break;
            default:
                modeStr = "UNKNOWN";
                break;
        }
        String json = "{ \"inputMode\": \"" + modeStr + "\" }";
        request->send(200, "application/json", json);
    });

    // Ferris wheel tare endpoint
    _server.on("/ferristare", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(500, "text/plain", "RobotController not attached");
            return;
        }
        _robotController->ferrisWheelTareCurrentPosition();
        request->send(200, "text/plain", "Ferris wheel tare set");
    });

    // Ferris wheel reset tare endpoint
    _server.on("/ferrisresettare", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(500, "text/plain", "RobotController not attached");
            return;
        }
        _robotController->ferrisWheelResetTare();
        request->send(200, "text/plain", "Ferris wheel tare reset");
    });
    
    _server.on("/controljoints", HTTP_POST, [this](AsyncWebServerRequest *request) {
        // Accept partial list of jointN parameters; missing entries are left as current targets
        if (!_robotController) {
            request->send(500, "text/plain", "RobotController not attached");
            return;
        }
        RobotState rs = _robotController->getRobotState();
        uint8_t num = _robotController->getRobotConfig().numOfSteppers;
        std::vector<int> newTargets;
        newTargets.resize(num);
        // populate with existing targets if available
        for (uint8_t i = 0; i < num; ++i) {
            if (i < rs.targetJointSteps.size()) newTargets[i] = rs.targetJointSteps[i];
            else newTargets[i] = 0;
        }
        bool any = false;
        for (uint8_t i = 0; i < num; ++i) {
            String pname = "joint" + String(i);
            if (request->hasParam(pname, true)) {
                int targetStep = request->getParam(pname, true)->value().toInt();
                newTargets[i] = targetStep;
                any = true;
            }
        }
        if (any) {
            _robotController->setJointTargetSteps(newTargets);
        }
        request->send(200, "text/plain", any ? "OK" : "No valid params");
    });

    _server.on("/controlposition", HTTP_POST, [this](AsyncWebServerRequest *request) {
        // Accept partial list of positionN parameters; missing entries are left as current targets
        if (!_robotController) {
            request->send(500, "text/plain", "RobotController not attached");
            return;
        }
        RobotState rs = _robotController->getRobotState();
        std::vector<float> newPos(6, 0.0f);
        for (uint8_t i = 0; i < 6; ++i) {
            if (i < rs.targetPosition.size()) newPos[i] = rs.targetPosition[i];
        }
        bool any = false;
        for (uint8_t i = 0; i < 6; ++i) {
            String pname = "position" + String(i);
            if (request->hasParam(pname, true)) {
                float targetPos = request->getParam(pname, true)->value().toFloat();
                newPos[i] = targetPos;
                any = true;
            }
        }
        if (any) {
            _robotController->setTargetPosition(newPos);
        }
        request->send(200, "text/plain", any ? "OK" : "No valid params");
    });

    // Robot Frames (FK) Endpoint
    _server.on("/robotframes", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(500, "application/json", "{\"message\":\"RobotController not attached\"}");
            return;
        }

        if (!_robotController->_kinematics) {
            request->send(500, "application/json", "{\"message\":\"This robot has no kinematics controller\"}");
            return;
        }

        RobotState robotState = _robotController->getRobotState();
        auto robotFrames = _robotController->_kinematics->forwardKinematicsAll(robotState.jointSteps);

        String json = "{\"robotFrames\":[";
        
        for (size_t i = 0; i < robotFrames.size(); i++) {
            json += "[";
            for (int r = 0; r < 4; r++) {
                json += "[";
                for (int c = 0; c < 4; c++) {
                    // Using (r, c) access. Change to [r][c] if your Matrix class requires it.
                    json += String(robotFrames[i](r, c), 4); 
                    if (c < 3) json += ",";
                }
                json += "]"; // End row
                if (r < 3) json += ",";
            }
            json += "]"; // End matrix
            if (i < robotFrames.size() - 1) json += ",";
        }
        json += "]}";

        request->send(200, "application/json", json);
    });

    // Robot frames based on sensors (FK with sensor-corrected joint angles) Endpoint
    _server.on("/robotframessensor", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(500, "application/json", "{\"message\":\"RobotController not attached\"}");
            return;
        }

        if (!_robotController->_kinematics) {
            request->send(500, "application/json", "{\"message\":\"This robot has no kinematics controller\"}");
            return;
        }

        RobotState robotState = _robotController->getRobotState();
        auto robotFramesSensor = _robotController->_kinematics->forwardKinematicsAll(robotState.sensorCorrectedJointSteps);

        String json = "{\"robotFramesSensor\":[";
        
        for (size_t i = 0; i < robotFramesSensor.size(); i++) {
            json += "[";
            for (int r = 0; r < 4; r++) {
                json += "[";
                for (int c = 0; c < 4; c++) {
                    // Using (r, c) access. Change to [r][c] if your Matrix class requires it.
                    json += String(robotFramesSensor[i](r, c), 4); 
                    if (c < 3) json += ",";
                }
                json += "]"; // End row
                if (r < 3) json += ",";
            }
            json += "]"; // End matrix
            if (i < robotFramesSensor.size() - 1) json += ",";
        }
        json += "]}";

        request->send(200, "application/json", json);
    });

    // IK Targets endpoint
    _server.on("/iktargets", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(500, "application/json", "{\"error\":\"RobotController not attached\"}");
            return;
        }

        if (!_robotController->_kinematics) {
            request->send(500, "application/json", "{\"error\":\"This robot has no kinematics controller\"}");
            return;
        }

        // Extract JSON values manually from body
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        
        int solutionNumber = -1; // Specifies which solution number from the IK solver to return (-1 for closest solution)

        // Try to extract from URL-encoded parameters first
        if (request->hasParam("x", true)) x = request->getParam("x", true)->value().toFloat();
        if (request->hasParam("y", true)) y = request->getParam("y", true)->value().toFloat();
        if (request->hasParam("z", true)) z = request->getParam("z", true)->value().toFloat();
        if (request->hasParam("roll", true)) roll = request->getParam("roll", true)->value().toFloat();
        if (request->hasParam("pitch", true)) pitch = request->getParam("pitch", true)->value().toFloat();
        if (request->hasParam("yaw", true)) yaw = request->getParam("yaw", true)->value().toFloat();
        if (request->hasParam("solutionNumber", true)) solutionNumber = request->getParam("solutionNumber", true)->value().toInt();


        Serial.printf("Received IK target: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f\n", x, y, z, roll, pitch, yaw);

        Serial.printf("Requested IK solution number: %d\n", solutionNumber);

        // Create target vector: [x, y, z, pitch, yaw, roll]
        std::vector<float> target = {x, y, z, pitch, yaw, roll};
        
        // Convert to homogeneous matrix
        auto H_desired = _robotController->_kinematics->targetToHomogeneous(target);
        
        // Solve inverse kinematics
        auto ikSolutions = _robotController->_kinematics->inverseKinematics(H_desired);
        
        if (ikSolutions.empty()) {
            request->send(400, "application/json", "{\"error\":\"No IK solution found\"}");
            return;
        }

        if (solutionNumber != -1) {
            if (solutionNumber < 0 || solutionNumber >= ikSolutions.size()) {
                request->send(400, "application/json", "{\"error\":\"Invalid solution number: " + String(solutionNumber) + "\"}");
                return;
            }
        }


        // Choose IK solution based on solutionNumber parameter; if -1, pick closest solution to current joint steps
        std::vector<int> ikSolution;
        if (solutionNumber == -1) {
            // Get current joint steps and pick closest solution
            RobotState robotState = _robotController->getRobotState();
            ikSolution = _robotController->_kinematics->pickClosestSolution(ikSolutions, robotState.jointSteps);
        } else {
            ikSolution = ikSolutions[solutionNumber];
        }

        // Compute forward kinematics for the solution
        auto robotFrames = _robotController->_kinematics->forwardKinematicsAll(ikSolution);

        // Build response JSON with robot frames
        String json = "{\"ikSolution\":[";
        for (size_t i = 0; i < ikSolution.size(); i++) {
            json += String(ikSolution[i]);
            if (i < ikSolution.size() - 1) json += ",";
        }
        json += "],\"robotFrames\":[";
        
        for (size_t i = 0; i < robotFrames.size(); i++) {
            json += "[";
            for (int r = 0; r < 4; r++) {
                json += "[";
                for (int c = 0; c < 4; c++) {
                    json += String(robotFrames[i](r, c), 4);
                    if (c < 3) json += ",";
                }
                json += "]";
                if (r < 3) json += ",";
            }
            json += "]";
            if (i < robotFrames.size() - 1) json += ",";
        }
        json += "]}";

        request->send(200, "application/json", json);
    });

    _server.on("/movetotargets", HTTP_POST, [this](AsyncWebServerRequest *request) {
        if (!_robotController) {
            request->send(500, "application/json", "{\"error\":\"RobotController not attached\"}");
            return;
        }

        if (!_robotController->_kinematics) {
            request->send(500, "application/json", "{\"error\":\"This robot has no kinematics controller\"}");
            return;
        }

        // Extract JSON values manually from body
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        
        int solutionNumber = -1; // Specifies which solution number from the IK solver to return (-1 for closest solution)

        // Try to extract from URL-encoded parameters first
        if (request->hasParam("x", true)) x = request->getParam("x", true)->value().toFloat();
        if (request->hasParam("y", true)) y = request->getParam("y", true)->value().toFloat();
        if (request->hasParam("z", true)) z = request->getParam("z", true)->value().toFloat();
        if (request->hasParam("roll", true)) roll = request->getParam("roll", true)->value().toFloat();
        if (request->hasParam("pitch", true)) pitch = request->getParam("pitch", true)->value().toFloat();
        if (request->hasParam("yaw", true)) yaw = request->getParam("yaw", true)->value().toFloat();
        if (request->hasParam("solutionNumber", true)) solutionNumber = request->getParam("solutionNumber", true)->value().toInt();


        Serial.printf("Received IK target: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f\n", x, y, z, roll, pitch, yaw);

        Serial.printf("Requested IK solution number: %d\n", solutionNumber);

        // Create target vector: [x, y, z, pitch, yaw, roll]
        std::vector<float> target = {x, y, z, pitch, yaw, roll};
        
        // Convert to homogeneous matrix
        auto H_desired = _robotController->_kinematics->targetToHomogeneous(target);
        
        // Solve inverse kinematics
        auto ikSolutions = _robotController->_kinematics->inverseKinematics(H_desired);
        
        if (ikSolutions.empty()) {
            request->send(400, "application/json", "{\"error\":\"No IK solution found\"}");
            return;
        }

        if (solutionNumber != -1) {
            if (solutionNumber < 0 || solutionNumber >= ikSolutions.size()) {
                request->send(400, "application/json", "{\"error\":\"Invalid solution number: " + String(solutionNumber) + "\"}");
                return;
            }
        }

        // Choose IK solution based on solutionNumber parameter; if -1, pick closest solution to current joint steps
        std::vector<int> ikSolution;
        if (solutionNumber == -1) {
            // Get current joint steps and pick closest solution
            RobotState robotState = _robotController->getRobotState();
            ikSolution = _robotController->_kinematics->pickClosestSolution(ikSolutions, robotState.jointSteps);
        } else {
            ikSolution = ikSolutions[solutionNumber];
        }

        // Move robot to the IK solution
        _robotController->setJointTargetSteps(ikSolution);
    });

    _server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
    });

    _server.begin();
    Serial.println("HTTP async server started on port 80");
    return true;
}

void MamriWebServer::attachRobotController(RobotController* rc) {
    _robotController = rc;
}