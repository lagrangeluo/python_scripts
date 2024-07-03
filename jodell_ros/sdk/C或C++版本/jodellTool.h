#pragma once

#include<iostream>
#include <stdio.h>
#include <tchar.h>
#include<Windows.h>


_declspec(dllexport) int* getComPort();
_declspec(dllexport) int serialOperation(int portNo, int baudRate, bool status);
_declspec(dllexport) int* getStatus(int salveId, int address, int readMode, int count);
_declspec(dllexport) int writeDataToRegister(int salveId, int address, int* sendCmdBuf, int count);
_declspec(dllexport) int clawEnable(int salveId, bool status);
_declspec(dllexport) int rotateEnable(int salveId, bool status);
_declspec(dllexport) int runWithoutParam(int salveId, int cmdId);
_declspec(dllexport) int runWithoutParamOfEvs(int salveId, bool status);
_declspec(dllexport) int rotateWithoutParam(int salveId, int cmdId);
_declspec(dllexport) int runWithParam(int salveId, int pos, int speed, int torque);
_declspec(dllexport) int runWithParamOfEvs(int salveId, int maxData, int minData, int timeout, bool status);
_declspec(dllexport) int runWithParamOfErg32(int salveId, int pos, int speed, int torque);
_declspec(dllexport) int rotateWithParam(int salveId, int angle, int speed, int torque, bool absStatus, int cycleNum);
_declspec(dllexport) int rotateWithParamOfErg26(int salveId, int angle, int speed, int torque);
_declspec(dllexport) int changeSalveId(int oldId, int newId);
_declspec(dllexport) int changeSalveIdOfErg32(int oldId, int newId);
_declspec(dllexport) int changeBaudRate(int salveId, int baudRate);
_declspec(dllexport) int changeBaudRateOfErg32(int salveId, int baudRate);
_declspec(dllexport) int clawEncoderZero(int salveId);
_declspec(dllexport) int switchAutoPatrolInspection(int salveId, bool status);
_declspec(dllexport) int getClawCurrentStatus(int salveId);
_declspec(dllexport) int getClawCurrentStatusOfErg32(int salveId);
_declspec(dllexport) int getRotateCurrentStatusOfErg32(int salveId);
_declspec(dllexport) int getClawCurrentLocation(int salveId);
_declspec(dllexport) int getClawCurrentLocationOfErg32(int salveId);
_declspec(dllexport) int getCurrentAbsAngleOfErg32(int salveId);
_declspec(dllexport) int getCurrentRelAngleOfErg32(int salveId);
_declspec(dllexport) int getClawCurrentSpeed(int salveId);
_declspec(dllexport) int getClawCurrentSpeedOfErg32(int salveId); 
_declspec(dllexport) int getRotateCurrentSpeedOfErg32(int salveId);
_declspec(dllexport) int getClawCurrentTorque(int salveId);
_declspec(dllexport) int getClawCurrentTorqueOfErg32(int salveId);
_declspec(dllexport) int getRotateCurrentTorqueOfErg32(int salveId);
_declspec(dllexport) int getClawCurrentTemperature(int salveId);
_declspec(dllexport) int getClawCurrentTemperatureOfErg32(int salveId);
_declspec(dllexport) int getClawCurrentVoltage(int salveId);
_declspec(dllexport) int getClawCurrentVoltageOfErg32(int salveId);
_declspec(dllexport) int* querySoftwareVersion(int salveId);
_declspec(dllexport) int* querySoftwareVersionOfErg32(int salveId);
