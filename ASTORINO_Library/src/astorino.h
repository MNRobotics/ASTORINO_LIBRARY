#include <Arduino.h>


class astorino {
private:
	#define ACK 0x06
	#define NACK 0xCC
	#define MC 0xAA
	#define MAX_PROGRAMS 30
	#define BUFFER_SIZE 128
    HardwareSerial* serialPort;
    int TIMEOUT = 1000;
    long MOTION_TIMEOUT = 300000;

    // Tablica nazw programów
    String programsName[MAX_PROGRAMS];
    int programsCount = 0;

    byte startBytes[2] = {0x01, 0x02};
    int _bufferIndex = 0;
    byte _buffer[BUFFER_SIZE];
    byte status[5];
    byte IO[18];
    double transPoints[100][8];
    double jointPoints[100][8];
    double Tool1[6];
    double Tool2[6];
    double Tool3[6];
    double pose[7] = {0, 0, 0, 0, 0, 0, 0};
    double abs_vel[7] = {0, 0, 0, 0, 0, 0, 0};
    double jt_vel[7] = {0, 0, 0, 0, 0, 0, 0};
    double joints[7] = {0, 0, 0, 0, 0, 0, 0};
    double home[7] = {0, 0, 0, 0, 0, 0, 0};
    double forw[7] = {0, 0, 0, 0, 0, 0, 0};
    double inv[7] = {0, 0, 0, 0, 0, 0, 0};
    double oat[3] = {0, 0, 0};
    double euler[3] = {0, 0, 0};

    byte errorCode = 0;
    byte cpu_temp = 0;

    String sn = "";
    String nm = "";
    String mP = "";
    String sP = "";
    String frmV;
    int rS = 0;
    int errCode = 0;
	String _version = "1.0.0";
	bool isConnected = false;
	
	bool sendData(byte* data, int length);
	byte* WaitForResponse(byte* startBytes, uint32_t timeout, size_t &frameSize);
	
	byte getCRC(byte* buffer, byte size);
	byte ValidateResponse(byte* buffer, int length);
	int DecodeResponse(byte* buffer);
	int BytesToInt32(byte* bytes);
	int ConvertBitsToInt(bool bit0, bool bit1, bool bit2);
	byte* ExtractFrame(byte* startBytes, size_t &frameSize);
	int FindStartBytes(byte* startBytes);
	bool IsStartBytes(int index, byte* startBytes);
	byte sendCommandMotion(byte cmd);
	String ExtractString(byte* byteArray, int startIndex);
	bool IsBitSet(byte b, int pos);
	bool SendFrame(String data, byte cmd);
	bool SendData(byte index, byte cmd, double* values, int length);
	byte sendCommand(byte cmd);
	byte sendCommandMotion(byte cmd, byte spd, byte acc, byte dec);
	bool isLittleEndian();
	void reverseBytes(byte* data, int length);
	int32_t convertToInt32(byte* data);
  void addProgramName(String nm);
  void waitForData();
public:
    
	astorino(HardwareSerial& port);

	void begin(unsigned long baud);
	int available();
    int read();
    size_t write(const uint8_t* buffer, size_t size);
    size_t write(uint8_t data);
	struct RetVal 
	{
        double values[7];
        String name;
        String names[MAX_PROGRAMS];
        int iVal;
        byte bValues[20];
        byte returnCode;

        RetVal() 
		{
            for (int i = 0; i < 7; ++i) values[i] = 0;
            name = "";
            for (int i = 0; i < MAX_PROGRAMS; ++i) names[i] = "";
            iVal = -1;
            for (int i = 0; i < 20; ++i) bValues[i] = 0;
            returnCode = 0;
        }

        RetVal(double* doubleValues, byte ret) 
		{
            for (int i = 0; i < 7; ++i) values[i] = doubleValues[i];
            name = "";
            for (int i = 0; i < MAX_PROGRAMS; ++i) names[i] = "";
            iVal = -1;
            for (int i = 0; i < 20; ++i) bValues[i] = 0;
            returnCode = ret;
        }

        RetVal(String name, byte ret) 
		{
            for (int i = 0; i < 7; ++i) values[i] = 0;
            this->name = name;
            for (int i = 0; i < MAX_PROGRAMS; ++i) names[i] = "";
            iVal = -1;
            for (int i = 0; i < 20; ++i) bValues[i] = 0;
            returnCode = ret;
        }

        RetVal(String* names, byte ret) 
		{
            for (int i = 0; i < 7; ++i) values[i] = 0;
            name = "";
            for (int i = 0; i < MAX_PROGRAMS; ++i) this->names[i] = names[i];
            iVal = -1;
            for (int i = 0; i < 20; ++i) bValues[i] = 0;
            returnCode = ret;
        }

        RetVal(int val, byte ret) 
		{
            for (int i = 0; i < 7; ++i) values[i] = 0;
            name = "";
            for (int i = 0; i < MAX_PROGRAMS; ++i) names[i] = "";
            iVal = val;
            for (int i = 0; i < 20; ++i) bValues[i] = 0;
            returnCode = ret;
        }

        RetVal(byte* array, byte ret) 
		{
            for (int i = 0; i < 7; ++i) values[i] = 0;
            name = "";
            for (int i = 0; i < MAX_PROGRAMS; ++i) names[i] = "";
            iVal = -1;
            for (int i = 0; i < 20; ++i) bValues[i] = array[i];
            returnCode = ret;
        }

        RetVal(byte ret) 
		{
            for (int i = 0; i < 7; ++i) values[i] = 0;
            name = "";
            for (int i = 0; i < MAX_PROGRAMS; ++i) names[i] = "";
            iVal = -1;
            for (int i = 0; i < 20; ++i) bValues[i] = 0;
            returnCode = ret;
        }
    };

    struct TransformationPoints 
	{
        double points[100][8];
        byte returnCode;

        TransformationPoints(double p[100][8], byte ret) 
		{
            for (int i = 0; i < 100; ++i)
                for (int j = 0; j < 8; ++j)
                    points[i][j] = p[i][j];
            returnCode = ret;
        }

        TransformationPoints(byte ret) 
		{
            for (int i = 0; i < 100; ++i)
                for (int j = 0; j < 8; ++j)
                    points[i][j] = 0;
            returnCode = ret;
        }
    };

    struct JointPoints 
	{
        double points[100][8];
        byte returnCode;

        JointPoints(double p[100][8], byte ret) 
		{
            for (int i = 0; i < 100; ++i)
                for (int j = 0; j < 8; ++j)
                    points[i][j] = p[i][j];
            returnCode = ret;
        }

        JointPoints(byte ret) 
		{
            for (int i = 0; i < 100; ++i)
                for (int j = 0; j < 8; ++j)
                    points[i][j] = 0;
            returnCode = ret;
        }
    };
	
	String getLibVersion();
	byte Connect();
	byte Disconnect();
	void setTimeOut(int timeOut);
	void setMotionTimeOut(long timeOut);
	byte cancelMotion();
	byte emergencyStop();
	RetVal ForwardKinematics(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6, double jt7);
	RetVal ForwardKinematics(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6);
	RetVal ForwardKinematics(double* jt, int length);
	RetVal InverseKinematics(double x, double y, double z, double o, double a, double t, double jt7);
	RetVal InverseKinematics(double x, double y, double z, double o, double a, double t);
	RetVal InverseKinematics(double* pose, int lenght);
	RetVal toOAT(double rx, double ry, double rz);
	RetVal toRPY(double o, double a, double t);

	byte setMotorOn();
	byte setMotorOff();
	byte reset();
	byte Hold();
	byte Run();
	byte turnDryRunOn();
	byte turnDryRunOff();
	byte setRepeatCont();
	byte setRepeatOnce();
	byte setStepOnce();
	byte setStepCont();
	byte cycleStop();
	byte cycleStart();
	byte nextStep();
	byte skipWait();
	RetVal readInput(int ind);
	RetVal readOutput(int ind);
	RetVal readInternal(int ind);
	byte setOutput(int output, int state);
	byte setInternal(int sig, int state);
	RetVal readStatusBytes();
	RetVal isMotorOn();
	RetVal isInHome();
	RetVal isRepeatModeOn();
	RetVal isHoldOn();
	RetVal isCycleOn();
	RetVal isEstopOn();
	RetVal isErrorOn();
	RetVal isReadyOn();
	RetVal isExternalHoldOn();
	RetVal isSafetyFenceOn();
	RetVal isRepeatContOn();
	RetVal isStepOnceOn();
	RetVal isStepWaitingOn();
	RetVal isDryRunOn();
	RetVal isZeroingDone();
	RetVal isInMotion();
	RetVal isIOActive();
	RetVal isModbusConnected();
	RetVal isCollisionDetectionActive();
	RetVal isZeroingRunning();
	RetVal isTeachMotionActive();
	RetVal isMotionCommand();
	RetVal readSelectedTeachMotionMode();
	RetVal readSelectedToolNumber();
	RetVal readSelectedTeachSpeed();
	RetVal isHallSensorOn(int jt);
	RetVal JT();
	RetVal JT_Velocity();
	RetVal Pose();
	RetVal TCP_Velocity();
	RetVal cpuTemp();
	RetVal readMonitorSpeed();
	RetVal readErrorCode();
	RetVal serialNumber();
	RetVal firmwareVersion(); 
	//TransformationPoints readTransPointsData();
	//JointPoints readJointPointsData();
	byte setUartTimeout(int timeout);
	byte setHome(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6, double jt7);
	byte setTool(int index);
	byte setWork(int index);
	byte setHomeHere();
	byte setMonitorSpeed(int speed);
	byte setHome(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6);
	byte setHome(double* jt, int length);
	byte saveCurrentJointsAsPoint(int point);
	byte saveCurrentPoseAsPoint(int point);
	byte setMainProgram(const char* name);
	byte selectProgram(const char* name);
	RetVal readProgramsName();
	RetVal mainProgram();
	RetVal selectedProgram();
	byte sendToolData(int index, double* data, int length);
	byte removePoint(int number, int type);
	RetVal readToolData(int index);
	RetVal readTransformationPoint(int index);
	RetVal readJointPoint(int index);
	byte Zero();
	byte JMOVE(byte pointType, byte pointIndex, byte spd, byte acc, byte dec);
	byte JMOVE(byte pointType, byte spd, byte acc, byte dec, double* target, int length);
	byte LMOVE(byte pointType, byte pointIndex, byte spd, byte acc, byte dec);
	byte LMOVE(byte pointType, byte spd, byte acc, byte dec, double* target, int length);
	byte HOME(byte spd, byte acc, byte dec);
	byte CMOVE(byte pointType, byte spd, byte acc, byte dec, double* middle, double* target, int length);
	byte CMOVE(byte pointType1, byte pointIndex1, byte pointIndex2, byte spd, byte acc, byte dec);
	byte JAPPRO(byte pointType, byte pointIndex, byte spd, byte acc, byte dec, double offset);
	byte LAPPRO(byte pointType, byte pointIndex, byte spd, byte acc, byte dec, double offset);
	byte LAPPRO(byte pointType, byte spd, byte acc, byte dec, double* target, int length, double offset);
	byte JAPPRO(byte pointType, byte spd, byte acc, byte dec, double* target, int length, double offset);
	byte RTC_ON();
	byte RTC_OFF();
	byte setRTC_OffsetData(double x, double y, double z, double rx, double ry, double rz);
	byte setRTC_OffsetData(double* offset, size_t f_size);
	byte RTC_move(byte type, byte period, double* target, size_t f_size); 
	byte executeASCommand(String command);

};
