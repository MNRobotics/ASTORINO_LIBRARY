#include "astorino.h"

astorino::astorino(HardwareSerial& port) : serialPort(&port) {}

void astorino::begin(unsigned long baud)
{
    if (serialPort != nullptr) 
    {
        serialPort->begin(baud);
    }
}
int astorino::available() 
{
    if (serialPort != nullptr)
    {
        return serialPort->available();
    }
    return 0;
}

// Metoda read
int astorino::read() {
    if (serialPort != nullptr) 
    {
        return serialPort->read();
    }
    return -1; // Brak danych do odczytu
}

// Metody write
size_t astorino::write(const uint8_t* buffer, size_t size) {

    if (serialPort != nullptr)
    {
        return serialPort->write(buffer, size);
    }
    return 0;
}

size_t astorino::write(uint8_t data) {

    if (serialPort != nullptr) 
    {
        return serialPort->write(data);
    }
    return 0;
}

byte astorino::getCRC(byte* buffer, byte size)
{
	byte i;
	uint16_t sum = 0;

	for (i = 0; i < size; i++)
		sum += buffer[i];
	byte ret = (byte)sum;
	return (byte)(ret & 0xFF);
}

bool astorino::sendData(byte* data, int length)
{
	if(!isConnected)
		return false;
	if (serialPort != nullptr) 
    {
        serialPort->write(data, length);
        serialPort->flush();
        return true;
    }
    return false;
    return true;
}

void astorino::waitForData()
{
  uint32_t timeOut = 100;
  uint32_t timeNow = micros();
  while(available() < 1)
  {
    if(micros() - timeNow > timeOut)
      return;
  }
  return;
}

byte* astorino::WaitForResponse(byte* startBytes, uint32_t timeout, size_t &frameSize)
{
	unsigned long startTime = millis();
    size_t f_size; 
	while ((millis() - startTime) < timeout) 
	{
		if (available() > 0) 
		{
			while (available() > 0) 
			{
				int data = read();
				if (data == -1)
					break;

				if (_bufferIndex < BUFFER_SIZE) 
				{
					_buffer[_bufferIndex] = (byte)data;
					_bufferIndex++;
				}
       
				waitForData();
			}
			byte* frame = ExtractFrame(startBytes, f_size);
			if (frame != NULL) 
			{
				frameSize = f_size;
				return frame;
			}
		}
	}
  
	static byte frame[5] = {0x01, 0x02, 0xCC, 0x20, 0xEF};
    frameSize = sizeof(frame);
	return frame;
}

 byte astorino::ValidateResponse(byte* buffer, int length) 
 {
	byte inCRC = buffer[length - 1];
	byte calcCRC = getCRC(buffer, length - 1);
	if (inCRC == calcCRC)
		return 0x00;
	else 
	{
		errorCode = 0x01;
		return 0x01;
	}
}

bool astorino::isLittleEndian() 
{
    int num = 1;
    return *(char*)&num == 1;
}

void astorino::reverseBytes(byte* data, int length) 
{
    for (int i = 0; i < length / 2; i++) 
    {
        byte temp = data[i];
        data[i] = data[length - i - 1];
        data[length - i - 1] = temp;
    }
}

int32_t astorino::convertToInt32(byte* data) 
{
    int32_t value;
    memcpy(&value, data, sizeof(value));
    return value;
}

void astorino::addProgramName(String nma) 
{
    bool found = false;
    
    // Sprawdzenie, czy program już istnieje w tablicy
    for (int i = 0; i < programsCount; i++) {
        if (programsName[i] == nma) {
            found = true;
            break;
        }
    }
    
    // Dodanie nowej nazwy programu, jeśli nie istnieje
    if (!found && programsCount < MAX_PROGRAMS) {
        programsName[programsCount] = nma;
        programsCount++;
    }
}

int astorino::DecodeResponse(byte* buffer) 
{
    int i = 0;
    switch (buffer[2]) {
        case ACK:
            errorCode = 0x00;
            return 1;
        case NACK:
            errorCode = buffer[3];
            return -1;
        case MC:
            return 2;
        case 0x27:
            status[0] = buffer[3];
            status[1] = buffer[4];
            status[2] = buffer[5];
            status[3] = buffer[6];
            status[4] = buffer[7];
            break;
        case 0x26:
            for (i = 0; i < 18; i++) {
                IO[i] = buffer[i + 3];
            }
            break;
        case 0x28:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                joints[i] = temp / 1000.0;
            }
            break;
        case 0x29:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                pose[i] = temp / 1000.0;
            }
            break;
        case 0x2A:
            sn = ExtractString(buffer, 3);
            break;
        case 0x38:
            frmV = ExtractString(buffer, 3);
            break;
        case 0x46:
            nm = ExtractString(buffer, 3);
            addProgramName(nm);
            break;
        case 0x47:
            mP = ExtractString(buffer, 3);
            break;
        case 0x57:
            sP = ExtractString(buffer, 3);
            break;
        case 0x3D:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[4 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                transPoints[buffer[3]][i] = temp / 1000.0;
            }
            transPoints[buffer[3]][7] = 1.0;
            break;
        case 0x3E:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[4 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                jointPoints[buffer[3]][i] = temp / 1000.0;
            }
            jointPoints[buffer[3]][7] = 1.0;
            break;
        case 0x56:
            for (i = 0; i < 6; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[4 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                if (buffer[3] == 1)
                    Tool1[i] = temp / 1000.0;
                else if (buffer[3] == 2)
                    Tool2[i] = temp / 1000.0;
                else if (buffer[3] == 3)
                    Tool3[i] = temp / 1000.0;
            }
            break;
        case 0x4A:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                home[i] = temp / 1000.0;
            }
            break;
        case 0x5E:
            rS = (int)buffer[3];
            break;
        case 0x54:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                forw[i] = temp / 1000.0;
            }
            break;
        case 0x55:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                inv[i] = temp / 1000.0;
            }
            break;
        case 0x58:
            for (i = 0; i < 3; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                oat[i] = temp / 1000.0;
            }
            break;
        case 0x59:
            for (i = 0; i < 3; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                euler[i] = temp / 1000.0;
            }
            break;
        case 0x5B:
            cpu_temp = buffer[3];
            break;
        case 0x5F:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[4 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                transPoints[buffer[3]][i] = temp / 1000.0;
            }
            transPoints[buffer[3]][7] = 1.0;
            break;
        case 0x60:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[4 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                jointPoints[buffer[3]][i] = temp / 1000.0;
            }
            jointPoints[buffer[3]][7] = 1.0;
            break;
        case 0x62:
            errCode = buffer[3];
            break;
        case 0x66:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                abs_vel[i] = temp / 1000.0;
            }
            break;
        case 0x67:
            for (i = 0; i < 7; i++) {
                byte dataBytes[4];
                memcpy(dataBytes, &buffer[3 + i * 4], 4);
                if (isLittleEndian()) {
                    reverseBytes(dataBytes, 4);
                }
                double temp = (double)convertToInt32(dataBytes);
                jt_vel[i] = temp / 1000.0;
            }
            break;
        default:
            return 0;
    }
    return 0;
}

int astorino::BytesToInt32(byte* bytes) 
{
    return ((int)bytes[0] << 24) | ((int)bytes[1] << 16) | ((int)bytes[2] << 8) | (int)bytes[3];
}


int astorino::ConvertBitsToInt(bool bit0, bool bit1, bool bit2)
{
	int result = 0;

	if (bit0)
		result |= 1 << 0;      

	if (bit1)
		result |= 1 << 1;    

	if (bit2)
		result |= 1 << 2;

	return result;
}

byte* astorino::ExtractFrame(byte* startBytes, size_t &frameSize) 
{
    int startIndex = FindStartBytes(startBytes);
    if (startIndex >= 0) 
	{
        int frameEndIndex = _bufferIndex; // Zakładamy, że ramka kończy się na końcu bufora

        // Szukamy kolejnego wystąpienia startBytes, aby określić koniec ramki
        for (int i = startIndex + sizeof(startBytes); i <= _bufferIndex - sizeof(startBytes); i++) 
		{
            if (IsStartBytes(i, startBytes)) 
			{
                frameEndIndex = i;
                break;
            }
        }
        int frameLength = frameEndIndex - startIndex;
        static byte frame[BUFFER_SIZE]; // Statyczna tablica na ramkę
        memcpy(frame, _buffer + startIndex, frameLength);
        // Przesunięcie pozostałych danych w buforze
        _bufferIndex -= frameEndIndex;
        memmove(_buffer, _buffer + frameEndIndex, _bufferIndex);
        frameSize = frameLength;
        return frame;
    }

    return NULL;
}

 bool astorino::IsStartBytes(int index, byte* startBytes) 
 {
	if (index + 2 > sizeof(_buffer))
		return false;

	for (int i = 0; i < 2; i++) 
	{
		if (_buffer[index + i] != startBytes[i])
			return false;
	}

	return true;
}

int astorino::FindStartBytes(byte* startBytes) 
{
	for (int i = 0; i <= _bufferIndex - 2; i++) 
	{
		if (IsStartBytes(i, startBytes))
			return i;
	}
	return -1;
}

byte astorino::sendCommand(byte cmd) 
{
    byte ret = 0;
    byte data[4] = { 0x01, 0x02, cmd, 0x00 };
    data[3] = getCRC(data, 3);
    size_t frameSize;
    
    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) 
		{
            if (DecodeResponse(res) >= 0) 
			{
                return 0;
            } 
			else 
			{
                return errorCode;
            }
        }
        return ret;
    } 
	else 
	{
        errorCode = 0xFE;
        return 0xFE;
    }
}

byte astorino::sendCommandMotion(byte cmd) 
{
	byte rn = 0;
	byte data[4] = {0x01, 0x02, cmd, 0x00};
	data[3] = getCRC(data, 3);
	int ret = 0;
	size_t frameSize;
	if (sendData(data, 4)) 
	{
	    byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
		rn = ValidateResponse(res, frameSize);
		if (rn == 0) 
		{
			ret = DecodeResponse(res);
			if (ret == 2)
				return 0;
			if (ret == -1)
				return errorCode;
		}
		else 
		{
			return rn;
		}
	}
	else 
	{
		return 0xFE;
	}
    return 0xFF;
}

byte astorino::sendCommandMotion(byte cmd, byte spd, byte acc, byte dec) 
{
    byte rn = 0;
    byte data[7] = { 0x01, 0x02, cmd, spd, acc, dec, 0x00 };
    data[6] = getCRC(data, 6);
    int ret = 0;
    size_t frameSize;
    
    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT,frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) 
		{
            ret = DecodeResponse(res);
            if (ret == 2) 
			{
                return 0;
            } 
			else if (ret == -1) 
			{
                return errorCode;
            } 
			else 
			{
                return 0;
            }
        } 
		else 
		{
            return rn;
        }
    } 
	else 
	{
        return 0xFE;
    }
    return 0xFF; // This should not be reached, added to avoid compiler warning
}

String astorino::ExtractString(byte* byteArray, int startIndex) 
{
    int endIndex = -1;

    // Szukamy końca ciągu znaków (0x03)
    for (int i = startIndex; i < 255; i++) 
	{
        if (byteArray[i] == 0x03) {
            endIndex = i;
            break;
        }
    }

    if (endIndex == -1) 
	{
        return "";
    }

    char stringBytes[255];
    int length = endIndex - startIndex;
    memcpy(stringBytes, byteArray + startIndex, length);
    stringBytes[length] = '\0'; // Dodanie znaku końca ciągu

    return String(stringBytes);
}

bool astorino::IsBitSet(byte b, int pos)
{
	return (b & (1 << pos)) != 0;
}

bool astorino::SendFrame(String data, byte cmd) 
{
    byte startBytes[] = { 0x01, 0x02, cmd };
    int dataLength = data.length();
    byte dataBytes[dataLength];
    data.getBytes(dataBytes, dataLength + 1);
    byte endByte[] = { 0x03 };
    byte CRC[] = { 0x00 };

    int frameLength = sizeof(startBytes) + dataLength + sizeof(endByte) + sizeof(CRC);
    byte frame[frameLength];

    // Kopiowanie startBytes do frame
    memcpy(frame, startBytes, sizeof(startBytes));

    // Kopiowanie dataBytes do frame
    memcpy(frame + sizeof(startBytes), dataBytes, dataLength);

    // Kopiowanie endByte do frame
    memcpy(frame + sizeof(startBytes) + dataLength, endByte, sizeof(endByte));

    // Obliczanie CRC i dodanie do frame
    frame[frameLength - 1] = getCRC(frame, frameLength - 1);

    return sendData(frame, frameLength);
}

bool astorino::SendData(byte index, byte cmd, double* values, int length) 
{
    if (length < 6) 
	{
        return false;
    }

    byte startBytes[] = { 0x01, 0x02, cmd };
    byte indexByte[] = { index };
    int len = length;
    byte dataBytes[len * 4]; // 7 wartości double przekształcone na int32, każda int32 to 4 bajty

    for (int i = 0; i < len; i++) 
	{
        int intValue = (int)(values[i] * 1000); // Konwersja na int32 po pomnożeniu przez 1000
        byte intBytes[4];
        intBytes[0] = (byte)((intValue >> 24) & 0xFF);
        intBytes[1] = (byte)((intValue >> 16) & 0xFF);
        intBytes[2] = (byte)((intValue >> 8) & 0xFF);
        intBytes[3] = (byte)(intValue & 0xFF);
        memcpy(dataBytes + i * 4, intBytes, 4); // Upewnij się, że bajty są w odpowiedniej kolejności (big-endian)
    }

    int frameLength = sizeof(startBytes) + sizeof(indexByte) + sizeof(dataBytes);
    byte frame[frameLength + 1]; // +1 na CRC

    memcpy(frame, startBytes, sizeof(startBytes));
    memcpy(frame + sizeof(startBytes), indexByte, sizeof(indexByte));
    memcpy(frame + sizeof(startBytes) + sizeof(indexByte), dataBytes, sizeof(dataBytes));

    // Obliczanie CRC
    byte crc = getCRC(frame, frameLength);
    frame[frameLength] = crc; // Dodaj CRC na koniec ramki

    return sendData(frame, frameLength + 1);
}

String astorino::getLibVersion()
{
	return _version;
}

byte astorino::Connect()
{
	if(isConnected)
		return 0xFE;
	
	begin(115200);
	isConnected = true;
	return sendCommand(0x19);
}

byte astorino::Disconnect()
{
	if (sendCommand(0x25) == 0)
	{
		isConnected = false;
		return 0;
	}
	else
		return errorCode;
}

void astorino::setTimeOut(int timeOut)
{
	TIMEOUT = timeOut;
}

void astorino::setMotionTimeOut(long timeOut)
{
	MOTION_TIMEOUT = timeOut;
}

byte astorino::cancelMotion()
{
	return sendCommand(0x45);
}

byte astorino::emergencyStop()
{
	return sendCommand(0x5A);
}

astorino::RetVal astorino::ForwardKinematics(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6, double jt7) 
{
    byte ret = 0;
    int32_t pose = 0;
    double jt[7] = { jt1, jt2, jt3, jt4, jt5, jt6, jt7 };
    byte data[32];
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x54;

    for (int i = 0; i < 7; i++) 
	{
        pose = (int32_t)(jt[i] * 1000.0);

        data[3 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[4 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[5 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[6 + i * 4] = (byte)(pose & 0xFF);
    }

    data[31] = getCRC(data, 31);

    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) 
		{
            if (DecodeResponse(res) >= 0) 
			{
                return RetVal(forw, 0);
            } 
			else 
			{
                return RetVal(errorCode);
            }
        }
        return RetVal(errorCode);
    } 
	else 
	{
        return RetVal(0xFE);
    }
    return RetVal(0xFF); // This should not be reached, added to avoid compiler warning
}

astorino::RetVal astorino::ForwardKinematics(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6)
{
	return ForwardKinematics(jt1, jt2, jt3, jt4, jt5, jt6, 0);
}

astorino::RetVal astorino::ForwardKinematics(double* jt, int length) 
{
    if (length == 6) 
	{
        return ForwardKinematics(jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], 0);
    } 
	else if (length == 7) 
	{
        return ForwardKinematics(jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], jt[6]);
    }
    return RetVal(0x22);
}

astorino::RetVal astorino::InverseKinematics(double x, double y, double z, double o, double a, double t, double jt7) 
{
    byte ret = 0;
    int32_t pose = 0;
    double xyz[7] = { x, y, z, o, a, t, jt7 };
    byte data[32];
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x55;

    for (int i = 0; i < 7; i++) 
	{
        pose = (int32_t)(xyz[i] * 1000.0);

        data[3 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[4 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[5 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[6 + i * 4] = (byte)(pose & 0xFF);
    }

    data[31] = getCRC(data, 31);

    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) 
		{
            if (DecodeResponse(res) >= 0) 
			{
                return RetVal(inv, 0);
            } 
			else 
			{
                return RetVal(errorCode);
            }
        }
        return RetVal(errorCode);
    } 
	else 
	{
        return RetVal(0xFE);
    }
    return RetVal(0xFF); // This should not be reached, added to avoid compiler warning
}

astorino::RetVal astorino::InverseKinematics(double x, double y, double z, double o, double a, double t)
{
	return InverseKinematics(x, y, z, o, a, t, 0);
}

astorino::RetVal astorino::InverseKinematics(double* pose, int length) 
{
    if (length == 6) 
	{
        return InverseKinematics(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], 0.0);
    } 
	else if (length == 7) 
	{
        return InverseKinematics(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
    }
    return RetVal(0x22);
}

astorino::RetVal astorino::toOAT(double rx, double ry, double rz) 
{
    byte ret = 0;
    int32_t pose = 0;
    double jt[3] = { rx, ry, rz };
    byte data[16];
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x58;

    for (int i = 0; i < 3; i++) 
	{
        pose = (int32_t)(jt[i] * 1000.0);

        data[3 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[4 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[5 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[6 + i * 4] = (byte)(pose & 0xFF);
    }

    data[15] = getCRC(data, 15);

    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) 
		{
            if (DecodeResponse(res) >= 0) 
			{
                return RetVal(oat, 0);
            } 
			else 
			{
                return RetVal(errorCode);
            }
        }
        return RetVal(errorCode);
    } 
	else 
	{
        return RetVal(0xFE);
    }
    return RetVal(0xFF); // This should not be reached, added to avoid compiler warning
}

astorino::RetVal astorino::toRPY(double o, double a, double t) 
{
    byte ret = 0;
    int32_t pose = 0;
    double jt[3] = { o, a, t };
    byte data[16];
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x59;

    for (int i = 0; i < 3; i++) 
	{
        pose = (int32_t)(jt[i] * 1000.0);

        data[3 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[4 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[5 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[6 + i * 4] = (byte)(pose & 0xFF);
    }

    data[15] = getCRC(data, 15);

    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) 
			{
                return RetVal(euler, 0);
            } 
			else 
			{
                return RetVal(errorCode);
            }
        }
        return RetVal(errorCode);
    } 
	else 
	{
        return RetVal(0xFE);
    }
    return RetVal(0xFF); // This should not be reached, added to avoid compiler warning
}

byte astorino::setMotorOn()
{
	return sendCommand(0x20);
}

byte astorino::setMotorOff() 
{
    return sendCommandMotion(0x21);
}

byte astorino::reset()
{
	errorCode = 0x00;
	return sendCommand(0x22);
}

byte astorino::Hold()
{
	return sendCommand(0x30);
}

byte astorino::Run() 
{
    return sendCommand(0x31);
}

byte astorino::turnDryRunOn() 
{
    return sendCommand(0x32);
}

byte astorino::turnDryRunOff() 
{
    return sendCommand(0x33);
}

byte astorino::setRepeatCont() 
{
    return sendCommand(0x34);
}

byte astorino::setRepeatOnce() 
{
    return sendCommand(0x35);
}

byte astorino::setStepOnce() 
{
    return sendCommand(0x36);
}

byte astorino::setStepCont() 
{
    return sendCommand(0x37);
}

byte astorino::cycleStop() 
{
    return sendCommand(0x3C);
}

byte astorino::cycleStart() 
{
    return sendCommand(0x2B);
}

byte astorino::nextStep() 
{
    return sendCommand(0x6C);
}

byte astorino::skipWait() 
{
    return sendCommand(0x6D);
}

astorino::RetVal astorino::readInput(int ind) 
{
    if (ind <= 0 || ind > 58) 
	{
        return RetVal(0x08);
    }

    ind = ind - 1;
    int count = ind / 8;

    if (sendCommand(0x26) == 0) 
	{
        if (IsBitSet(IO[count], ind % 8)) 
		{
            return RetVal(1, 0);
        } 
		else 
		{
            return RetVal(-1, 0);
        }
    } 
	else 
	{
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readOutput(int ind) 
{
    if (ind <= 0 || ind > 58) 
	{
        return RetVal(0x08);
    }

    ind = ind - 1;
    int count = ind / 8;

    if (sendCommand(0x26) == 0) 
	{
        if (IsBitSet(IO[count + 8], ind % 8)) 
		{
            return RetVal(1, 0);
        } 
		else 
		{
            return RetVal(-1, 0);
        }
    } 
	else 
	{
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readInternal(int ind) 
{
    if (ind <= 0 || ind > 16) 
	{
        return RetVal(0x08);
    }

    if (sendCommand(0x26) == 0) 
	{
        if (ind < 9) 
		{
            if (IsBitSet(IO[17], ind - 1)) 
			{
                return RetVal(1, 0);
            } 
			else 
			{
                return RetVal(-1, 0);
            }
        } 
		else 
		{
            if (IsBitSet(IO[16], ind - 9)) 
			{
                return RetVal(1, 0);
            } 
			else 
			{
                return RetVal(-1, 0);
            }
        }
    } 
	else 
	{
        return RetVal(errorCode);
    }
}

byte astorino::setOutput(int output, int state) 
{
    byte ret = 0;
    size_t frameSize;
    if (state == -1) 
	{
        state = 0;
    }
    byte data[6] = { 0x01, 0x02, 0x3A, (byte)output, (byte)state, 0x00 };
    data[5] = getCRC(data, 5);

    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) 
			{
                return ret;
            } 
			else 
			{
                return errorCode;
            }
        }
        return ret;
    } 
	else 
	{
        return 0xFE;
    }
    return 0xFF; // This should not be reached, added to avoid compiler warning
}

byte astorino::setInternal(int sig, int state) 
{
    byte ret = 0;
    size_t frameSize;
    if (state == -1) 
	{
        state = 0;
    }
    byte data[6] = { 0x01, 0x02, 0x4C, (byte)sig, (byte)state, 0x00 };
    data[5] = getCRC(data, 5);

    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) 
		{
            if (DecodeResponse(res) >= 0) 
			{
                return ret;
            } 
			else 
			{
                return errorCode;
            }
        }
        return ret;
    } 
	else 
	{
        return 0xFE;
    }
    return 0xFF; // This should not be reached, added to avoid compiler warning
}

astorino::RetVal astorino::readStatusBytes() 
{
    if (sendCommand(0x27) == 0) {
        return RetVal(status, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isMotorOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 6)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isInHome() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 7)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isRepeatModeOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 5)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isHoldOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 4)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isCycleOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 3)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isEstopOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 2)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isErrorOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 1)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isReadyOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[0], 0)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isExternalHoldOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 7)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isSafetyFenceOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 6)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isRepeatContOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 5)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isStepOnceOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 4)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isStepWaitingOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 3)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isDryRunOn() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 2)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isZeroingDone() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 1)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isInMotion() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[1], 0)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isIOActive() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[2], 7)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isModbusConnected() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[3], 1)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isCollisionDetectionActive() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[3], 0)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isZeroingRunning()
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[4], 6)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isTeachMotionActive() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[4], 5)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isMotionCommand() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[4], 4)) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readSelectedTeachMotionMode() 
{
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[4], 0)) {
            return RetVal(1, 0); // tool
        }
        if (IsBitSet(status[4], 1)) {
            return RetVal(2, 0); // joint
        }
        if (IsBitSet(status[4], 2)) {
            return RetVal(3, 0); // conv
        }
        if (IsBitSet(status[4], 3)) {
            return RetVal(4, 0); // base
        }
        if (IsBitSet(status[4], 7)) {
            return RetVal(5, 0); // work
        }
        return RetVal(-1, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readSelectedToolNumber() 
{
    if (sendCommand(0x27) == 0) {
        int num = ConvertBitsToInt(IsBitSet(status[3], 5), IsBitSet(status[3], 6), IsBitSet(status[3], 7)) + 1;
        return RetVal(num, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readSelectedTeachSpeed() 
{
    if (sendCommand(0x27) == 0) {
        int num = ConvertBitsToInt(IsBitSet(status[3], 2), IsBitSet(status[3], 3), IsBitSet(status[3], 4)) + 1;
        return RetVal(num, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::isHallSensorOn(int jt) 
{
    if (jt < 1 || jt > 7) {
        return RetVal(0x08);
    }
    if (sendCommand(0x27) == 0) {
        if (IsBitSet(status[2], (jt - 1))) {
            return RetVal(1, 0);
        } else {
            return RetVal(-1, 0);
        }
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::JT() 
{
    if (sendCommand(0x28) == 0) {
        return RetVal(joints, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::JT_Velocity() 
{
    if (sendCommand(0x67) == 0) {
        return RetVal(jt_vel, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::Pose() 
{
    if (sendCommand(0x29) == 0) {
        return RetVal(pose, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::TCP_Velocity() 
{
    if (sendCommand(0x66) == 0) {
        return RetVal(abs_vel, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::cpuTemp() 
{
    if (sendCommand(0x5B) == 0) {
        return RetVal(cpu_temp, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readMonitorSpeed() 
{
    if (sendCommand(0x5E) == 0) {
        return RetVal(rS, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::readErrorCode() 
{
    if (sendCommand(0x62) == 0) {
        return RetVal(errCode, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::serialNumber() {
    if (sendCommand(0x2A) == 0) {
        return RetVal(sn, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::firmwareVersion() {
    if (sendCommand(0x38) == 0) {
        return RetVal(frmV, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::TransformationPoints astorino::readTransPointsData() 
{
    byte ret = 0;
    int rn = 0;
    byte data[4] = { 0x01, 0x02, 0x3D, 0x00 };
    data[3] = getCRC(data, 3);
    if (sendData(data, sizeof(data))) {
        do {
            byte* res = WaitForResponse(startBytes, TIMEOUT);
            ret = ValidateResponse(res, sizeof(res));
            if (ret == 0) {
                rn = DecodeResponse(res);
                if (rn == 1) {
                    break;
                }
                if (rn == -1) {
                    return TransformationPoints(errorCode);
                }
            } else {
                return TransformationPoints(errorCode);
            }
        } while (true);
    } else {
        return TransformationPoints(errorCode);
    }

    return TransformationPoints(transPoints, 0);
}

astorino::JointPoints astorino::readJointPointsData() 
{
    byte ret = 0;
    int rn = 0;
    byte data[4] = { 0x01, 0x02, 0x3E, 0x00 };
    data[3] = getCRC(data, 3);
    if (sendData(data, sizeof(data))) {
        do {
            byte* res = WaitForResponse(startBytes, TIMEOUT);
            ret = ValidateResponse(res, sizeof(res));
            if (ret == 0) {
                rn = DecodeResponse(res);
                if (rn == 1) {
                    break;
                }
                if (rn == -1) {
                    return JointPoints(errorCode);
                }
            } else {
                return JointPoints(errorCode);
            }
        } while (true);
    } else {
        return JointPoints(errorCode);
    }

    return JointPoints(jointPoints, 0);
}

byte astorino::setTool(int index) 
{
    if (index == 1)
        return sendCommand(0x2C);
    else if (index == 2)
        return sendCommand(0x2D);
    else if (index == 3)
        return sendCommand(0x2E);
    else
        return 0x08;
}

byte astorino::setWork(int index) 
{
    if (index == 1)
        return sendCommand(0x63);
    else if (index == 2)
        return sendCommand(0x64);
    else
        return 0x08;
}

byte astorino::setHomeHere() 
{
    return sendCommand(0x5D);
}

byte astorino::setUartTimeout(int timeout)
{
    byte ret = 0;
    size_t frameSize;
    byte data[5] = { 0x01, 0x02, 0x6F, (byte)timeout, 0x00 };
    data[4] = getCRC(data, 4);
    if (sendData(data, sizeof(data))) 
    {
        byte* res = WaitForResponse(startBytes, TIMEOUT, frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) {
                return ret;
            }
            else {
                return errorCode;
            }
        }
        return ret;
    }
    else {
        return 0xFE;
    }
}

byte astorino::setMonitorSpeed(int speed) 
{
    byte ret = 0;
    size_t frameSize;
    byte data[5] = { 0x01, 0x02, 0x39, (byte)speed, 0x00 };
    data[4] = getCRC(data, 4);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, TIMEOUT, frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) {
                rS = speed;
                return ret;
            } else {
                return errorCode;
            }
        }
        return ret;
    } else {
        return 0xFE;
    }
}

byte astorino::setHome(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6, double jt7) 
{
    byte ret = 0;
    int32_t pose = 0;
    double jt[7] = { jt1, jt2, jt3, jt4, jt5, jt6, jt7 };
    byte data[32];
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x23;
    for (int i = 0; i < 7; i++) {
        pose = (int32_t)(jt[i] * 1000.0);

        data[3 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[4 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[5 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[6 + i * 4] = (byte)(pose & 0xFF);
    }

    data[31] = getCRC(data, 31);
    if (sendData(data, sizeof(data))) 
	{
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) {
                return ret;
            } else {
                return errorCode;
            }
        }
        return ret;
    } else {
        return 0xFE;
    }
}

byte astorino::setHome(double jt1, double jt2, double jt3, double jt4, double jt5, double jt6) 
{
    return setHome(jt1, jt2, jt3, jt4, jt5, jt6, 0.0);
}

byte astorino::setHome(double* jt, int length) 
{
    if (length == 6)
        return setHome(jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], 0.0);
    if (length == 7)
        return setHome(jt[0], jt[1], jt[2], jt[3], jt[4], jt[5], jt[6]);

    return 0x22;
}

byte astorino::saveCurrentJointsAsPoint(int point) 
{
    double joint[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    int i = 0;
    byte ret = 0;
    size_t frameSize;
    RetVal temp;
    if (point < 0) return 0x08;
    if (point > 99) return 0x02;
    temp = JT();
    if (temp.returnCode == 0) {
        for (i = 0; i < 7; i++)
            jointPoints[point][i] = temp.values[i];
        byte data[5] = { 0x01, 0x02, 0x3F, (byte)point, 0x00 };
        data[4] = getCRC(data, 4);
        if (sendData(data, sizeof(data))) {
            byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
            ret = ValidateResponse(res, frameSize);
            if (ret == 0) {
                if (DecodeResponse(res) >= 0)
                    return ret;
                else
                    return errorCode;
            }
            return ret;
        } else {
            return 0xFE;
        }
    } else {
        return temp.returnCode;
    }
}

byte astorino::saveCurrentPoseAsPoint(int point) 
{
    double pose[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    int i = 0;
    byte ret = 0;
    size_t frameSize;
    RetVal temp;
    temp = Pose();
    if (temp.returnCode == 0) {
        for (i = 0; i < 7; i++)
            transPoints[point][i] = temp.values[i];
        byte data[5] = { 0x01, 0x02, 0x41, (byte)point, 0x00 };
        data[4] = getCRC(data, 4);
        if (sendData(data, sizeof(data))) {
            byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
            ret = ValidateResponse(res, frameSize);
            if (ret == 0) {
                if (DecodeResponse(res) >= 0)
                    return ret;
                else
                    return errorCode;
            }
            return ret;
        } else {
            return 0xFE;
        }
    } else {
        return temp.returnCode;
    }
}

byte astorino::setMainProgram(const char* name) 
{
    byte ret = 0;
    size_t frameSize;
    if (SendFrame(name, 0x43)) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0)
                return ret;
            else
                return errorCode;
        }
        return ret;
    } else {
        return 0xFE;
    }
}

byte astorino::selectProgram(const char* name) 
{
    byte ret = 0;
    size_t frameSize;
    if (SendFrame(name, 0x44)) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0)
                return ret;
            else
                return errorCode;
        }
        return ret;
    } else {
        return 0xFE;
    }
}

astorino::RetVal astorino::readProgramsName() 
{
    byte ret = 0;
    int rn = 0;
    size_t frameSize;
    byte data[4] = { 0x01, 0x02, 0x46, 0x00 };
    data[3] = getCRC(data, 3);
    if (sendData(data, sizeof(data))) {
        do {
            byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
            ret = ValidateResponse(res, frameSize);
            if (ret == 0) {
                rn = DecodeResponse(res);
                if (rn == 1)
                    break;
                if (rn == -1)
                    return RetVal(errorCode);
            } else {
                return RetVal(errorCode);
            }
        } while (true);

        return RetVal(programsName, 0);
    } else {
        return RetVal(0xFE);
    }
}

astorino::RetVal astorino::mainProgram() 
{
    if (sendCommand(0x47) == 0) {
        return RetVal(mP, 0);
    } else {
        return RetVal(errorCode);
    }
}

astorino::RetVal astorino::selectedProgram() 
{
    if (sendCommand(0x57) == 0) {
        return RetVal(sP, 0);
    } else {
        return RetVal(errorCode);
    }
}

byte astorino::sendToolData(int index, double* data, int length) 
{
    byte ret = 0;
    size_t frameSize;
    if (SendData((byte)index, 0x48, data, length)) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0)
                return ret;
            else
                return errorCode;
        }
        return ret;
    } else {
        return 0xFE;
    }
}

byte astorino::removePoint(int number, int type) 
{
    byte ret = 0;
    size_t frameSize;
    byte data[6] = { 0x01, 0x02, 0x49, (byte)number, (byte)type, 0x00 };
    data[5] = getCRC(data, 5);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0)
                return ret;
            else
                return errorCode;
        }
        return ret;
    } else {
        return 0xFE;
    }
}

astorino::RetVal astorino::readToolData(int index) 
{
    byte ret = 0;
    size_t frameSize;
    if (index < 1 || index > 3)
        return RetVal(0x08);

    byte data[5] = { 0x01, 0x02, 0x56, (byte)index, 0x00 };
    data[4] = getCRC(data, 4);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) {
                if (index == 1)
                    return RetVal(Tool1, 0);
                else if (index == 2)
                    return RetVal(Tool2, 0);
                else if (index == 3)
                    return RetVal(Tool3, 0);
                else
                    return RetVal(0x08);
            } else {
                return RetVal(errorCode);
            }
        }
    } else {
        return RetVal(0xFE);
    }

    return RetVal(errorCode);
}

astorino::RetVal astorino::readTransformationPoint(int index) 
{
    byte ret = 0;
    size_t frameSize;
    if (index < 0 || index > 99)
        return RetVal(0x08);

    byte data[5] = { 0x01, 0x02, 0x5F, (byte)index, 0x00 };
    data[4] = getCRC(data, 4);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) {
                double arr[7];
                for (int i = 0; i < 7; i++)
                    arr[i] = transPoints[index][i];
                return RetVal(arr, 0);
            } else {
                return RetVal(errorCode);
            }
        }
    } else {
        return RetVal(0xFE);
    }

    return RetVal(errorCode);
}

astorino::RetVal astorino::readJointPoint(int index) 
{
    byte ret = 0;
    size_t frameSize;
    if (index < 0 || index > 99)
        return RetVal(0x08);

    byte data[5] = { 0x01, 0x02, 0x60, (byte)index, 0x00 };
    data[4] = getCRC(data, 4);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0) {
                double arr[7];
                for (int i = 0; i < 7; i++)
                    arr[i] = jointPoints[index][i];
                return RetVal(arr, 0);
            } else {
                return RetVal(errorCode);
            }
        }
    } else {
        return RetVal(0xFE);
    }

    return RetVal(errorCode);
}

byte astorino::Zero() 
{
    return sendCommandMotion(0x3B);
}

byte astorino::JMOVE(byte pointType, byte pointIndex, byte spd, byte acc, byte dec) 
{
    byte rn = 0;
    byte data[9] = { 0x01, 0x02, 0x4F, pointType, pointIndex, spd, acc, dec, 0x00 };
    data[8] = getCRC(data, 8);
    int ret = 0;
    size_t frameSize;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::JMOVE(byte pointType, byte spd, byte acc, byte dec, double* target, int length) 
{
    byte rn = 0;
    int32_t pose = 0;
    size_t frameSize;
    if (length < 6)
        return 0x22;
    byte data[36] = { 0x01, 0x02, 0x50, pointType, spd, acc, dec, 0x00 };
    for (int i = 0; i < length; i++) {
        pose = (int32_t)(target[i] * 1000.0);
        data[7 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[8 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[9 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[10 + i * 4] = (byte)(pose & 0xFF);
    }
    if (length == 6) {
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        data[34] = 0;
    }
    data[35] = getCRC(data, 35);
    int ret = 0;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::LMOVE(byte pointType, byte pointIndex, byte spd, byte acc, byte dec)
{
    byte rn = 0;
    byte data[9] = { 0x01, 0x02, 0x4D, pointType, pointIndex, spd, acc, dec, 0x00 };
    data[8] = getCRC(data, 8);
    int ret = 0;
    size_t frameSize;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::LMOVE(byte pointType, byte spd, byte acc, byte dec, double* target, int length) 
{
    byte rn = 0;
    int32_t pose = 0;
    size_t frameSize;
    if (length < 6)
        return 0x22;

    byte data[36] = { 0x01, 0x02, 0x4E, pointType, spd, acc, dec, 0x00 };
    for (int i = 0; i < length; i++) {
        pose = (int32_t)(target[i] * 1000.0);
        data[7 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[8 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[9 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[10 + i * 4] = (byte)(pose & 0xFF);
    }
    if (length == 6) {
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        data[34] = 0;
    }
    data[35] = getCRC(data, 35);
    int ret = 0;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::HOME(byte spd, byte acc, byte dec) 
{
    return sendCommandMotion(0x2F, spd, acc, dec);
}

byte astorino::CMOVE(byte pointType, byte spd, byte acc, byte dec, double* middle, double* target, int length)
{
    byte rn = 0;
    int32_t pose = 0;
    size_t frameSize;
    if (length < 6)
        return 0x22;

    byte data[64] = { 0x01, 0x02, 0x61, pointType, spd, acc, dec, 0x00 };
    for (int i = 0; i < length; i++) {
        pose = (int32_t)(middle[i] * 1000.0);
        data[7 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[8 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[9 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[10 + i * 4] = (byte)(pose & 0xFF);
    }
    if (length == 6) {
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        data[34] = 0;
    }
    for (int i = 0; i < length; i++) {
        pose = (int32_t)(target[i] * 1000.0);
        data[35 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[36 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[37 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[38 + i * 4] = (byte)(pose & 0xFF);
    }
    if (length == 6) {
        data[59] = 0;
        data[60] = 0;
        data[61] = 0;
        data[62] = 0;
    }
    data[63] = getCRC(data, 63);
    int ret = 0;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::CMOVE(byte pointType1, byte pointIndex1, byte pointIndex2, byte spd, byte acc, byte dec) 
{
    byte rn = 0;
    byte data[10] = { 0x01, 0x02, 0x65, pointType1, pointIndex1, pointIndex2, spd, acc, dec, 0x00 };
    data[9] = getCRC(data, 9);
    int ret = 0;
    size_t frameSize;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::JAPPRO(byte pointType, byte pointIndex, byte spd, byte acc, byte dec, double offset) 
{
    byte rn = 0;
    byte data[13];
    int ret = 0;
    int32_t pose = (int32_t)(offset * 1000.0);
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x68;
    data[3] = pointType;
    data[4] = pointIndex;
    data[5] = spd;
    data[6] = acc;
    data[7] = dec;

    data[8] = (byte)((pose >> 24) & 0xFF);
    data[9] = (byte)((pose >> 16) & 0xFF);
    data[10] = (byte)((pose >> 8) & 0xFF);
    data[11] = (byte)(pose & 0xFF);

    data[12] = getCRC(data, 12);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT,frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::LAPPRO(byte pointType, byte pointIndex, byte spd, byte acc, byte dec, double offset) 
{
    byte rn = 0;
    byte data[13];
    int ret = 0;
    int32_t pose = (int32_t)(offset * 1000.0);
    size_t frameSize;
    
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x69;
    data[3] = pointType;
    data[4] = pointIndex;
    data[5] = spd;
    data[6] = acc;
    data[7] = dec;

    data[8] = (byte)((pose >> 24) & 0xFF);
    data[9] = (byte)((pose >> 16) & 0xFF);
    data[10] = (byte)((pose >> 8) & 0xFF);
    data[11] = (byte)(pose & 0xFF);

    data[12] = getCRC(data, 12);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::LAPPRO(byte pointType, byte spd, byte acc, byte dec, double* target, int length, double offset) 
{
    byte rn = 0;
    int32_t pose = 0;
    size_t frameSize;
    if (length < 6)
        return 0x22;

    byte data[40];
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x6A;
    data[3] = pointType;
    data[4] = spd;
    data[5] = acc;
    data[6] = dec;

    for (int i = 0; i < length; i++) {
        pose = (int32_t)(target[i] * 1000.0);
        data[7 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[8 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[9 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[10 + i * 4] = (byte)(pose & 0xFF);
    }
    if (length == 6) {
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        data[34] = 0;
    }

    pose = (int32_t)(offset * 1000.0);
    data[35] = (byte)((pose >> 24) & 0xFF);
    data[36] = (byte)((pose >> 16) & 0xFF);
    data[37] = (byte)((pose >> 8) & 0xFF);
    data[38] = (byte)(pose & 0xFF);

    data[39] = getCRC(data, 39);

    int ret = 0;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::JAPPRO(byte pointType, byte spd, byte acc, byte dec, double* target, int length, double offset) 
{
    byte rn = 0;
    int32_t pose = 0;
    size_t frameSize;
    if (length < 6)
        return 0x22;

    byte data[40];
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x6B;
    data[3] = pointType;
    data[4] = spd;
    data[5] = acc;
    data[6] = dec;

    for (int i = 0; i < length; i++) {
        pose = (int32_t)(target[i] * 1000.0);
        data[7 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[8 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[9 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[10 + i * 4] = (byte)(pose & 0xFF);
    }
    if (length == 6) {
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        data[34] = 0;
    }

    pose = (int32_t)(offset * 1000.0);
    data[35] = (byte)((pose >> 24) & 0xFF);
    data[36] = (byte)((pose >> 16) & 0xFF);
    data[37] = (byte)((pose >> 8) & 0xFF);
    data[38] = (byte)(pose & 0xFF);

    data[39] = getCRC(data, 39);

    int ret = 0;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT, frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::RTC_ON() 
{
    return sendCommand(0x51);
}

byte astorino::RTC_OFF() 
{
    return sendCommand(0x52);
}

byte astorino::setRTC_OffsetData(double x, double y, double z, double rx, double ry, double rz) 
{
    byte ret = 0;
    int32_t pose = 0;
    double jt[6] = {x, y, z, rx, ry, rz};
    size_t frameSize;
    byte data[29];
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x53;
    for (int i = 0; i < 6; i++) {
        pose = static_cast<int32_t>(jt[i] * 1000.0);

        data[3 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[4 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[5 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[6 + i * 4] = (byte)(pose & 0xFF);
    }

    data[28] = getCRC(data, 28);
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0)
                return ret;
            else
                return errorCode;
        }
        return ret;
    } else {
        return 0xFE;
    }
}

byte astorino::setRTC_OffsetData(double* offset, size_t f_size) 
{
    if (f_size < 6)
        return 0x22;
    return setRTC_OffsetData(offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]);
}

byte astorino::RTC_move(byte type, byte period, double* target, size_t f_size) 
{
    byte rn = 0;
    int32_t pose = 0;
    int j = f_size;
    if (j < 6)
        return 0x22;

    byte data[34];
    size_t frameSize;
    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x5C;
    data[3] = type;
    data[4] = period;

    for (int i = 0; i < j; i++) {
        pose = static_cast<int32_t>(target[i] * 1000.0);

        data[5 + i * 4] = (byte)((pose >> 24) & 0xFF);
        data[6 + i * 4] = (byte)((pose >> 16) & 0xFF);
        data[7 + i * 4] = (byte)((pose >> 8) & 0xFF);
        data[8 + i * 4] = (byte)(pose & 0xFF);
    }
    if (j == 6) {
        data[29] = 0;
        data[30] = 0;
        data[31] = 0;
        data[32] = 0;
    }
    data[33] = getCRC(data, 33);
    int ret = 0;
    if (sendData(data, sizeof(data))) {
        byte* res = WaitForResponse(startBytes, 2 * static_cast<int>(period), frameSize);
        rn = ValidateResponse(res, frameSize);
        if (rn == 0) {
            ret = DecodeResponse(res);
            if (ret == 2)
                return 0;
            else if (ret == -1)
                return errorCode;
            else
                return 0x10;
        } else {
            return rn;
        }
    } else {
        return 0xFE;
    }
}

byte astorino::executeASCommand(String command) 
{
    byte ret = 0;
    size_t frameSize;
    if (SendFrame(command, 0x4B)) {
        byte* res = WaitForResponse(startBytes, MOTION_TIMEOUT,frameSize);
        ret = ValidateResponse(res, frameSize);
        if (ret == 0) {
            if (DecodeResponse(res) >= 0)
                return ret;
            else
                return errorCode;
        }
        return ret;
    } else {
        return 0xFE;
    }
}
