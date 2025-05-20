#ifndef READER_METHOD_H
#define READER_METHOD_H
#include "inc/serial/serial.h"
#include <cstdint>
#include <array>
#include <string>
#include <functional>
#include <vector>


class ReaderProcessor;
class MessageTran;

class ReaderMethod
{
    public:

    serial::Serial iSerialPort;

    using ReceiveDataCallback = std::function<void(const std::vector<uint8_t>&)>;
    using SendDataCallback = std::function<void(const std::vector<uint8_t>&)>;
    using AnalyDataCallback = std::function<void(const MessageTran&)>;
    ReaderMethod(/* args */);
    ~ReaderMethod();


    int CancelAccessEpcMatch(uint8_t btReadId, uint8_t btMode);
    uint8_t CheckValue(const std::vector<uint8_t>& btAryData);
    int ClearTagMask(uint8_t btReadId, uint8_t btMaskNO);
    void CloseCom();
    int Connect(std::istream& otherSrc);
    int ConnectServer(const std::string& ipAddress, int nPort, std::string& strException);
    int CustomizedInventory(uint8_t btReadId, uint8_t session, uint8_t target, uint8_t SL, uint8_t byRound);
    int FastSwitchInventory(uint8_t btReadId, const std::vector<uint8_t>& btAryData);
    int GetAccessEpcMatch(uint8_t btReadId);

    // Define more methods based on your C# code structure here

    void SetReceiveCallback(ReceiveDataCallback callback);
    void SetSendCallback(SendDataCallback callback);
    void SetAnalyCallback(AnalyDataCallback callback);
private:
    int SendMessage(const std::vector<uint8_t>& btArySenderData);
    int SendMessage(uint8_t btReadId, uint8_t btCmd);
    int SendMessage(uint8_t btReadId, uint8_t btCmd, const std::vector<uint8_t>& btAryData);

    void ReceivedComData();
    void RunReceiveDataCallback(const std::vector<uint8_t>& btAryReceiveData);

    // Member variables
    //SerialPort iSerialPort;

    

    int m_nType = -1;
    ReaderProcessor* m_Processor;
    std::vector<uint8_t> m_btAryBuffer;
    int m_nLenth = 0;

    ReceiveDataCallback ReceiveCallback;
    SendDataCallback SendCallback;
    AnalyDataCallback AnalyCallback;

};


#endif // READER_METHOD_H