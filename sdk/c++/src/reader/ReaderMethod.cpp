#include "inc/reader/ReaderMethod.h"
#include "inc/reader/ReaderProcessor.h"
#include "inc/reader/MessageTran.h"
#include <iostream>
#include <cstring>

// ReaderMethod::ReaderMethod()
//     : m_btAryBuffer(0x1000, 0) // Initialize buffer with 0x1000 size
// {
//     m_Processor = new ReaderProcessor(this);
//     AnalyCallback = [this](const MessageTran& msg) { m_Processor->AnalyData(msg); };

//     // Initialize serial port if needed
//     iSerialPort.SetDataReceivedHandler([this]() { ReceivedComData(); });
// }

// ReaderMethod::~ReaderMethod() {
//     delete m_Processor;
// }

int ReaderMethod::CancelAccessEpcMatch(uint8_t btReadId, uint8_t btMode) {
    uint8_t btCmd = 0x85;
    return SendMessage(btReadId, btCmd, {btMode});
}

// uint8_t ReaderMethod::CheckValue(const std::vector<uint8_t>& btAryData) {
//     return MessageTran().CheckSum(btAryData, 0, btAryData.size());
// }

int ReaderMethod::ClearTagMask(uint8_t btReadId, uint8_t btMaskNO) {
    return SendMessage(btReadId, 0x98, {btMaskNO});
}

void ReaderMethod::CloseCom() {
    if (iSerialPort.isOpen()) {
        iSerialPort.close();
    }
    m_nType = -1;
}

int ReaderMethod::Connect(std::istream& otherSrc) {
    
    if (!otherSrc) {
        return -1;
    }
    m_nType = 1;
    return 0;
}

int ReaderMethod::SendMessage(const std::vector<uint8_t>& btArySenderData) {
    if (m_nType == 0 && iSerialPort.isOpen()) {
        iSerialPort.write(btArySenderData.data(), btArySenderData.size());
        if (SendCallback) {
            SendCallback(btArySenderData);
        }
        return 0;
    }
    return -1;
}

// int ReaderMethod::SendMessage(uint8_t btReadId, uint8_t btCmd) {
//     MessageTran tran(btReadId, btCmd);
//     return SendMessage(tran.AryTranData);
// }

// int ReaderMethod::SendMessage(uint8_t btReadId, uint8_t btCmd, const std::vector<uint8_t>& btAryData) {
//     MessageTran tran(btReadId, btCmd, btAryData);
//     return SendMessage(tran.AryTranData);
// }

void ReaderMethod::ReceivedComData() {
    // try {
    //     int bytesToRead = iSerialPort.BytesToRead();
    //     if (bytesToRead > 0) {
    //         std::vector<uint8_t> buffer(bytesToRead);
    //         iSerialPort.Read(buffer.data(), bytesToRead);
    //         RunReceiveDataCallback(buffer);
    //     }
    // } catch (const std::exception& e) {
    //     std::cerr << "Error receiving data: " << e.what() << std::endl;
    // }
}

void ReaderMethod::RunReceiveDataCallback(const std::vector<uint8_t>& btAryReceiveData) {
    if (ReceiveCallback) {
        ReceiveCallback(btAryReceiveData);
    }
    // Process data (simplified for this example)
    m_btAryBuffer.insert(m_btAryBuffer.end(), btAryReceiveData.begin(), btAryReceiveData.end());
    // Additional processing here as needed
}

void ReaderMethod::SetReceiveCallback(ReceiveDataCallback callback) {
    ReceiveCallback = callback;
}

void ReaderMethod::SetSendCallback(SendDataCallback callback) {
    SendCallback = callback;
}

void ReaderMethod::SetAnalyCallback(AnalyDataCallback callback) {
    AnalyCallback = callback;
}