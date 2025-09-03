#include "inc/reader/MessageTran.h"
#include <algorithm>

MessageTran::MessageTran()
{
}

MessageTran::MessageTran(const std::vector<uint8_t> &btAryTranData)
{
    int length = btAryTranData.size();
    this->btAryTranData = btAryTranData;

    if (this->CheckSum(this->btAryTranData, 0, length - 1) == btAryTranData[length - 1])
    {
        this->btPacketType = btAryTranData[0];
        this->btDataLen = btAryTranData[1];
        this->btReadId = btAryTranData[2];
        this->btCmd = btAryTranData[3];
        this->btCheck = btAryTranData[length - 1];

        if (length > 5)
        {
            this->btAryData = std::vector<uint8_t>(btAryTranData.begin() + 4, btAryTranData.begin() + length - 1);
        }
    }
}

MessageTran::MessageTran(uint8_t btReadId, uint8_t btCmd)
    : btPacketType(160), btDataLen(3), btReadId(btReadId), btCmd(btCmd)
{

    this->btAryTranData = {this->btPacketType, this->btDataLen, this->btReadId, this->btCmd};
    this->btCheck = this->CheckSum(this->btAryTranData, 0, 4);
    this->btAryTranData.push_back(this->btCheck);
}

MessageTran::MessageTran(uint8_t btReadId, uint8_t btCmd, const std::vector<uint8_t> &btAryData)
    : btPacketType(160), btReadId(btReadId), btCmd(btCmd), btAryData(btAryData)
{

    this->btDataLen = static_cast<uint8_t>(btAryData.size() + 3);
    this->btAryTranData = {this->btPacketType, this->btDataLen, this->btReadId, this->btCmd};
    this->btAryTranData.insert(this->btAryTranData.end(), btAryData.begin(), btAryData.end());
    this->btCheck = this->CheckSum(this->btAryTranData, 0, btAryData.size() + 4);
    this->btAryTranData.push_back(this->btCheck);
}

uint8_t MessageTran::CheckSum(const std::vector<uint8_t> &btAryBuffer, int nStartPos, int nLen)
{
    uint8_t sum = 0;
    for (int i = nStartPos; i < nStartPos + nLen; i++)
    {
        sum += btAryBuffer[i];
    }
    return static_cast<uint8_t>((~sum + 1) & 0xff);
}

const std::vector<uint8_t> &MessageTran::AryTranData() const
{
    return this->btAryTranData;
}

const std::vector<uint8_t> &MessageTran::AryData() const
{
    return this->btAryData;
}

uint8_t MessageTran::ReadId() const
{
    return this->btReadId;
}

uint8_t MessageTran::Cmd() const
{
    return this->btCmd;
}

uint8_t MessageTran::PacketType() const
{
    return this->btPacketType;
}
