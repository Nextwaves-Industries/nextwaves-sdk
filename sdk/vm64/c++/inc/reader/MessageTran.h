#ifndef MESSAGETRAN_H
#define MESSAGETRAN_H

#include <vector>

class MessageTran
{
private:
    uint8_t btPacketType;
    uint8_t btDataLen;
    uint8_t btReadId;
    uint8_t btCmd;
    std::vector<uint8_t> btAryData;
    uint8_t btCheck;
    std::vector<uint8_t> btAryTranData;

    uint8_t CheckSum(const std::vector<uint8_t> &btAryBuffer, int nStartPos, int nLen);

public:
    MessageTran();
    MessageTran(const std::vector<uint8_t> &btAryTranData);
    MessageTran(uint8_t btReadId, uint8_t btCmd);
    MessageTran(uint8_t btReadId, uint8_t btCmd, const std::vector<uint8_t> &btAryData);

    const std::vector<uint8_t> &AryTranData() const;
    const std::vector<uint8_t> &AryData() const;
    uint8_t ReadId() const;
    uint8_t Cmd() const;
    uint8_t PacketType() const;
};

#endif // MESSAGETRAN_H