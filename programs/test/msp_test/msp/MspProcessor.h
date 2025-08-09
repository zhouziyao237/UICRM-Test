#ifndef _ESPFC_MSP_PROCESSOR_H_
#define _ESPFC_MSP_PROCESSOR_H_

#include "Msp.h"
#include "MspParser.h"
#include "MspProcessor.h"
#include "bsp_serial.h"
// #include "Model/Model.h"
//#include "Hardware.h"
//#include "platform.h"

namespace Msp {

class MspProcessor
{
  public:
    // MspProcessor(Model& model): _model(model) {}

    bool process(char c, MspMessage& msg, MspResponse& res, Device::SerialDevice& s);

    void processCommand(MspMessage& m, MspResponse& r);

    void sendResponse(MspResponse& r, Device::SerialDevice& s);

    void sendResponseV1(MspResponse& r, Device::SerialDevice& s);

    void sendResponseV2(MspResponse& r, Device::SerialDevice& s);

    void postCommand();

    bool debugSkip(uint8_t cmd);

    void debugMessage(const MspMessage& m);

    void debugResponse(const MspResponse& r);

  private:
    // Model& _model;
    MspParser _parser;
    bool _reboot;
};

}

#endif
