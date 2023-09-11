#include "canbus.h"

class CanBusMasterBase : public CanBusBase {

    /**
     * @brief Receives a CAN message
     * 
     * @returns: true on success or false otherwise (e.g. if the send queue is
     * full).
     */
    virtual bool receive_message(const can_Message_t& message) = 0;
};
