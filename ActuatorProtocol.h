#ifndef ALIVIA_ACTUATOR_PROTOCOL_H
#define ALIVIA_ACTUATOR_PROTOCOL_H



#include "ActuatorLogic.h"

namespace alivia {

inline void fillActuatorHello(JsonDocument& hello, const ChannelConfig* channels, uint8_t count, const char* firmware) {
    JsonArray capabilities = hello["capabilities"].to<JsonArray>();
    capabilities.add("actuators");
    hello["fw"] = firmware;
    JsonArray list = hello["channels"].to<JsonArray>();
    for (uint8_t i = 0; i < count; i++) {
        JsonObject item = list.add<JsonObject>();
        item["channel"] = i;
        item["mac"] = channels[i].mac;
        item["kind"] = kindName(channels[i].kind);
    }
}

}  // namespace alivia

#endif  // ALIVIA_ACTUATOR_PROTOCOL_H
