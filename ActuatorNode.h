#ifndef ALIVIA_ACTUATOR_NODE_H
#define ALIVIA_ACTUATOR_NODE_H


#include <Arduino.h>
#include <ModestIoT.h>

#include "ActuatorLogic.h"
#include "ActuatorProtocol.h"

namespace alivia {

class ServoBank : public ServoPort {
public:
    ServoBank(const int* pins, const ChannelConfig* channels, uint8_t count) : pins(pins), channels(channels), count(count) {}

    void begin() {
        for (uint8_t i = 0; i < count; i++) motors[i] = new ServoMotor(pins[i], channels[i].closedAngle);
    }

    void write(uint8_t channel, int angle) override {
        if (channel >= count || motors[channel] == nullptr) return;
        motors[channel]->setTargetAngleBufferInDegrees(angle);
        motors[channel]->handle(ServoMotor::DRIVE_TO_BUFFERED_ANGLE_COMMAND);
    }

private:
    const int* pins;
    const ChannelConfig* channels;
    uint8_t count;
    ServoMotor* motors[ActuatorLogic::MAX_CHANNELS] = {nullptr, nullptr, nullptr, nullptr};
};

class ActuatorNode : public LinkProtocolDelegate {
public:
    ActuatorNode(StreamGatewayClient& edgeLink, ActuatorLogic& actuatorLogic, const ChannelConfig* channelConfigs,
                 uint8_t channelCount, const char* firmwareVersion)
        : link(edgeLink), logic(actuatorLogic), channels(channelConfigs), count(channelCount), firmware(firmwareVersion) {}

    bool begin() {
        lock = xSemaphoreCreateMutex();
        if (lock == nullptr) return false;
        link.setProtocolDelegate(this);
        return link.begin();
    }

    void service() {
        lock_();
        logic.tick(millis());
        unlock_();
        for (uint8_t guard = 0; guard < 8; guard++) {
            ActuatorResult r;
            lock_();
            const bool has = logic.peekResult(r);
            unlock_();
            if (!has) return;
            char json[256];
            ActuatorLogic::toJson(r, json, sizeof(json));
            if (strncmp(r.correlationId, "local-", 6) == 0) {
                SerialConsole::printf("[Local] canal %d: %s %s %s (%lu ms)\n", r.channel, r.ok ? "OK" : "FALLO", r.detail,
                                      r.state, r.elapsedMs);
            } else if (!link.sendText(json)) {
                return;
            }
            lock_();
            logic.ackResult();
            unlock_();
        }
    }

    void submitLocal(int channel, const char* action) {
        if (channel < 0 || channel >= count) {
            SerialConsole::printf("[Local] canal %d inexistente (0 a %u)\n", channel, count - 1);
            return;
        }
        JsonDocument doc;
        char id[24];
        snprintf(id, sizeof(id), "local-%lu", ++localCounter);
        doc["correlationId"] = id;
        doc["channel"] = channel;
        doc["mac"] = channels[channel].mac;
        doc["kind"] = kindName(channels[channel].kind);
        doc["action"] = action;
        doc["ttlMs"] = 5000;
        lock_();
        logic.enqueue(doc.as<JsonVariantConst>(), millis());
        unlock_();
    }

    void fillHello(JsonDocument& hello) override { fillActuatorHello(hello, channels, count, firmware); }

    void onLinkReady(JsonDocument& ready) override {
        String text;
        serializeJson(ready["channels"], text);
        SerialConsole::printf("[Actuadores] el edge habilito los canales %s\n", text.c_str());
    }

    void onText(const char* type, JsonDocument& message) override {
        if (strcmp(type, "command") != 0) return;
        lock_();
        logic.enqueue(message.as<JsonVariantConst>(), millis());
        unlock_();
    }

    void onLinkDown() override {
        lock_();
        logic.clearResults();
        unlock_();
    }

private:
    void lock_() { if (lock != nullptr) xSemaphoreTake(lock, portMAX_DELAY); }
    void unlock_() { if (lock != nullptr) xSemaphoreGive(lock); }

    StreamGatewayClient& link;
    ActuatorLogic& logic;
    const ChannelConfig* channels;
    uint8_t count;
    const char* firmware;
    SemaphoreHandle_t lock = nullptr;
    unsigned long localCounter = 0;
};

}  // namespace alivia

#endif  // ALIVIA_ACTUATOR_NODE_H
