#ifndef ALIVIA_ACTUATOR_LOGIC_H
#define ALIVIA_ACTUATOR_LOGIC_H



#include <ArduinoJson.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

namespace alivia {

enum class Kind : uint8_t { DOOR, WINDOW, CURTAIN, LIGHT, UNKNOWN };

inline const char* kindName(Kind kind) {
    switch (kind) {
        case Kind::DOOR: return "DOOR";
        case Kind::WINDOW: return "WINDOW";
        case Kind::CURTAIN: return "CURTAIN";
        case Kind::LIGHT: return "LIGHT";
        default: return "?";
    }
}

inline Kind kindFromName(const char* name) {
    if (name == nullptr) return Kind::UNKNOWN;
    if (strcasecmp(name, "DOOR") == 0) return Kind::DOOR;
    if (strcasecmp(name, "WINDOW") == 0) return Kind::WINDOW;
    if (strcasecmp(name, "CURTAIN") == 0) return Kind::CURTAIN;
    if (strcasecmp(name, "LIGHT") == 0) return Kind::LIGHT;
    return Kind::UNKNOWN;
}

inline bool kindAccepts(Kind kind, const char* action) {
    if (kind == Kind::LIGHT) return strcmp(action, "on") == 0 || strcmp(action, "off") == 0;
    if (kind == Kind::UNKNOWN) return false;
    return strcmp(action, "open") == 0 || strcmp(action, "close") == 0;
}

inline bool actionIsPositive(const char* action) { return strcmp(action, "open") == 0 || strcmp(action, "on") == 0; }

inline const char* stateAfter(const char* action) {
    if (strcmp(action, "open") == 0) return "open";
    if (strcmp(action, "close") == 0) return "closed";
    if (strcmp(action, "on") == 0) return "on";
    return "off";
}

inline void copyText(char* destination, size_t capacity, const char* source) {
    if (capacity == 0) return;
    size_t n = source == nullptr ? 0 : strlen(source);
    if (n >= capacity) n = capacity - 1;
    if (n > 0) memcpy(destination, source, n);
    destination[n] = '\0';
}

inline void logicalMac(const uint8_t base[6], uint8_t channel, char* out, size_t capacity) {
    uint8_t m[6];
    memcpy(m, base, 6);
    m[0] |= 0x02;
    m[5] = static_cast<uint8_t>(m[5] + channel + 1);
    snprintf(out, capacity, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

struct ChannelConfig {
    Kind kind = Kind::UNKNOWN;
    char mac[18] = {0};
    int closedAngle = 0;
    int openAngle = 90;
    unsigned long settleMs = 700;
};

class ServoPort {
public:
    virtual ~ServoPort() = default;
    virtual void write(uint8_t channel, int angle) = 0;
};

static constexpr size_t kCorrLen = 48;

struct ActuatorResult {
    char correlationId[kCorrLen] = {0};
    int channel = -1;
    bool ok = false;
    char detail[24] = {0};
    char state[8] = {0};
    int angle = -1;
    unsigned long elapsedMs = 0;
};

class ActuatorLogic {
public:
    static constexpr uint8_t MAX_CHANNELS = 4;
    static constexpr uint8_t QUEUE_SIZE = 8;
    static constexpr uint8_t OUTBOX_SIZE = 16;
    static constexpr uint8_t DONE_SIZE = 8;
    static constexpr uint8_t MAX_FOLLOWERS = 3;

    ActuatorLogic(const ChannelConfig* channels, uint8_t count, ServoPort& servos, unsigned long repeatSettleMs = 150)
        : cfg(channels), n(count > MAX_CHANNELS ? MAX_CHANNELS : count), servo(servos), repeatMs(repeatSettleMs) {
        for (uint8_t i = 0; i < n; i++) angle[i] = cfg[i].closedAngle;  // se asume cerrado al arrancar
    }

    uint8_t channelCount() const { return n; }
    int assumedAngle(uint8_t channel) const { return channel < n ? angle[channel] : -1; }

    bool enqueue(JsonVariantConst command, unsigned long receivedAtMs) {
        Pending p;
        const char* corr = command["correlationId"] | "";
        if (corr[0] == '\0' || strlen(corr) >= kCorrLen) return true;  // sin id valido nadie espera respuesta
        copyText(p.corr, sizeof(p.corr), corr);
        p.channel = command["channel"].is<int>() ? command["channel"].as<int>() : -1;
        copyText(p.mac, sizeof(p.mac), command["mac"] | "");
        copyText(p.kind, sizeof(p.kind), command["kind"] | "");
        const char* action = command["action"] | "";
        p.hasAction = action[0] != '\0';
        copyText(p.action, sizeof(p.action), action);
        p.ttlMs = command["ttlMs"] | 5000UL;
        p.receivedAt = receivedAtMs;
        if (qCount >= QUEUE_SIZE) {
            fail(p.corr, p.channel, "cola llena", false);
            return false;
        }
        queue[(qHead + qCount) % QUEUE_SIZE] = p;
        qCount++;
        return true;
    }

    void tick(unsigned long now) {
        for (uint8_t ch = 0; ch < n; ch++) {
            Motion& m = motion[ch];
            if (m.active && now - m.startedAt >= m.settleMs) finish(ch, now);
        }
        while (qCount > 0) {
            Pending p = queue[qHead];
            qHead = (qHead + 1) % QUEUE_SIZE;
            qCount--;
            process(p, now);
        }
    }

    // ---- salida: resultados por enviar ----
    bool peekResult(ActuatorResult& out) const {
        if (oCount == 0) return false;
        out = outbox[oHead];
        return true;
    }
    void ackResult() {
        if (oCount == 0) return;
        oHead = (oHead + 1) % OUTBOX_SIZE;
        oCount--;
    }
    void clearResults() { oHead = 0; oCount = 0; }
    uint8_t pendingResults() const { return oCount; }

    static size_t toJson(const ActuatorResult& r, char* out, size_t capacity) {
        JsonDocument doc;
        doc["type"] = "result";
        doc["correlationId"] = r.correlationId;
        doc["channel"] = r.channel;
        doc["ok"] = r.ok;
        doc["detail"] = r.detail;
        doc["state"] = r.state;
        doc["angle"] = r.angle;
        doc["elapsedMs"] = r.elapsedMs;
        return serializeJson(doc, out, capacity);
    }

private:
    struct Pending {
        char corr[kCorrLen] = {0};
        int channel = -1;
        char mac[20] = {0};
        char kind[10] = {0};
        char action[8] = {0};
        bool hasAction = false;
        unsigned long ttlMs = 0;
        unsigned long receivedAt = 0;
    };

    struct Waiter {
        char corr[kCorrLen] = {0};
        unsigned long since = 0;
    };

    struct Motion {
        bool active = false;
        char action[8] = {0};
        int target = 0;
        unsigned long startedAt = 0;
        unsigned long settleMs = 0;
        Waiter leader;
        Waiter followers[MAX_FOLLOWERS];
        uint8_t followerCount = 0;
    };

    const ChannelConfig* cfg;
    uint8_t n;
    ServoPort& servo;
    unsigned long repeatMs;
    int angle[MAX_CHANNELS] = {0};
    Motion motion[MAX_CHANNELS];

    Pending queue[QUEUE_SIZE];
    uint8_t qHead = 0, qCount = 0;
    ActuatorResult outbox[OUTBOX_SIZE];
    uint8_t oHead = 0, oCount = 0;
    ActuatorResult done[DONE_SIZE];
    uint8_t dNext = 0, dCount = 0;

    void emit(const ActuatorResult& r, bool remember) {
        if (oCount == OUTBOX_SIZE) {
            oHead = (oHead + 1) % OUTBOX_SIZE;
            oCount--;
        }
        outbox[(oHead + oCount) % OUTBOX_SIZE] = r;
        oCount++;
        if (remember) {
            done[dNext] = r;
            dNext = (dNext + 1) % DONE_SIZE;
            if (dCount < DONE_SIZE) dCount++;
        }
    }

    void fail(const char* corr, int channel, const char* detail, bool remember = true) {
        ActuatorResult r;
        copyText(r.correlationId, sizeof(r.correlationId), corr);
        r.channel = channel;
        r.ok = false;
        copyText(r.detail, sizeof(r.detail), detail);
        emit(r, remember);
    }

    bool inFlight(const char* corr) const {
        for (uint8_t ch = 0; ch < n; ch++) {
            const Motion& m = motion[ch];
            if (!m.active) continue;
            if (strcmp(m.leader.corr, corr) == 0) return true;
            for (uint8_t i = 0; i < m.followerCount; i++)
                if (strcmp(m.followers[i].corr, corr) == 0) return true;
        }
        return false;
    }

    const ActuatorResult* findDone(const char* corr) const {
        for (uint8_t i = 0; i < dCount; i++)
            if (strcmp(done[i].correlationId, corr) == 0) return &done[i];
        return nullptr;
    }

    void process(const Pending& p, unsigned long now) {
        if (inFlight(p.corr)) return;  // duplicado mientras se mueve: el resultado ya viene en camino
        if (const ActuatorResult* known = findDone(p.corr)) {
            emit(*known, false);  // duplicado ya resuelto: mismo resultado, sin mover el servo
            return;
        }
        if (!p.hasAction) return fail(p.corr, p.channel, "comando invalido");
        if (p.channel < 0 || p.channel >= n) return fail(p.corr, p.channel, "canal inexistente");
        const uint8_t ch = static_cast<uint8_t>(p.channel);
        const ChannelConfig& c = cfg[ch];
        if (strcasecmp(p.mac, c.mac) != 0) return fail(p.corr, p.channel, "mac no coincide");
        if (kindFromName(p.kind) != c.kind) return fail(p.corr, p.channel, "tipo no coincide");
        if (!kindAccepts(c.kind, p.action)) return fail(p.corr, p.channel, "accion invalida");
        if (now - p.receivedAt > p.ttlMs) return fail(p.corr, p.channel, "caducado");

        Motion& m = motion[ch];
        if (m.active && strcmp(m.action, p.action) == 0) {  // misma accion en curso: espera el mismo movimiento
            if (m.followerCount >= MAX_FOLLOWERS) return fail(p.corr, p.channel, "ocupado");
            copyText(m.followers[m.followerCount].corr, kCorrLen, p.corr);
            m.followers[m.followerCount].since = now;
            m.followerCount++;
            return;
        }
        if (m.active) abort(ch, "reemplazado");  // accion contraria: la nueva reemplaza a la anterior

        const int target = actionIsPositive(p.action) ? c.openAngle : c.closedAngle;
        const unsigned long settle = (angle[ch] == target && repeatMs < c.settleMs) ? repeatMs : c.settleMs;
        servo.write(ch, target);  // siempre se reenvia el angulo: no se asume la posicion fisica
        angle[ch] = target;
        m.active = true;
        copyText(m.action, sizeof(m.action), p.action);
        m.target = target;
        m.startedAt = now;
        m.settleMs = settle;
        copyText(m.leader.corr, kCorrLen, p.corr);
        m.leader.since = now;
        m.followerCount = 0;
    }

    void abort(uint8_t ch, const char* detail) {
        Motion& m = motion[ch];
        fail(m.leader.corr, ch, detail);
        for (uint8_t i = 0; i < m.followerCount; i++) fail(m.followers[i].corr, ch, detail);
        m.active = false;
        m.followerCount = 0;
    }

    void finish(uint8_t ch, unsigned long now) {
        Motion& m = motion[ch];
        auto success = [&](const Waiter& w) {
            ActuatorResult r;
            copyText(r.correlationId, sizeof(r.correlationId), w.corr);
            r.channel = ch;
            r.ok = true;
            copyText(r.state, sizeof(r.state), stateAfter(m.action));
            r.angle = m.target;
            r.elapsedMs = now - w.since;
            emit(r, true);
        };
        success(m.leader);
        for (uint8_t i = 0; i < m.followerCount; i++) success(m.followers[i]);
        m.active = false;
        m.followerCount = 0;
    }
};

}  // namespace alivia

#endif  // ALIVIA_ACTUATOR_LOGIC_H
