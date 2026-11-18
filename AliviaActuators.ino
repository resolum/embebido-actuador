/**
 * @file AliviaActuators.ino
 * @brief Alivia | ESP32 de actuadores: 4 servos (puerta, ventana, cortina, luz) controlados por el edge de actuadores.
 *
 * Flujo: voz -> broker MQTT -> edge de actuadores -> (WebSocket o Bluetooth) -> este ESP32 -> servo.
 * Este ESP32 recibe {"type":"command",...}, mueve el servo del canal y siempre responde {"type":"result",...} con
 * ok:true o ok:false. El edge lo reenvia por el broker al edge de voz y (mas adelante) al backend.
 * Contrato completo: docs/protocolo-embebido.md (el mismo archivo esta en el repo del edge).
 *
 * Cada servo es un dispositivo distinto en el backend, con su MAC. El ESP32 solo tiene una MAC real, asi que cada
 * canal usa una MAC logica derivada de la base (ver ActuatorLogic.h, logicalMac). Se imprimen al arrancar: esas son las
 * que se registran en el backend (DOOR, WINDOW, CURTAIN y LIGHT). Si prefieres otras, rellena MAC_OVERRIDE.
 *
 * Elegir transporte: USE_BLUETOOTH 0 = WiFi + WebSocket, 1 = Bluetooth Classic (SPP, hace falta el esquema Huge APP).
 *
 * Requisitos:
 *   - Placa ESP32 Dev Module (ESP32 clasico), core esp32 de Espressif 3.x.
 *   - Librerias: ModestIoT (libraries/ModestIoT del repo), ArduinoJson 7.x, WebSockets 2.7.x, ESP32Servo y las demas de ModestIoT.
 *   - Servos: alimentalos con una fuente externa de 5 V (1 A o mas) con la tierra COMUN con el ESP32. Nunca desde el pin 3V3.
 *   - Monitor serie a 115200: "s" estado, "t <canal> <accion>" prueba local (ej. "t 0 open"), "b <0-100>" simula la
 *     bateria (b off la vuelve a leer), "h" ayuda.
 *   - Bateria (opcional): divisor de voltaje en BATTERY_PIN; se envia en {"type":"telemetry"} cada minuto.
 *
 * Sin el dispositivo: tools/fake_embebido.py del edge de actuadores hace de este ESP32 (mismo protocolo).
 */

#include <ModestIoT.h>
#include <esp_mac.h>  // esp_read_mac

#include "ActuatorLogic.h"
#include "ActuatorNode.h"

#ifndef USE_BLUETOOTH
#define USE_BLUETOOTH 0
#endif

// Credenciales (WIFI_SSID, WIFI_PASSWORD, EDGE_HOST, EDGE_TOKEN): Secrets.h, ignorado por git.
#include "Secrets.h"

static constexpr const char* DEVICE_ID      = "alivia-act-01";  // etiqueta para logs; NO identifica al dispositivo
static constexpr const char* FIRMWARE       = "1.0.0";
static constexpr uint16_t    EDGE_PORT      = 8766;             // [server].port del edge de actuadores
static constexpr const char* EDGE_PATH      = "/ws";            // [server].path
static constexpr const char* BT_DEVICE_NAME = "Alivia-Act-01";
static constexpr unsigned long SERIAL_BAUD_RATE = 115200;

// ---------------------------------------------------------------------------
// Servos. Orden de canales = orden del hello. Pines sin funcion de arranque (no uses 0, 2, 12 ni 15).
// ---------------------------------------------------------------------------
static constexpr uint8_t CHANNELS = 4;
static const int SERVO_PINS[CHANNELS] = {13, 16, 17, 18};

// Bateria (opcional): BAT+ -> 100k -> BATTERY_PIN -> 100k -> GND. -1 = sin bateria (la telemetria sale sin ella).
// Se informa al edge cada TELEMETRY_INTERVAL_MS; el edge avisa al familiar cuando baja del 20 %. Ver BatteryMonitor.h.
static constexpr int   BATTERY_PIN            = 35;
static constexpr float BATTERY_DIVIDER_RATIO  = 2.0f;
static constexpr unsigned long TELEMETRY_INTERVAL_MS = 60000;

// MAC logica fija por canal (opcional). nullptr = derivarla de la MAC base.
static const char* const MAC_OVERRIDE[CHANNELS] = {nullptr, nullptr, nullptr, nullptr};

// closedAngle = cerrado/apagado, openAngle = abierto/encendido, settleMs = tiempo que tarda en llegar.
// CALIBRALOS con tu mecanismo (prueba con "t 0 open" / "t 0 close" por el monitor serie).
static alivia::ChannelConfig channelConfigs[CHANNELS];

static void configureChannels() {
    channelConfigs[0].kind = alivia::Kind::DOOR;    channelConfigs[0].closedAngle = 0; channelConfigs[0].openAngle = 90;  channelConfigs[0].settleMs = 700;
    channelConfigs[1].kind = alivia::Kind::WINDOW;  channelConfigs[1].closedAngle = 0; channelConfigs[1].openAngle = 90;  channelConfigs[1].settleMs = 700;
    channelConfigs[2].kind = alivia::Kind::CURTAIN; channelConfigs[2].closedAngle = 0; channelConfigs[2].openAngle = 180; channelConfigs[2].settleMs = 1200;
    channelConfigs[3].kind = alivia::Kind::LIGHT;   channelConfigs[3].closedAngle = 0; channelConfigs[3].openAngle = 60;  channelConfigs[3].settleMs = 300;
    uint8_t base[6] = {0};
    esp_read_mac(base, ESP_MAC_WIFI_STA);
    for (uint8_t i = 0; i < CHANNELS; i++) {
        if (MAC_OVERRIDE[i] != nullptr) alivia::copyText(channelConfigs[i].mac, sizeof(channelConfigs[i].mac), MAC_OVERRIDE[i]);
        else alivia::logicalMac(base, i, channelConfigs[i].mac, sizeof(channelConfigs[i].mac));
    }
}

// ---------------------------------------------------------------------------
// Objetos (el hardware se inicia en setup(), no en los constructores)
// ---------------------------------------------------------------------------
static StreamGatewayClient::Settings linkSettings() {
    StreamGatewayClient::Settings settings;
    settings.deviceId = DEVICE_ID;  // la MAC base del hello la agrega StreamGatewayClient
    settings.token = EDGE_TOKEN;
    settings.taskStackBytes = 8192;
    return settings;
}

#if USE_BLUETOOTH
BluetoothConnectivityDriver connectivity(BT_DEVICE_NAME);
BluetoothStreamGatewayClient edgeLink(connectivity, linkSettings(), 4096);
#else
WiFiConnectivityDriver connectivity(WIFI_SSID, WIFI_PASSWORD);
WebSocketGatewayClient edgeLink(connectivity, EDGE_HOST, EDGE_PORT, EDGE_PATH, linkSettings());
#endif

alivia::ServoBank servoBank(SERVO_PINS, channelConfigs, CHANNELS);
alivia::ActuatorLogic actuatorLogic(channelConfigs, CHANNELS, servoBank);
alivia::BatteryMonitor batteryMonitor(BATTERY_PIN, BATTERY_DIVIDER_RATIO);
alivia::ActuatorNode actuatorNode(edgeLink, actuatorLogic, channelConfigs, CHANNELS, FIRMWARE, batteryMonitor,
                                  TELEMETRY_INTERVAL_MS);

static void printStatus() {
    char base[18];
    StreamGatewayClient::baseMacAddress(base, sizeof(base));
    SerialConsole::printf("\nMAC base: %s | enlace: %s (%s)\n", base, StreamGatewayClient::describeState(edgeLink.state()), edgeLink.transportName());
    for (uint8_t i = 0; i < CHANNELS; i++) {
        SerialConsole::printf("  canal %u  %-7s  mac %s  angulo supuesto %d  (cerrado %d, abierto %d, espera %lu ms)\n", i,
                              alivia::kindName(channelConfigs[i].kind), channelConfigs[i].mac, actuatorLogic.assumedAngle(i),
                              channelConfigs[i].closedAngle, channelConfigs[i].openAngle, channelConfigs[i].settleMs);
    }
}

static void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    if (line == "s") {
        printStatus();
    } else if (line == "b" || line.startsWith("b ")) {
        const String value = line.length() > 2 ? line.substring(2) : String("");
        if (value == "off" || value.length() == 0) {
            actuatorNode.simulateBattery(-1);
            SerialConsole::printf("[Bateria] sin simulacion: se lee el pin %d\n", BATTERY_PIN);
        } else {
            actuatorNode.simulateBattery(value.toInt());
            SerialConsole::printf("[Bateria] simulando %d %% (se envia al edge ahora)\n", batteryMonitor.simulatedPercent());
        }
    } else if (line.startsWith("t ")) {
        const int space = line.indexOf(' ', 2);
        if (space < 0) {
            SerialConsole::printf("Uso: t <canal> <open|close|on|off>\n");
            return;
        }
        const int channel = line.substring(2, space).toInt();
        String action = line.substring(space + 1);
        action.trim();
        actuatorNode.submitLocal(channel, action.c_str());
    } else {
        SerialConsole::printf("Comandos: s estado | t <canal> <open|close|on|off> prueba local sin edge (ej. t 0 open) | "
                              "b <0-100> simula la bateria, b off la vuelve a leer\n");
    }
}

void setup() {
    SerialConsole::begin(SERIAL_BAUD_RATE);
    delay(300);
    SerialConsole::printf("\n=== Alivia Actuators | ESP32 con 4 servos | ModestIoT ===\n");
    configureChannels();
    servoBank.begin();  // lleva cada servo a su angulo de cerrado/apagado
    batteryMonitor.begin();
    printStatus();
    SerialConsole::printf("Registra en el backend una MAC por dispositivo (DOOR, WINDOW, CURTAIN, LIGHT) con las MAC de arriba.\n");
#if USE_BLUETOOTH
    SerialConsole::printf("Transporte: Bluetooth SPP como \"%s\" (en el edge: [[bluetooth.links]] mac = MAC base)\n", BT_DEVICE_NAME);
#else
    SerialConsole::printf("Transporte: WiFi \"%s\" -> ws://%s:%u%s\n", WIFI_SSID, EDGE_HOST, (unsigned)EDGE_PORT, EDGE_PATH);
#endif
    if (!actuatorNode.begin()) SerialConsole::printf("ERROR: no se pudo iniciar el enlace\n");
}

void loop() {
    actuatorNode.service();
    handleSerial();
    delay(5);
}
