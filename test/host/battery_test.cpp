// Prueba en la PC de la parte de BatteryMonitor.h que no depende del hardware.
//   c++ -std=c++17 -I. test/host/battery_test.cpp -o /tmp/battery_test && /tmp/battery_test
#include <cassert>
#include <cstdio>
#include <cstring>

#include "BatteryMonitor.h"

using alivia::BatteryMath;
using alivia::BatteryReading;
using alivia::TelemetryBuilder;

int main() {
    // Curva de la celda Li-ion: extremos, puntos de la tabla e interpolacion.
    assert(BatteryMath::percentFromMillivolts(3000) == 0);
    assert(BatteryMath::percentFromMillivolts(3300) == 0);
    assert(BatteryMath::percentFromMillivolts(3600) == 20);
    assert(BatteryMath::percentFromMillivolts(3650) == 27);
    assert(BatteryMath::percentFromMillivolts(3800) == 50);
    assert(BatteryMath::percentFromMillivolts(4200) == 100);
    assert(BatteryMath::percentFromMillivolts(4400) == 100);
    for (int mv = 3300; mv < 4200; mv += 10) {
        assert(BatteryMath::percentFromMillivolts(mv) <= BatteryMath::percentFromMillivolts(mv + 10));  // monotona
    }

    // Un pin al aire o en corto no es una bateria.
    assert(!BatteryMath::plausible(0));
    assert(!BatteryMath::plausible(1200));
    assert(BatteryMath::plausible(3700));
    assert(!BatteryMath::plausible(5000));

    // Telemetria con y sin bateria, con el formato que leen los edge.
    char json[192];
    BatteryReading low;
    low.present = true;
    low.millivolts = 3550;
    low.percent = BatteryMath::percentFromMillivolts(3550);
    assert(TelemetryBuilder::build(json, sizeof(json), 1000, 150000, -60, low) > 0);
    assert(strcmp(json, "{\"type\":\"telemetry\",\"uptimeMs\":1000,\"freeHeap\":150000,\"rssi\":-60,\"batteryMv\":3550,"
                        "\"batteryPct\":15}") == 0);
    std::printf("%s\n", json);

    BatteryReading none;
    assert(TelemetryBuilder::build(json, sizeof(json), 5, 1, 0, none) > 0);
    assert(strcmp(json, "{\"type\":\"telemetry\",\"uptimeMs\":5,\"freeHeap\":1,\"rssi\":0}") == 0);
    std::printf("%s\n", json);

    char tiny[10];
    assert(TelemetryBuilder::build(tiny, sizeof(tiny), 5, 1, 0, low) == 0);  // no cabe: no se envia a medias

    std::printf("battery_test: OK\n");
    return 0;
}
