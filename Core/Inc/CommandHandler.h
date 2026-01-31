#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>

// Enum für die Befehle
enum class Command : uint8_t {
    G0 = 0,
    G1 = 1,
    G2 = 2,
    M18 = 3,
    M19 = 4,
    M114 = 5
};

// Callback-Typen
// G0: p, l, u, f Parameter (alle optional)
typedef void (*G0Callback)(float p, float l, float u, float f);

// G1: p, l, u Parameter (alle optional, kein f)
typedef void (*G1Callback)(float p, float l, float u);

// G2: keine Parameter
typedef void (*G2Callback)(void);

// M114: keine Parameter
typedef void (*M18Callback)(void);

// M114: keine Parameter
typedef void (*M19Callback)(void);

// M114: keine Parameter
typedef void (*M114Callback)(void);

// Haupt-Interpreter-Klasse
class CommandInterpreter {
public:
    CommandInterpreter();

    // Registriere Callbacks
    void setG0Callback(G0Callback callback);
    void setG1Callback(G1Callback callback);
    void setG2Callback(G2Callback callback);
    void setM18Callback(M18Callback callback);
    void setM19Callback(M19Callback callback);
    void setM114Callback(M114Callback callback);

    // Setze initiale Default-Werte
    void setInitialDefaults(float p, float l, float u, float f);

    // Füge einzelnes Zeichen hinzu
    void addChar(char c);

    // Buffer manuell leeren
    void flush();

    // Aktuell gespeicherte Werte abrufen (für Debugging)
    float getCurrentP() const { return current_p; }
    float getCurrentL() const { return current_l; }
    float getCurrentU() const { return current_u; }
    float getCurrentF() const { return current_f; }

private:
    static const uint8_t MAX_BUFFER = 128;

    char buffer[MAX_BUFFER];
    uint8_t bufferPos;

    G0Callback g0Callback;
    G1Callback g1Callback;
    G2Callback g2Callback;
    M18Callback m18Callback;
    M19Callback m19Callback;
    M114Callback m114Callback;

    // Aktuell gespeicherte Werte (werden bei jedem Befehl aktualisiert)
    float current_p;
    float current_l;
    float current_u;
    float current_f; // kommandoübergreifend

    void processCommand();
    bool parseCommand(const char* str, Command& cmd);
    bool parseFloat(const char* str, float& result);
    bool isDigit(char c);
    void toUpper(char* str);
};

#endif // COMMAND_HANDLER_H
