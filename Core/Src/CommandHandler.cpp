#include "CommandHandler.h"

CommandInterpreter::CommandInterpreter()
    : bufferPos(0)
    , g0Callback(nullptr)
    , g1Callback(nullptr)
    , g2Callback(nullptr)
    , m114Callback(nullptr)
	, m119Callback(nullptr)
    , current_p(0.0f)
    , current_l(0.0f)
    , current_u(0.0f)
    , current_f(0.0f) {
    buffer[0] = '\0';
}

void CommandInterpreter::setG0Callback(G0Callback callback) {
    g0Callback = callback;
}

void CommandInterpreter::setG1Callback(G1Callback callback) {
    g1Callback = callback;
}

void CommandInterpreter::setG2Callback(G2Callback callback) {
    g2Callback = callback;
}

void CommandInterpreter::setM18Callback(M18Callback callback) {
    m18Callback = callback;
}

void CommandInterpreter::setM19Callback(M19Callback callback) {
    m19Callback = callback;
}

void CommandInterpreter::setM114Callback(M114Callback callback) {
    m114Callback = callback;
}

void CommandInterpreter::setM119Callback(M119Callback callback) {
    m119Callback = callback;
}

void CommandInterpreter::setInitialDefaults(float p, float l, float u, float f) {
    current_p = p;
    current_l = l;
    current_u = u;
    current_f = f;
}

void CommandInterpreter::addChar(char c) {
    if (c == '\n' || c == '\r') {
        if (bufferPos > 0) {
            buffer[bufferPos] = '\0';
            processCommand();
            bufferPos = 0;
            buffer[0] = '\0';
        }
    } else if (bufferPos < MAX_BUFFER - 1) {
        buffer[bufferPos++] = c;
        buffer[bufferPos] = '\0';
    }
}

void CommandInterpreter::flush() {
    if (bufferPos > 0) {
        buffer[bufferPos] = '\0';
        processCommand();
        bufferPos = 0;
        buffer[0] = '\0';
    }
}

bool CommandInterpreter::isDigit(char c) {
    return c >= '0' && c <= '9';
}

void CommandInterpreter::toUpper(char* str) {
    while (*str) {
        if (*str >= 'a' && *str <= 'z') {
            *str = *str - 32;
        }
        str++;
    }
}

bool CommandInterpreter::parseCommand(const char* str, Command& cmd) {
    // Überspringe führende Leerzeichen
    while (*str == ' ') str++;

    char temp[8];
    uint8_t i = 0;

    // Kopiere und konvertiere zu Großbuchstaben
    while (*str && *str != ' ' && i < 7) {
        temp[i] = *str;
        if (temp[i] >= 'a' && temp[i] <= 'z') {
            temp[i] = temp[i] - 32;
        }
        i++;
        str++;
    }
    temp[i] = '\0';

    // Prüfe G-Befehle
    if (temp[0] == 'G' && temp[2] == '\0') {
        if (temp[1] == '0') {
            cmd = Command::G0;
            return true;
        } else if (temp[1] == '1') {
            cmd = Command::G1;
            return true;
        } else if (temp[1] == '2') {
            cmd = Command::G2;
            return true;
        }
    }

    // Prüfe M-Befehle
    if (temp[0] == 'M') {
        // M114
        if (temp[1] == '1' && temp[2] == '1' && temp[3] == '4' && temp[4] == '\0') {
            cmd = Command::M114;
            return true;
        } else  if (temp[1] == '1' && temp[2] == '1' && temp[3] == '9' && temp[4] == '\0') {
            cmd = Command::M119;
            return true;
        } else  if (temp[1] == '1' && temp[2] == '8' && temp[3] == '\0') {
            cmd = Command::M18;
            return true;
        } else  if (temp[1] == '1' && temp[2] == '9' && temp[3] == '\0') {
            cmd = Command::M19;
            return true;
        }
    }

    return false;
}

bool CommandInterpreter::parseFloat(const char* str, float& result) {
    result = 0.0f;
    float sign = 1.0f;
    bool hasDigits = false;

    // Vorzeichen
    if (*str == '-') {
        sign = -1.0f;
        str++;
    } else if (*str == '+') {
        str++;
    }

    // Ganzzahlteil
    while (isDigit(*str)) {
        result = result * 10.0f + (*str - '0');
        str++;
        hasDigits = true;
    }

    // Dezimalteil
    if (*str == '.' || *str == ',') {
        str++;
        float decimal = 0.1f;
        while (isDigit(*str)) {
            result += (*str - '0') * decimal;
            decimal *= 0.1f;
            str++;
            hasDigits = true;
        }
    }

    result *= sign;
    return hasDigits;
}

void CommandInterpreter::processCommand() {
    if (bufferPos == 0) {
        return;
    }

    Command cmd;
    if (!parseCommand(buffer, cmd)) {
        return; // Unbekannter Befehl
    }

    // Befehle ohne Parameter - direkt Callback aufrufen
    if (cmd == Command::G2) {
        if (g2Callback) {
            g2Callback();
        }
        return;
    }

    if (cmd == Command::M18) {
        if (m18Callback) {
            m18Callback();
        }
        return;
    }

    if (cmd == Command::M19) {
        if (m19Callback) {
            m19Callback();
        }
        return;
    }

    if (cmd == Command::M114) {
        if (m114Callback) {
            m114Callback();
        }
        return;
    }

    if (cmd == Command::M119) {
        if (m119Callback) {
            m119Callback();
        }
        return;
    }

    // Finde Start der Parameter (nach dem Befehl)
    const char* params = buffer;
    while (*params && *params != ' ') {
        params++;
    }

    // Starte mit aktuellen gespeicherten Werten
    float p_val = current_p;
    float l_val = current_l;
    float u_val = current_u;
    float f_val = current_f;

    // Parse Parameter (überschreibe gespeicherte Werte falls vorhanden)
    bool has_p = false;
    bool has_l = false;
    bool has_u = false;
    bool has_f = false;

    const char* current = params;
    while (*current) {
        // Überspringe Leerzeichen
        while (*current == ' ' || *current == '\t') current++;
        if (!*current) break;

        // Prüfe Parameter-Buchstabe
        char paramType = *current;
        if (paramType >= 'a' && paramType <= 'z') {
            paramType = paramType - 32; // zu Großbuchstaben
        }

        if (paramType == 'P' || paramType == 'L' ||
            paramType == 'U' || paramType == 'F') {

            current++; // Überspringe Parameter-Buchstabe

            float value;
            if (parseFloat(current, value)) {
                switch (paramType) {
                    case 'P':
                        p_val = value;
                        has_p = true;
                        break;
                    case 'L':
                        l_val = value;
                        has_l = true;
                        break;
                    case 'U':
                        u_val = value;
                        has_u = true;
                        break;
                    case 'F':
                        f_val = value;
                        has_f = true;
                        break;
                }
            }

            // Überspringe bis zum nächsten Leerzeichen oder Ende
            while (*current && *current != ' ' && *current != '\t') {
                current++;
            }
        } else {
            // Unbekanntes Zeichen, überspringe
            current++;
        }
    }

    // Aktualisiere gespeicherte Werte nur wenn sie im Befehl vorhanden waren
    if (has_p) current_p = p_val;
    if (has_l) current_l = l_val;
    if (has_u) current_u = u_val;
    if (has_f) current_f = f_val; // F ist kommandoübergreifend

    // Rufe entsprechenden Callback auf mit aktuellen Werten
    if (cmd == Command::G0 && g0Callback) {
        g0Callback(current_p, current_l, current_u, current_f);
    } else if (cmd == Command::G1 && g1Callback) {
        g1Callback(current_p, current_l, current_u);
    }
}
