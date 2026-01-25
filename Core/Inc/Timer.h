#ifndef TIMER_H
#define TIMER_H

#include <functional>
#include <cstdint>
#include <vector>
#include <algorithm>

class Timer {
public:
    // Typ-Definition für die Callback-Funktion
    using CallbackFunction = std::function<void()>;

    /**
     * @brief Konstruktor
     * @param timeout_ms Timeout in Millisekunden
     * @param callback Optional: Callback-Funktion die beim Ablauf aufgerufen wird
     * @param auto_register Automatisch beim TimerManager registrieren (Standard: true)
     * @param auto_restart Timer automatisch neu starten nach Ablauf (Standard: true)
     * @param auto_start Timer automatisch beim Erstellen starten (Standard: true)
     */
    Timer(uint32_t timeout_ms = 1000,
          CallbackFunction callback = nullptr,
          bool auto_register = true,
          bool auto_restart = true,
          bool auto_start = true);

    /**
     * @brief Destruktor - entfernt Timer automatisch aus Manager
     */
    ~Timer();

    /**
     * @brief Update-Methode - wird vom TimerManager aufgerufen
     */
    void update();

    /**
     * @brief Timer starten/aktivieren
     */
    void start() {
        is_active_ = true;
        current_time_ = 0;
    }

    /**
     * @brief Timer stoppen/deaktivieren
     */
    void stop() {
        is_active_ = false;
    }

    /**
     * @brief Timer zurücksetzen (behält aktiv/inaktiv Status bei)
     */
    void reset() {
        current_time_ = 0;
    }

    /**
     * @brief Prüft ob Timer aktiv ist
     */
    bool isActive() const {
        return is_active_;
    }

    /**
     * @brief Prüft ob Timer abgelaufen ist
     */
    bool hasElapsed() const {
        return current_time_ >= timeout_;
    }

    /**
     * @brief Setzt eine neue Callback-Funktion
     */
    void setCallback(CallbackFunction callback) {
        callback_ = callback;
    }

    /**
     * @brief Prüft ob ein Callback gesetzt ist
     */
    bool hasCallback() const {
        return static_cast<bool>(callback_);
    }

    /**
     * @brief Setzt einen neuen Timeout-Wert
     */
    void setTimeout(uint32_t timeout_ms) {
        timeout_ = timeout_ms;
    }

    /**
     * @brief Auto-Restart aktivieren
     */
    void enableAutoRestart() {
        auto_restart_ = true;
    }

    /**
     * @brief Auto-Restart deaktivieren
     */
    void disableAutoRestart() {
        auto_restart_ = false;
    }

    /**
     * @brief Auto-Restart Status setzen
     * @param enabled true = aktiviert, false = deaktiviert
     */
    void setAutoRestart(bool enabled) {
        auto_restart_ = enabled;
    }

    /**
     * @brief Gibt Auto-Restart Status zurück
     */
    bool isAutoRestartEnabled() const {
        return auto_restart_;
    }

    /**
     * @brief Gibt den aktuellen Timeout-Wert zurück
     */
    uint32_t getTimeout() const {
        return timeout_;
    }

    /**
     * @brief Gibt die aktuelle verstrichene Zeit zurück
     */
    uint32_t getElapsedTime() const {
        return current_time_;
    }

    /**
     * @brief Gibt die verbleibende Zeit zurück
     */
    uint32_t getRemainingTime() const {
        if (current_time_ >= timeout_) {
            return 0;
        }
        return timeout_ - current_time_;
    }

private:
    uint32_t timeout_;        // Timeout-Wert in ms
    uint32_t current_time_;   // Aktueller Zeitwert in ms
    bool is_active_;          // Aktiv/Inaktiv Status
    bool auto_restart_;       // Auto-Restart aktiviert
    CallbackFunction callback_; // Callback-Funktion
    bool is_registered_;      // Ist beim Manager registriert
};


/**
 * @brief TimerManager - Zentrale Verwaltung aller Timer
 * Singleton Pattern für globalen Zugriff
 */
class TimerManager {
public:
    /**
     * @brief Singleton-Instanz abrufen
     */
    static TimerManager& getInstance() {
        static TimerManager instance;
        return instance;
    }

    /**
     * @brief Zentrale Update-Methode für ALLE Timer - alle 10ms aufrufen!
     */
    void updateAll() {
        for (auto* timer : timers_) {
            if (timer != nullptr) {
                timer->update();
            }
        }
    }

    /**
     * @brief Timer beim Manager registrieren
     */
    void registerTimer(Timer* timer) {
        if (timer != nullptr) {
            timers_.push_back(timer);
        }
    }

    /**
     * @brief Timer vom Manager abmelden
     */
    void unregisterTimer(Timer* timer) {
        timers_.erase(
            std::remove(timers_.begin(), timers_.end(), timer),
            timers_.end()
        );
    }

    /**
     * @brief Anzahl der registrierten Timer
     */
    size_t getTimerCount() const {
        return timers_.size();
    }

    /**
     * @brief Alle Timer zurücksetzen
     */
    void resetAll() {
        for (auto* timer : timers_) {
            if (timer != nullptr) {
                timer->reset();
            }
        }
    }

    /**
     * @brief Alle Timer starten
     */
    void startAll() {
        for (auto* timer : timers_) {
            if (timer != nullptr) {
                timer->start();
            }
        }
    }

    /**
     * @brief Alle Timer stoppen
     */
    void stopAll() {
        for (auto* timer : timers_) {
            if (timer != nullptr) {
                timer->stop();
            }
        }
    }

    /**
     * @brief Auto-Restart für alle Timer aktivieren
     */
    void enableAutoRestartAll() {
        for (auto* timer : timers_) {
            if (timer != nullptr) {
                timer->enableAutoRestart();
            }
        }
    }

    /**
     * @brief Auto-Restart für alle Timer deaktivieren
     */
    void disableAutoRestartAll() {
        for (auto* timer : timers_) {
            if (timer != nullptr) {
                timer->disableAutoRestart();
            }
        }
    }

private:
    TimerManager() = default;
    ~TimerManager() = default;

    // Singleton: Kopieren verhindern
    TimerManager(const TimerManager&) = delete;
    TimerManager& operator=(const TimerManager&) = delete;

    std::vector<Timer*> timers_;
};


// Implementation der Timer-Methoden
inline Timer::Timer(uint32_t timeout_ms, CallbackFunction callback, bool auto_register, bool auto_restart, bool auto_start)
    : timeout_(timeout_ms)
    , current_time_(0)
    , is_active_(false)
    , auto_restart_(auto_restart)
    , callback_(callback)
    , is_registered_(false)
{
    if (auto_register) {
        TimerManager::getInstance().registerTimer(this);
        is_registered_ = true;
    }

    // Automatischer Start wenn gewünscht
    if (auto_start) {
        start();
    }
}

inline Timer::~Timer() {
    if (is_registered_) {
        TimerManager::getInstance().unregisterTimer(this);
    }
}

inline void Timer::update() {
    if (!is_active_) {
        return;
    }

    current_time_ += 10; // 10ms Inkrement

    if (current_time_ >= timeout_) {
        // Timer abgelaufen - Callback NUR aufrufen wenn gesetzt!
        if (callback_) {  // <-- GESCHÜTZT: Prüfung ob Callback existiert
            callback_();
        }

        // Auto-Restart Logik
        if (auto_restart_) {
            current_time_ = 0;  // Timer zurücksetzen und weiterlaufen
            // is_active_ bleibt true
        } else {
            is_active_ = false;  // Timer stoppen
        }
    }
}

#endif // TIMER_H
