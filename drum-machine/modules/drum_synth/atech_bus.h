/**
 * @file atech_bus.h
 * @brief Shared USB-serial command router for custom Atech modules.
 *
 * Several modules in one firmware all want the host's
 * `{"action":"<prefix>_<cmd>","value":"..."}` lines. If each read Serial on
 * its own, they would steal each other's bytes. Instead every module
 * subscribes its prefix here and calls atech_bus::poll() from its loop hook;
 * whichever call runs first reads the line and hands it to the right module.
 *
 * Header-only on purpose: identical copies live in each module folder, and the
 * inline functions (with their static state) collapse to one instance at link.
 * Keep all copies identical.
 */

#ifndef ATECH_BUS_H
#define ATECH_BUS_H

#include <Arduino.h>
#include <string.h>

namespace atech_bus {

// Returns false if the command was not understood.
typedef bool (*Handler)(void* ctx, const char* cmd, const char* value);

struct Entry {
    const char* prefix;
    Handler fn;
    void* ctx;
};

static const int MAX_SUBSCRIBERS = 8;

inline Entry* entries() {
    static Entry table[MAX_SUBSCRIBERS];
    return table;
}

inline int& subscriberCount() {
    static int n = 0;
    return n;
}

/** Route "<prefix>_<cmd>" actions to fn(ctx, cmd, value). Safe to call repeatedly. */
inline void subscribe(const char* prefix, Handler fn, void* ctx) {
    Entry* e = entries();
    int& n = subscriberCount();
    for (int i = 0; i < n; i++) {
        if (e[i].ctx == ctx && !strcmp(e[i].prefix, prefix)) return;
    }
    if (n < MAX_SUBSCRIBERS) e[n++] = { prefix, fn, ctx };
}

/** Copy the string value of "key":"..." (or a bare number) out of a JSON line. */
inline bool jsonField(const char* line, const char* key, char* out, size_t outLen) {
    char pat[24];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char* p = strstr(line, pat);
    if (!p) return false;
    p += strlen(pat);
    while (*p == ' ') p++;
    size_t n = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"' && n + 1 < outLen) {
            if (*p == '\\' && p[1]) p++;
            out[n++] = *p++;
        }
    } else {
        while (*p && *p != ',' && *p != '}' && n + 1 < outLen) out[n++] = *p++;
    }
    out[n] = 0;
    return true;
}

inline void dispatch(const char* line) {
    char action[48], value[128];
    if (!jsonField(line, "action", action, sizeof(action))) return;
    if (!jsonField(line, "value", value, sizeof(value))) value[0] = 0;

    Entry* e = entries();
    int n = subscriberCount();
    for (int i = 0; i < n; i++) {
        size_t plen = strlen(e[i].prefix);
        if (strncmp(action, e[i].prefix, plen) == 0 && action[plen] == '_') {
            const char* cmd = action + plen + 1;
            if (!e[i].fn(e[i].ctx, cmd, value)) {
                Serial.printf("{\"type\":\"log\",\"payload\":{\"message\":\"%s: unknown command %s\"}}\n",
                              e[i].prefix, cmd);
            }
            return;
        }
    }
}

/** Read any complete lines waiting on Serial and dispatch them. */
inline void poll() {
    static char rx[200];
    static int len = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') {
            if (len < (int)sizeof(rx) - 1) rx[len++] = c;
            continue;
        }
        rx[len] = 0;
        len = 0;
        dispatch(rx);
    }
}

}  // namespace atech_bus

#endif  // ATECH_BUS_H
