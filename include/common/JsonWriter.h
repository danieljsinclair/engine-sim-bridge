// JsonWriter.h - Minimal JSON writer (no external dependencies)
// Header-only implementation

#ifndef ENGINE_SIM_BRIDGE_JSON_WRITER_H
#define ENGINE_SIM_BRIDGE_JSON_WRITER_H

#include <cstdarg>
#include <cstdio>
#include <cmath>
#include <string>
#include "function.h"

class JsonWriter {
public:
    JsonWriter() : indent_(0), needsComma_(false) {}

    void beginObject() {
        maybeComma();
        append("{");
        indent_++;
        needsComma_ = false;
    }

    void endObject() {
        indent_--;
        newline();
        append("}");
        needsComma_ = true;
    }

    void beginArray() {
        maybeComma();
        append("[");
        indent_++;
        needsComma_ = false;
    }

    void endArray() {
        indent_--;
        newline();
        append("]");
        needsComma_ = true;
    }

    void key(const char* k) {
        maybeComma();
        newline();
        append("\"%s\": ", k);
        needsComma_ = false;
    }

    void value(double v) {
        maybeComma();
        // Handle NaN/Inf
        if (std::isnan(v)) append("null");
        else if (std::isinf(v)) append(v > 0 ? "1e308" : "-1e308");
        else {
            char buf[64];
            // Use enough precision to round-trip doubles
            snprintf(buf, sizeof(buf), "%.15g", v);
            append("%s", buf);
        }
        needsComma_ = true;
    }

    void value(int v) {
        maybeComma();
        append("%d", v);
        needsComma_ = true;
    }

    void value(const char* v) {
        maybeComma();
        // Escape basic JSON special chars
        append("\"");
        for (const char* p = v; *p; p++) {
            switch (*p) {
                case '"':  append("\\\""); break;
                case '\\': append("\\\\"); break;
                case '\n': append("\\n"); break;
                case '\r': append("\\r"); break;
                case '\t': append("\\t"); break;
                default:   buf_ += *p; break;
            }
        }
        append("\"");
        needsComma_ = true;
    }

    void valueBool(bool v) {
        maybeComma();
        append(v ? "true" : "false");
        needsComma_ = true;
    }

    // Convenience: write a key-value pair
    void kv(const char* k, double v) { key(k); value(v); }
    void kv(const char* k, int v) { key(k); value(v); }
    void kv(const char* k, const char* v) { key(k); value(v); }
    void kvBool(const char* k, bool v) { key(k); valueBool(v); }

    // Serialize Function with complete metadata including filterRadius
    void serializeFunction(const char* k, Function* fn) {
        if (!fn) return;
        key(k);
        beginObject();
        kv("filterRadius", fn->getFilterRadius());
        kv("inputScale", fn->getInputScale());
        kv("outputScale", fn->getOutputScale());

        key("samples");
        beginArray();
        for (int i = 0; i < fn->getSampleCount(); i++) {
            beginArray();
            value(fn->getX(i));
            value(fn->getY(i));
            endArray();
        }
        endArray();

        endObject();
    }

    std::string str() const { return buf_; }

private:
    void maybeComma() {
        if (needsComma_) {
            buf_ += ", ";
        }
    }

    void newline() {
        buf_ += '\n';
        for (int i = 0; i < indent_; i++) buf_ += "  ";
    }

    void append(const char* fmt, ...) {
        char tmp[512];
        va_list args;
        va_start(args, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, args);
        va_end(args);
        buf_ += tmp;
    }

    std::string buf_;
    int indent_;
    bool needsComma_;
};

#endif // ENGINE_SIM_BRIDGE_JSON_WRITER_H