#pragma once

#include <memory>
#include <string>

// Raw 8N1 serial reader for newline-oriented device logs (ESP, etc.).
class SerialTextPort {
public:
    SerialTextPort();
    ~SerialTextPort();

    SerialTextPort(const SerialTextPort&) = delete;
    SerialTextPort& operator=(const SerialTextPort&) = delete;

    bool open(const std::string& port, int baudrate);
    void close();
    bool is_open() const;
    std::string port_name() const;
    int baudrate() const;

    // Bytes received since the last call (may be empty). UI thread only.
    std::string take();
    bool write(const std::string& text);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
