#ifndef ARDUINO_H
#define ARDUINO_H

#include <chrono>
#include <iostream>
#include <stdint.h>
#include <string>

using String = std::string;

inline unsigned long millis() {
    using namespace std::chrono;
    static auto start = steady_clock::now();
    return duration_cast<milliseconds>(steady_clock::now() - start).count();
}

class HardwareSerial {
  public:
    template <typename T>
    void print(const T &val) { std::cout << val; }
    template <typename T>
    void print(const T &val, int) { std::cout << val; }
    template <typename T>
    void println(const T &val) { std::cout << val << std::endl; }
    template <typename T>
    void println(const T &val, int) { std::cout << val << std::endl; }
};

static HardwareSerial Serial;

#endif /**< ARDUINO_H */
