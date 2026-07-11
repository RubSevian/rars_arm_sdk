#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <libserial/SerialPort.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rars_arm
{

class ArmSerialPort
{
public:

    // -------------------- Формат протокола --------------------

    static constexpr std::uint8_t FrameHead = 0xAA;
    static constexpr std::uint8_t FrameTail = 0x55;

    // Полезная часть пакета.
    static constexpr std::size_t FrameDataLength = 59;

    // AA + length + 59 data + CRC16 + 55 = 64 байта.
    static constexpr std::size_t FrameSize = 1 + 1 + FrameDataLength + 2 + 1;

    using Payload = std::array<std::uint8_t, FrameDataLength>;

    using Frame = std::array<std::uint8_t, FrameSize>;

    // -------------------- Конструкторы --------------------

    /**
     * Создаёт объект с параметрами по умолчанию:
     * /dev/ttyACM0, 921600 baud.
     *
     * Сам порт конструктор не открывает.
     */
    ArmSerialPort();

    /**
     * Создаёт объект с указанным портом и скоростью.
     *
     * Сам порт конструктор не открывает.
     */
    ArmSerialPort(std::string port_name, unsigned int baud_rate);

    /**
     * При уничтожении:
     * 1. останавливает поток приёма;
     * 2. закрывает serial-порт.
     */
    ~ArmSerialPort();

    // Запрещаем копирование, потому что объект владеет serial-портом и отдельным потоком.
    ArmSerialPort(const ArmSerialPort&)			   = delete;
    ArmSerialPort& operator=(const ArmSerialPort&) = delete;

    // Пока также запрещаем перемещение.
    ArmSerialPort(ArmSerialPort&&)			  = delete;
    ArmSerialPort& operator=(ArmSerialPort&&) = delete;

    // -------------------- Настройки --------------------

    /**
     * Меняет имя порта.
     *
     * Возвращает false, если порт уже открыт.
     */
    bool setPortName(const std::string& port_name);

    /**
     * Меняет baud rate.
     *
     * Возвращает false, если порт уже открыт.
     */
    bool setBaudRate(unsigned int baud_rate);

    [[nodiscard]] const std::string& portName() const noexcept;

    [[nodiscard]] unsigned int baudRate() const noexcept;

    // -------------------- Подключение --------------------

    /**
     * Открывает serial-порт и применяет настройки.
     */
    bool open();

    /**
     * Останавливает приём и закрывает порт.
     */
    void close();

    [[nodiscard]] bool isOpen() const noexcept;

    // -------------------- Передача --------------------

    /**
     * Упаковывает payload длиной 59 байт
     * в полный кадр длиной 64 байта и отправляет его.
     */
    bool sendPayload(const Payload& payload);

    // -------------------- Приём --------------------

    /**
     * Запускает отдельный поток чтения.
     */
    bool startReceiving();

    /**
     * Останавливает поток чтения.
     */
    void stopReceiving();

    [[nodiscard]] bool isReceiving() const noexcept;

    /**
     * Копирует последний корректно принятый payload.
     *
     * Возвращает true, если после предыдущего вызова
     * был получен новый пакет.
     */
    bool getReceivedPayload(Payload& payload);

    /**
     * Возвращает последний корректный payload,
     * не сбрасывая признак новых данных.
     */
    [[nodiscard]] Payload lastReceivedPayload() const;

    /**
     * Показывает, имеется ли ещё не прочитанный пакет.
     */
    [[nodiscard]] bool hasNewPayload() const;

    /**
     * Текст последней ошибки.
     */
    [[nodiscard]] std::string lastError() const;

private:

    // -------------------- Упаковка протокола --------------------

    [[nodiscard]] Frame packFrame(const Payload& payload) const;

    [[nodiscard]] bool validateFrame(const Frame& frame, Payload& payload) const;

    /**
     * CRC16 для полезной части пакета.
     *
     * Эту функцию необходимо держать полностью
     * одинаковой на ПК и на STM32.
     */
    [[nodiscard]] static std::uint16_t calculateCrc16(const std::uint8_t* data, std::size_t length);

    // -------------------- Работа потока --------------------

    void receiveLoop();

    /**
     * Ищет начало кадра 0xAA, затем считывает
     * оставшиеся 63 байта.
     */
    bool readFrame(Frame& frame);

    // -------------------- Вспомогательные функции --------------------

    [[nodiscard]] static LibSerial::BaudRate toLibSerialBaudRate(unsigned int baud_rate);

    void setLastError(const std::string& message);

private:

    // Конфигурация.
    std::string	 port_name_ = "/dev/ttyACM0";
    unsigned int baud_rate_ = 921600;

    // Непосредственно serial-порт LibSerial.
    LibSerial::SerialPort serial_port_;

    // Поток приёма.
    std::thread		  receive_thread_;
    std::atomic<bool> receiving_{false};

    // Защита отправки, чтобы два потока одновременно
    // не писали в serial.
    mutable std::mutex write_mutex_;

    // Последний принятый payload.
    mutable std::mutex received_mutex_;
    Payload			   last_payload_{};

    // true, когда получен новый корректный пакет.
    bool new_payload_available_ = false;

    // Последняя ошибка.
    mutable std::mutex error_mutex_;
    std::string		   last_error_;
};

} // namespace rars_arm
