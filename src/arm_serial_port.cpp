#include "arm_serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace rars_arm
{
static const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039,
  0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099,
  0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA,
  0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
  0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309,
  0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359,
  0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
  0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249,
  0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
  0x0208, 0x820D, 0x8207, 0x0202};

ArmSerialPort::ArmSerialPort() = default;

ArmSerialPort::ArmSerialPort(std::string port_name, unsigned int baud_rate)
    : port_name_(std::move(port_name)),
      baud_rate_(baud_rate)
{}

ArmSerialPort::~ArmSerialPort()
{
    close();
}

// ---------------------------------------------------------
// Настройки
// ---------------------------------------------------------

bool ArmSerialPort::setPortName(const std::string& port_name)
{
    if (isOpen())
    {
        setLastError("Нельзя изменить имя порта, пока он открыт.");
        return false;
    }

    if (port_name.empty())
    {
        setLastError("Имя serial-порта не может быть пустым.");
        return false;
    }

    port_name_ = port_name;
    return true;
}

bool ArmSerialPort::setBaudRate(unsigned int baud_rate)
{
    if (isOpen())
    {
        setLastError("Нельзя изменить baud rate, пока порт открыт.");
        return false;
    }

    try
    {
        // Только проверяем, поддерживается ли скорость.
        static_cast<void>(toLibSerialBaudRate(baud_rate));
    }
    catch (const std::exception& exception)
    {
        setLastError(exception.what());
        return false;
    }

    baud_rate_ = baud_rate;
    return true;
}

const std::string& ArmSerialPort::portName() const noexcept
{
    return port_name_;
}

unsigned int ArmSerialPort::baudRate() const noexcept
{
    return baud_rate_;
}

// ---------------------------------------------------------
// Открытие и закрытие
// ---------------------------------------------------------

bool ArmSerialPort::open()
{
    if (isOpen())
        return true;

    try
    {
        serial_port_.Open(port_name_);

        serial_port_.SetBaudRate(toLibSerialBaudRate(baud_rate_));

        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);

        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

        serial_port_.FlushIOBuffers();

        setLastError("");
        return true;
    }
    catch (const std::exception& exception)
    {
        setLastError(std::string("Не удалось открыть serial-порт: ") + exception.what());

        if (serial_port_.IsOpen())
        {
            try
            {
                serial_port_.Close();
            }
            catch (...)
            {}
        }

        return false;
    }
}

void ArmSerialPort::close()
{
    stopReceiving();

    if (!serial_port_.IsOpen())
        return;

    try
    {
        serial_port_.FlushIOBuffers();
        serial_port_.Close();
    }
    catch (const std::exception& exception)
    {
        setLastError(std::string("Ошибка закрытия serial-порта: ") + exception.what());
    }
}

bool ArmSerialPort::isOpen() const noexcept
{
    return serial_port_.IsOpen();
}

// ---------------------------------------------------------
// Отправка
// ---------------------------------------------------------

bool ArmSerialPort::sendPayload(const Payload& payload)
{
    if (!isOpen())
    {
        setLastError("Невозможно отправить пакет: serial-порт не открыт.");
        return false;
    }

    const Frame frame = packFrame(payload);

    std::vector<std::uint8_t> output(frame.begin(), frame.end());

    try
    {
        std::lock_guard<std::mutex> lock(write_mutex_);

        serial_port_.Write(output);
        serial_port_.DrainWriteBuffer();

        return true;
    }
    catch (const std::exception& exception)
    {
        setLastError(std::string("Ошибка отправки serial-пакета: ") + exception.what());
        return false;
    }
}

// ---------------------------------------------------------
// Поток приёма
// ---------------------------------------------------------

bool ArmSerialPort::startReceiving()
{
    if (!isOpen())
    {
        setLastError("Нельзя запустить приём: serial-порт не открыт.");
        return false;
    }

    if (receiving_.load())
        return true;

    resetStatistics();
    receiving_.store(true);

    try
    {
        receive_thread_ = std::thread(&ArmSerialPort::receiveLoop, this);
    }
    catch (const std::exception& exception)
    {
        receiving_.store(false);

        setLastError(std::string("Не удалось создать поток приёма: ") + exception.what());

        return false;
    }

    return true;
}

void ArmSerialPort::stopReceiving()
{
    receiving_.store(false);

    if (receive_thread_.joinable())
        receive_thread_.join();
}

bool ArmSerialPort::isReceiving() const noexcept
{
    return receiving_.load();
}

void ArmSerialPort::receiveLoop()
{
    while (receiving_.load())
    {
        Frame frame{};

        if (!readFrame(frame))
        {
            // Timeout чтения не является аварией.
            continue;
        }

        Payload payload{};

        if (!validateFrame(frame, payload))
        {
            std::lock_guard<std::mutex> lock(received_mutex_);
            ++statistics_.invalid_frames;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(received_mutex_);

			last_payload_		   = payload;
			new_payload_available_ = true;
            ++statistics_.valid_frames;
            statistics_.feedback_received = true;
            statistics_.last_valid_frame = std::chrono::steady_clock::now();
        }
    }
}

// ---------------------------------------------------------
// Считывание одного полного кадра
// ---------------------------------------------------------

bool ArmSerialPort::readFrame(Frame& frame)
{
    if (!isOpen())
        return false;

    try
    {
        std::uint8_t byte = 0;

        // Сначала ищем байт начала кадра 0xAA.
        while (receiving_.load())
        {
            serial_port_.ReadByte(byte, 20);

            if (byte == FrameHead)
            {
                frame[0] = byte;
                break;
            }
        }

        if (!receiving_.load())
            return false;

        // После найденного 0xAA читаем ещё 63 байта.
        for (std::size_t i = 1; i < FrameSize; ++i)
        {
            if (!receiving_.load())
                return false;

            serial_port_.ReadByte(frame[i], 20);
        }

        return true;
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::lock_guard<std::mutex> lock(received_mutex_);
        ++statistics_.read_timeouts;
        return false;
    }
    catch (const std::exception& exception)
    {
        if (receiving_.load())
        {
            {
                std::lock_guard<std::mutex> lock(received_mutex_);
                ++statistics_.read_errors;
            }
            setLastError(std::string("Ошибка чтения serial-порта: ") + exception.what());
        }

        return false;
    }
}

// ---------------------------------------------------------
// Получение данных пользователем SDK
// ---------------------------------------------------------

bool ArmSerialPort::getReceivedPayload(Payload& payload)
{
    std::lock_guard<std::mutex> lock(received_mutex_);

    if (!new_payload_available_)
        return false;

    payload				   = last_payload_;
    new_payload_available_ = false;

    return true;
}

ArmSerialPort::Payload ArmSerialPort::lastReceivedPayload() const
{
    std::lock_guard<std::mutex> lock(received_mutex_);

    return last_payload_;
}

bool ArmSerialPort::hasNewPayload() const
{
    std::lock_guard<std::mutex> lock(received_mutex_);
    return new_payload_available_;
}

SerialStatistics ArmSerialPort::statistics() const
{
    std::lock_guard<std::mutex> lock(received_mutex_);
    return statistics_;
}

void ArmSerialPort::resetStatistics()
{
    std::lock_guard<std::mutex> lock(received_mutex_);
    statistics_ = {};
    last_payload_ = {};
    new_payload_available_ = false;
}

// ---------------------------------------------------------
// Упаковка кадра
// ---------------------------------------------------------

ArmSerialPort::Frame ArmSerialPort::packFrame(const Payload& payload) const
{
    Frame frame{};

    frame[0] = FrameHead;

    // В протоколе длина занимает один байт.
    frame[1] = static_cast<std::uint8_t>(FrameDataLength);

    // Payload начинается с индекса 2.
    std::copy(payload.begin(), payload.end(), frame.begin() + 2);

    const std::uint16_t crc = calculateCrc16(payload.data(), payload.size());

    // CRC записываем младшим байтом вперёд.
    frame[2 + FrameDataLength] = static_cast<std::uint8_t>(crc & 0xFFU);

    frame[3 + FrameDataLength] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);

    frame[FrameSize - 1] = FrameTail;

    return frame;
}

// ---------------------------------------------------------
// Проверка принятого кадра
// ---------------------------------------------------------

bool ArmSerialPort::validateFrame(const Frame& frame, Payload& payload) const
{
    if (frame[0] != FrameHead)
        return false;

    if (frame[1] != FrameDataLength)
        return false;

    if (frame[FrameSize - 1] != FrameTail)
        return false;

    std::copy(frame.begin() + 2, frame.begin() + 2 + FrameDataLength, payload.begin());

    const std::uint16_t received_crc =
      static_cast<std::uint16_t>(frame[2 + FrameDataLength])
      | (static_cast<std::uint16_t>(frame[3 + FrameDataLength]) << 8U);

    const std::uint16_t calculated_crc = calculateCrc16(payload.data(), payload.size());

    return received_crc == calculated_crc;
}

// ---------------------------------------------------------
// CRC16
// ---------------------------------------------------------

std::uint16_t ArmSerialPort::calculateCrc16(const std::uint8_t* data, std::size_t length)
{
    // Алгоритм и таблица перенесены из рабочего
    // протокола ПК <-> STM32 без изменения.

    std::uint16_t crc = 0xFFFFU;

    for (std::size_t i = 0; i < length; ++i)
    {
        const std::uint8_t index = static_cast<std::uint8_t>((crc >> 8U) ^ data[i]);

        crc = static_cast<std::uint16_t>((crc << 8U) ^ CRC16_TABLE[index]);
    }

    return crc;
}

// ---------------------------------------------------------
// Преобразование baud rate
// ---------------------------------------------------------

LibSerial::BaudRate ArmSerialPort::toLibSerialBaudRate(unsigned int baud_rate)
{
    switch (baud_rate)
    {
        case 9600:
            return LibSerial::BaudRate::BAUD_9600;

        case 19200:
            return LibSerial::BaudRate::BAUD_19200;

        case 38400:
            return LibSerial::BaudRate::BAUD_38400;

        case 57600:
            return LibSerial::BaudRate::BAUD_57600;

        case 115200:
            return LibSerial::BaudRate::BAUD_115200;

        case 230400:
            return LibSerial::BaudRate::BAUD_230400;

        case 460800:
            return LibSerial::BaudRate::BAUD_460800;

        case 921600:
            return LibSerial::BaudRate::BAUD_921600;

        default:
            throw std::invalid_argument("ArmSerialPort: неподдерживаемый baud rate: "
                                        + std::to_string(baud_rate));
    }
}

// ---------------------------------------------------------
// Ошибки
// ---------------------------------------------------------

void ArmSerialPort::setLastError(const std::string& message)
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
}

std::string ArmSerialPort::lastError() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

} // namespace rars_arm
