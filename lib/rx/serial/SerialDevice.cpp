#include "SerialDevice.hpp"

#include "../msg_queue/RxMessage.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <sys/ioctl.h>
#endif
#include <cstdlib>

namespace {

std::string native_port_name(std::string port) {
#ifdef _WIN32
    if (port.rfind("\\\\.\\", 0) == 0) return port;
    if (port.rfind("COM", 0) == 0) {
        const int n = std::atoi(port.c_str() + 3);
        if (n >= 10)
            return "\\\\.\\" + port;
    }
#endif
    return port;
}

}  // namespace

SerialDevice::SerialDevice(std::string port, int baudrate, MessageQueue& queue,
                           IFrameParser* parser, bool assert_dtr_rts)
    : port_name_(native_port_name(std::move(port)))
    , baudrate_(baudrate)
    , assert_dtr_rts_(assert_dtr_rts)
    , queue_(queue)
    , session_(std::make_shared<Session>(ioc_))
    , parser_(parser ? parser : &owned_parser_) {
    session_->read_buf.resize(RX_SERIAL_DEVICE_READ_BUFFER_LENGTH);
    parser_->set_buffer(
        ring_[current_slot_].data.data(),
        static_cast<uint32_t>(ring_[current_slot_].data.size()));
    parser_->set_on_frame([this](uint8_t cmd, uint16_t len) {
        publish_frame(cmd, len);
    });
}

SerialDevice::~SerialDevice() {
    close();
    wait_for_in_flight();
}

bool SerialDevice::open() {
    if (session_->port.is_open()) return true;

    boost::system::error_code ec;
    session_->port.open(port_name_, ec);
    if (ec) return false;

    session_->port.set_option(
        boost::asio::serial_port_base::baud_rate(baudrate_), ec);
    if (ec) { close(); return false; }
    session_->port.set_option(
        boost::asio::serial_port_base::character_size(8), ec);
    if (ec) { close(); return false; }
    session_->port.set_option(
        boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none), ec);
    if (ec) { close(); return false; }
    session_->port.set_option(
        boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one), ec);
    if (ec) { close(); return false; }
    session_->port.set_option(
        boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none), ec);
    if (ec) { close(); return false; }

#ifdef _WIN32
    const auto handle = session_->port.native_handle();
    EscapeCommFunction(handle, assert_dtr_rts_ ? SETDTR : CLRDTR);
    EscapeCommFunction(handle, assert_dtr_rts_ ? SETRTS : CLRRTS);
    if (!assert_dtr_rts_)
        PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
#elif defined(__linux__)
    int bits = TIOCM_DTR | TIOCM_RTS;
    ioctl(session_->port.native_handle(),
          assert_dtr_rts_ ? TIOCMBIS : TIOCMBIC, &bits);
#endif

    return true;
}

void SerialDevice::close() {
    stop();
    if (!session_->port.is_open()) return;
    boost::system::error_code ec;
    session_->port.close(ec);
}

void SerialDevice::start() {
    if (session_->running || !session_->port.is_open()) return;
    start_io_thread();
    session_->on_bytes = [this](std::span<const uint8_t> bytes) {
        for (uint8_t b : bytes)
            parser_->pump(b);
    };
    session_->running = true;
    session_->arm_read();
}

void SerialDevice::stop() {
    session_->running = false;
    boost::system::error_code ec;
    session_->port.cancel(ec);
    stop_io_thread();
}

bool SerialDevice::write(std::span<const uint8_t> data) {
    if (!work_ || !session_->port.is_open() || data.empty())
        return false;
    std::vector<uint8_t> buf(data.begin(), data.end());
    auto sess = session_;
    boost::asio::post(ioc_, [sess, buf = std::move(buf)]() mutable {
        sess->enqueue_write(std::move(buf));
    });
    return true;
}

void SerialDevice::start_io_thread() {
    if (work_) return;
    ioc_.restart();
    work_.emplace(boost::asio::make_work_guard(ioc_.get_executor()));
    thread_ = std::thread([this] { ioc_.run(); });
}

void SerialDevice::stop_io_thread() {
    if (!work_) return;
    work_.reset();
    ioc_.stop();
    if (thread_.joinable()) thread_.join();
    session_->write_q.clear();
    session_->writing = false;
}

void SerialDevice::publish_frame(uint8_t cmd, uint16_t len) {
    const std::size_t next = find_free_slot();
    if (next == kRingSlots)
        return;

    Slot& slot = ring_[current_slot_];
    slot.in_use.store(true, std::memory_order_release);

    RxMessage msg;
    msg.cmd = cmd;
    msg.len = len;
    msg.payload = slot.data.data();
    msg.lease = SlotRelease(&slot.in_use);
    queue_.push(std::move(msg));

    current_slot_ = next;
    parser_->set_buffer(
        ring_[current_slot_].data.data(),
        static_cast<uint32_t>(ring_[current_slot_].data.size()));
}

std::size_t SerialDevice::find_free_slot() const {
    for (std::size_t n = 1; n < kRingSlots; ++n) {
        const std::size_t i = (current_slot_ + n) % kRingSlots;
        if (!ring_[i].in_use.load(std::memory_order_acquire))
            return i;
    }
    return kRingSlots;
}

void SerialDevice::wait_for_in_flight() {
    for (;;) {
        bool busy = false;
        for (auto& slot : ring_) {
            if (slot.in_use.load(std::memory_order_acquire)) {
                busy = true;
                break;
            }
        }
        if (!busy) return;
        std::this_thread::yield();
    }
}

void SerialDevice::Session::arm_read() {
    auto self = shared_from_this();
    port.async_read_some(
        boost::asio::buffer(read_buf),
        [self](const boost::system::error_code& ec, std::size_t n) {
            if (!self->running) return;
            if (ec) {
                if (ec == boost::asio::error::operation_aborted) return;
                self->arm_read();
                return;
            }
            if (n > 0 && self->on_bytes)
                self->on_bytes({self->read_buf.data(), n});
            if (self->running)
                self->arm_read();
        });
}

void SerialDevice::Session::enqueue_write(std::vector<uint8_t> data) {
    if (!port.is_open() || data.empty())
        return;
    if (write_q.size() >= TX_SERIAL_DEVICE_WRITE_QUEUE_MAX)
        return;
    write_q.push_back(std::move(data));
    pump_write();
}

void SerialDevice::Session::pump_write() {
    if (writing || write_q.empty() || !port.is_open())
        return;
    writing = true;
    auto self = shared_from_this();
    boost::asio::async_write(
        port,
        boost::asio::buffer(write_q.front()),
        [self](const boost::system::error_code& ec, std::size_t) {
            self->writing = false;
            if (!self->write_q.empty())
                self->write_q.pop_front();
            if (ec) {
                self->write_q.clear();
                return;
            }
            self->pump_write();
        });
}
