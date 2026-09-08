#include "SerialTextPort.hpp"

#include "../../utils/params.hpp"

#include <boost/asio.hpp>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <atomic>
#include <cstdlib>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

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

struct SerialTextPort::Impl {
    struct Session : std::enable_shared_from_this<Session> {
        explicit Session(boost::asio::io_context& ioc)
            : port(ioc) {}

        void arm_read() {
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

        void enqueue_write(std::vector<uint8_t> data) {
            if (!port.is_open() || data.empty())
                return;
            if (write_q.size() >= TX_SERIAL_DEVICE_WRITE_QUEUE_MAX)
                return;
            write_q.push_back(std::move(data));
            pump_write();
        }

        void pump_write() {
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

        boost::asio::serial_port port;
        std::atomic<bool> running{false};
        std::function<void(std::span<const uint8_t>)> on_bytes;
        std::vector<uint8_t> read_buf;
        std::deque<std::vector<uint8_t>> write_q;
        bool writing = false;
    };

    std::string display_port;
    int baud = 0;
    boost::asio::io_context ioc;
    std::optional<boost::asio::executor_work_guard<
        boost::asio::io_context::executor_type>> work;
    std::thread thread;
    std::shared_ptr<Session> session;
    std::mutex mu;
    std::string pending;

    static constexpr std::size_t kPendingMax = 1 << 20;

    Impl()
        : session(std::make_shared<Session>(ioc)) {
        session->read_buf.resize(RX_SERIAL_DEVICE_READ_BUFFER_LENGTH);
    }

    ~Impl() { close(); }

    void append(std::span<const uint8_t> bytes) {
        std::lock_guard<std::mutex> lock(mu);
        pending.append(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        if (pending.size() > kPendingMax)
            pending.erase(0, pending.size() - kPendingMax / 2);
    }

    std::string take() {
        std::lock_guard<std::mutex> lock(mu);
        std::string out;
        out.swap(pending);
        return out;
    }

    void start_io() {
        if (work) return;
        ioc.restart();
        work.emplace(boost::asio::make_work_guard(ioc.get_executor()));
        thread = std::thread([this] { ioc.run(); });
    }

    void stop_io() {
        if (!work) return;
        work.reset();
        ioc.stop();
        if (thread.joinable()) thread.join();
        session->write_q.clear();
        session->writing = false;
    }

    bool open(const std::string& port, int baudrate) {
        close();

        boost::system::error_code ec;
        session->port.open(native_port_name(port), ec);
        if (ec) return false;

        session->port.set_option(
            boost::asio::serial_port_base::baud_rate(baudrate), ec);
        if (ec) { close(); return false; }
        session->port.set_option(
            boost::asio::serial_port_base::character_size(8), ec);
        if (ec) { close(); return false; }
        session->port.set_option(
            boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none), ec);
        if (ec) { close(); return false; }
        session->port.set_option(
            boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one), ec);
        if (ec) { close(); return false; }
        session->port.set_option(
            boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none), ec);
        if (ec) { close(); return false; }

#ifdef _WIN32
        EscapeCommFunction(session->port.native_handle(), SETDTR);
        EscapeCommFunction(session->port.native_handle(), SETRTS);
#endif

        display_port = port;
        baud = baudrate;
        session->on_bytes = [this](std::span<const uint8_t> bytes) {
            append(bytes);
        };
        start_io();
        session->running = true;
        session->arm_read();
        return true;
    }

    void close() {
        session->running = false;
        boost::system::error_code ec;
        session->port.cancel(ec);
        stop_io();
        if (session->port.is_open())
            session->port.close(ec);
        display_port.clear();
        baud = 0;
        std::lock_guard<std::mutex> lock(mu);
        pending.clear();
    }

    bool is_open() const {
        return session->running && session->port.is_open();
    }
};

SerialTextPort::SerialTextPort() : impl_(std::make_unique<Impl>()) {}
SerialTextPort::~SerialTextPort() = default;

bool SerialTextPort::open(const std::string& port, int baudrate) {
    return impl_->open(port, baudrate);
}

void SerialTextPort::close() { impl_->close(); }

bool SerialTextPort::is_open() const { return impl_->is_open(); }

std::string SerialTextPort::port_name() const { return impl_->display_port; }

int SerialTextPort::baudrate() const { return impl_->baud; }

std::string SerialTextPort::take() { return impl_->take(); }

bool SerialTextPort::write(const std::string& text) {
    if (!impl_->work || !impl_->session->port.is_open() || text.empty())
        return false;
    std::vector<uint8_t> buf(text.begin(), text.end());
    auto sess = impl_->session;
    boost::asio::post(impl_->ioc, [sess, buf = std::move(buf)]() mutable {
        sess->enqueue_write(std::move(buf));
    });
    return true;
}
