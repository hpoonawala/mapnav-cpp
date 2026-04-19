#ifndef TELEMETRY_SERVER_H
#define TELEMETRY_SERVER_H

#include <asio.hpp>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <queue>
#include <tuple>
#include "pose.h"
#include "OccupancyGrid.h"

// Command types that can be received from clients
enum class CommandType {
    SET_GOAL,
    STOP,
    RESUME,
    REQUEST_MAP
};

// Command structure
struct TelemetryCommand {
    CommandType type;
    double x;
    double y;
};

// Forward declaration
class TelemetryClient;

// Main telemetry server class - NON-BLOCKING design
class TelemetryServer {
public:
    TelemetryServer(unsigned short port);
    ~TelemetryServer();

    // Start/stop server (no threads!)
    void start();
    void stop();

    // NON-BLOCKING tick - call this from your main loop
    // Accepts new connections, reads commands, cleans up dead clients
    void tick();

    // Broadcasting methods
    void publishPose(const Pose2D& pose, int scan_count);
    void publishMap(const OccupancyGrid& grid);
    void publishPath(const std::vector<std::pair<double, double>>& path);

    // Command queue access
    bool hasCommand();
    TelemetryCommand popCommand();

private:
    unsigned short port_;
    asio::io_context io_context_;
    asio::ip::tcp::acceptor acceptor_;
    bool running_;

    std::vector<std::shared_ptr<TelemetryClient>> clients_;
    std::mutex clients_mutex_;

    std::queue<TelemetryCommand> command_queue_;
    std::mutex command_mutex_;

    // Helper methods
    void tryAcceptNewClient();
    void readFromClients();
    void cleanupDisconnectedClients();
    void broadcast(const std::string& message, bool blocking = true);
    bool parseCommand(const std::string& json, TelemetryCommand& cmd);

    // Formatting methods
    std::string formatPoseJson(const Pose2D& pose, int scan_count);
    std::string formatMapJson(const OccupancyGrid& grid);
    std::string formatMapDeltaJson(int width, int height, double resolution,
                                    const std::vector<std::tuple<int,int,uint8_t>>& cells);
    std::string formatPathJson(const std::vector<std::pair<double, double>>& path);
    std::string base64Encode(const std::vector<uint8_t>& data);

    // Delta map state
    std::vector<uint8_t> last_sent_pixels_;
    int last_sent_width_ = 0;
    int last_sent_height_ = 0;
};

// Client class - represents a single connected client
class TelemetryClient {
public:
    TelemetryClient(asio::ip::tcp::socket socket);

    void send(const std::string& message, bool blocking = true);
    std::string tryReadLine();  // Non-blocking read
    bool isConnected() const;

private:
    asio::ip::tcp::socket socket_;
    bool connected_;
    std::string line_buffer_;
    std::mutex write_mutex_;
};

#endif // TELEMETRY_SERVER_H
