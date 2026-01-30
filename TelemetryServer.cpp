#include "TelemetryServer.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>

// Base64 encoding table
static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// ============================================================================
// TelemetryClient Implementation
// ============================================================================

TelemetryClient::TelemetryClient(asio::ip::tcp::socket socket)
    : socket_(std::move(socket)), connected_(true) {
    // Set socket to non-blocking mode
    socket_.non_blocking(true);
}

void TelemetryClient::send(const std::string& message, bool blocking) {
    if (!connected_) return;

    std::lock_guard<std::mutex> lock(write_mutex_);
    try {
        if (blocking) {
            // Temporarily set to blocking mode for writes to handle large messages
            socket_.non_blocking(false);
            asio::write(socket_, asio::buffer(message + "\n"));
            // Set back to non-blocking for reads
            socket_.non_blocking(true);
        } else {
            // Non-blocking write - drop message if it would block
            asio::error_code ec;
            asio::write(socket_, asio::buffer(message + "\n"), ec);
            if (ec == asio::error::would_block) {
                // Message dropped - next one comes soon
            } else if (ec) {
                connected_ = false;
                std::cerr << "TelemetryClient send error: " << ec.message() << std::endl;
            }
        }
    } catch (const std::exception& e) {
        connected_ = false;
        std::cerr << "TelemetryClient send error: " << e.what() << std::endl;
    }
}

std::string TelemetryClient::tryReadLine() {
    if (!connected_) return "";

    try {
        // Non-blocking read
        size_t avail = socket_.available();
        if (avail == 0) return "";

        std::vector<char> buf(avail);
        size_t n = socket_.read_some(asio::buffer(buf));
        line_buffer_.append(buf.data(), n);

        size_t pos = line_buffer_.find('\n');
        if (pos == std::string::npos) return "";

        std::string line = line_buffer_.substr(0, pos);
        line_buffer_.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        return line;
    } catch (const std::exception& e) {
        connected_ = false;
        return "";
    }
}

bool TelemetryClient::isConnected() const {
    return connected_;
}

// ============================================================================
// TelemetryServer Implementation
// ============================================================================

TelemetryServer::TelemetryServer(unsigned short port)
    : port_(port),
      acceptor_(io_context_),
      running_(false) {
    // Set up acceptor with reuse_address
    acceptor_.open(asio::ip::tcp::v4());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port));
    acceptor_.listen();
    
    // Make acceptor non-blocking
    acceptor_.non_blocking(true);
}

TelemetryServer::~TelemetryServer() {
    stop();
}

void TelemetryServer::start() {
    if (running_) return;
    running_ = true;
    std::cout << "TelemetryServer started on port " << port_ << " (non-blocking mode)" << std::endl;
}

void TelemetryServer::stop() {
    if (!running_) return;
    running_ = false;

    // Close acceptor and all clients
    try {
        acceptor_.close();
    } catch (...) {}

    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.clear();
    }

    std::cout << "TelemetryServer stopped" << std::endl;
}

// NON-BLOCKING tick method - call this from your main loop!
void TelemetryServer::tick() {
    if (!running_) return;

    tryAcceptNewClient();
    readFromClients();
    cleanupDisconnectedClients();
}

void TelemetryServer::tryAcceptNewClient() {
    try {
        asio::ip::tcp::socket socket(io_context_);
        asio::error_code ec;
        
        // Non-blocking accept
        acceptor_.accept(socket, ec);
        
        if (!ec) {
            // Successfully accepted a new client
            auto client = std::make_shared<TelemetryClient>(std::move(socket));
            
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.push_back(client);
                std::cout << "TelemetryServer: client connected (total: "
                          << clients_.size() << ")" << std::endl;
            }
        }
        // If ec == would_block, no client was waiting - that's fine
    } catch (const std::exception& e) {
        // Ignore errors in non-blocking accept
    }
}

void TelemetryServer::readFromClients() {
    // Snapshot client list
    std::vector<std::shared_ptr<TelemetryClient>> snapshot;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        snapshot = clients_;
    }

    // Read from each client (non-blocking)
    for (auto& client : snapshot) {
        if (!client->isConnected()) continue;
        
        std::string line = client->tryReadLine();
        if (!line.empty()) {
            TelemetryCommand cmd;
            if (parseCommand(line, cmd)) {
                std::lock_guard<std::mutex> cmd_lock(command_mutex_);
                command_queue_.push(cmd);
            }
        }
    }
}

void TelemetryServer::cleanupDisconnectedClients() {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = clients_.begin();
    while (it != clients_.end()) {
        if (!(*it)->isConnected()) {
            it = clients_.erase(it);
            std::cout << "TelemetryServer: client disconnected" << std::endl;
        } else {
            ++it;
        }
    }
}

bool TelemetryServer::parseCommand(const std::string& json, TelemetryCommand& cmd) {
    // Simple JSON parsing without external library
    // Expected formats:
    // {"command":"set_goal","x":2.3,"y":-0.5}
    // {"command":"stop"}
    // {"command":"resume"}
    // {"command":"request_map"}

    if (json.find("\"set_goal\"") != std::string::npos) {
        cmd.type = CommandType::SET_GOAL;

        // Extract x value
        size_t x_pos = json.find("\"x\"");
        if (x_pos != std::string::npos) {
            size_t colon = json.find(':', x_pos);
            if (colon != std::string::npos) {
                size_t end = json.find_first_of(",}", colon);
                std::string x_str = json.substr(colon + 1, end - colon - 1);
                cmd.x = std::stod(x_str);
            }
        }

        // Extract y value
        size_t y_pos = json.find("\"y\"");
        if (y_pos != std::string::npos) {
            size_t colon = json.find(':', y_pos);
            if (colon != std::string::npos) {
                size_t end = json.find_first_of(",}", colon);
                std::string y_str = json.substr(colon + 1, end - colon - 1);
                cmd.y = std::stod(y_str);
            }
        }

        std::cout << "TelemetryServer: SET_GOAL command x=" << cmd.x << " y=" << cmd.y << std::endl;
        return true;
    } else if (json.find("\"stop\"") != std::string::npos) {
        cmd.type = CommandType::STOP;
        std::cout << "TelemetryServer: STOP command" << std::endl;
        return true;
    } else if (json.find("\"resume\"") != std::string::npos) {
        cmd.type = CommandType::RESUME;
        std::cout << "TelemetryServer: RESUME command" << std::endl;
        return true;
    } else if (json.find("\"request_map\"") != std::string::npos) {
        cmd.type = CommandType::REQUEST_MAP;
        std::cout << "TelemetryServer: REQUEST_MAP command" << std::endl;
        return true;
    }

    return false;
}

void TelemetryServer::broadcast(const std::string& message, bool blocking) {
    std::vector<std::shared_ptr<TelemetryClient>> snapshot;
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        snapshot = clients_;
    }
    for (auto& client : snapshot) {
        if (client->isConnected()) {
            client->send(message, blocking);
        }
    }
}

void TelemetryServer::publishPose(const Pose2D& pose, int scan_count) {
    broadcast(formatPoseJson(pose, scan_count), false);
}

void TelemetryServer::publishMap(const OccupancyGrid& grid) {
    auto size = grid.getGridSize();
    int width = size.first;
    int height = size.second;
    double resolution = grid.getResolution();

    // Build current pixel data
    auto prob_grid = grid.probabilityGrid();
    std::vector<uint8_t> pixels;
    pixels.reserve(width * height);
    for (int j = height - 1; j >= 0; j--) {
        for (int i = 0; i < width; i++) {
            pixels.push_back(static_cast<uint8_t>(prob_grid[i][j] * 255.0));
        }
    }

    bool send_full = (last_sent_width_ != width || last_sent_height_ != height
                      || last_sent_pixels_.empty());

    if (!send_full) {
        // Compute diff
        std::vector<std::tuple<int,int,uint8_t>> changed;
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                int idx = j * width + i;
                if (pixels[idx] != last_sent_pixels_[idx]) {
                    changed.emplace_back(i, j, pixels[idx]);
                }
            }
        }

        size_t total = static_cast<size_t>(width) * height;
        if (changed.size() > total / 2) {
            send_full = true;
        } else {
            // Send delta (blocking to prevent partial writes corrupting the stream)
            broadcast(formatMapDeltaJson(width, height, resolution, changed), true);
        }
    }

    if (send_full) {
        broadcast(formatMapJson(grid), true);
    }

    last_sent_pixels_ = std::move(pixels);
    last_sent_width_ = width;
    last_sent_height_ = height;
}

void TelemetryServer::publishPath(const std::vector<std::pair<double, double>>& path) {
    broadcast(formatPathJson(path), false);
}

bool TelemetryServer::hasCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return !command_queue_.empty();
}

TelemetryCommand TelemetryServer::popCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_queue_.empty()) {
        return TelemetryCommand{CommandType::STOP, 0, 0};
    }
    TelemetryCommand cmd = command_queue_.front();
    command_queue_.pop();
    return cmd;
}

std::string TelemetryServer::base64Encode(const std::vector<uint8_t>& data) {
    std::string encoded;
    encoded.reserve(((data.size() + 2) / 3) * 4);

    size_t i = 0;
    while (i < data.size()) {
        uint32_t octet_a = i < data.size() ? data[i++] : 0;
        uint32_t octet_b = i < data.size() ? data[i++] : 0;
        uint32_t octet_c = i < data.size() ? data[i++] : 0;

        uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

        encoded.push_back(base64_chars[(triple >> 18) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 12) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 6) & 0x3F]);
        encoded.push_back(base64_chars[triple & 0x3F]);
    }

    // Add padding
    size_t padding = (3 - (data.size() % 3)) % 3;
    for (size_t p = 0; p < padding; p++) {
        encoded[encoded.size() - 1 - p] = '=';
    }

    return encoded;
}

std::string TelemetryServer::formatPoseJson(const Pose2D& pose, int scan_count) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "{\"type\":\"pose\",\"scan_count\":" << scan_count
        << ",\"pose\":{\"x\":" << pose.getX()
        << ",\"y\":" << pose.getY()
        << ",\"theta\":" << pose.getTheta() << "}}";
    return oss.str();
}

std::string TelemetryServer::formatMapJson(const OccupancyGrid& grid) {
    // Get grid dimensions
    auto size = grid.getGridSize();
    int width = size.first;
    int height = size.second;
    double resolution = grid.getResolution();

    // Convert probability grid to PGM-like bytes
    auto prob_grid = grid.probabilityGrid();
    std::vector<uint8_t> pgm_data;
    pgm_data.reserve(width * height);

    for (int j = height - 1; j >= 0; j--) {  // Flip Y for image coordinates
        for (int i = 0; i < width; i++) {
            // Convert probability to grayscale (0-255)
            // High probability of free = white (255), occupied = black (0)
            uint8_t pixel = static_cast<uint8_t>(prob_grid[i][j] * 255.0);
            pgm_data.push_back(pixel);
        }
    }

    // Base64 encode the data
    std::string encoded_data = base64Encode(pgm_data);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "{\"type\":\"map\",\"width\":" << width
        << ",\"height\":" << height
        << ",\"resolution\":" << resolution
        << ",\"data\":\"" << encoded_data << "\"}";
    return oss.str();
}

std::string TelemetryServer::formatMapDeltaJson(int width, int height, double resolution,
                                                  const std::vector<std::tuple<int,int,uint8_t>>& cells) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "{\"type\":\"map_delta\",\"width\":" << width
        << ",\"height\":" << height
        << ",\"resolution\":" << resolution
        << ",\"cells\":[";

    for (size_t k = 0; k < cells.size(); k++) {
        if (k > 0) oss << ",";
        oss << "[" << std::get<0>(cells[k])
            << "," << std::get<1>(cells[k])
            << "," << static_cast<int>(std::get<2>(cells[k])) << "]";
    }

    oss << "]}";
    return oss.str();
}

std::string TelemetryServer::formatPathJson(const std::vector<std::pair<double, double>>& path) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "{\"type\":\"path\",\"waypoints\":[";

    for (size_t i = 0; i < path.size(); i++) {
        if (i > 0) oss << ",";
        oss << "{\"x\":" << path[i].first << ",\"y\":" << path[i].second << "}";
    }

    oss << "]}";
    return oss.str();
}
