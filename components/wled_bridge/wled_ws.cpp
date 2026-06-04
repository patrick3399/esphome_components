#ifdef USE_ESP32
#include "wled_ws.h"
#include "wled_bridge.h"
#include "wled_json.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstring>

namespace esphome {
namespace wled_bridge {

static const char *const TAG = "wled_bridge.ws";

// Maximum incoming frame payload we will process (guard against huge messages).
static constexpr size_t WS_MAX_RECV = 4096;
static constexpr size_t WS_MAX_CLIENTS = 4;

static bool receive_frame_payload(httpd_req_t *req, httpd_ws_frame_t *frame, std::vector<uint8_t> *buf) {
  if (frame == nullptr || buf == nullptr)
    return false;
  buf->assign(frame->len + 1, 0);
  frame->payload = buf->data();
  return httpd_ws_recv_frame(req, frame, frame->len) == ESP_OK;
}

WLEDWsHandler::WLEDWsHandler(WLEDBridgeComponent *comp, WLEDJsonHandler *json_handler, httpd_handle_t server)
    : comp_(comp), json_handler_(json_handler), server_(server) {}

void WLEDWsHandler::register_handler() {
  httpd_uri_t ws_uri = {};
  ws_uri.uri = "/ws";
  ws_uri.method = HTTP_GET;
  ws_uri.handler = WLEDWsHandler::ws_handler_;
  ws_uri.user_ctx = this;
  ws_uri.is_websocket = true;
  ws_uri.handle_ws_control_frames = false;

  esp_err_t err = httpd_register_uri_handler(this->server_, &ws_uri);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register /ws handler: %d", err);
  } else {
    ESP_LOGD(TAG, "WebSocket /ws registered");
  }
}

esp_err_t WLEDWsHandler::ws_handler_(httpd_req_t *req) {
  auto *self = static_cast<WLEDWsHandler *>(req->user_ctx);

  if (req->method == HTTP_GET) {
    // New WebSocket connection upgrade
    int fd = httpd_req_to_sockfd(req);
    self->on_client_connect_(fd);
    return ESP_OK;
  }

  // Probe frame length first (payload = nullptr, len = 0)
  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
  if (ret != ESP_OK)
    return ret;

  if (frame.len > WS_MAX_RECV) {
    int fd = httpd_req_to_sockfd(req);
    ESP_LOGW(TAG, "WS frame too large (%u bytes), closing fd=%d", static_cast<unsigned>(frame.len), fd);
    httpd_sess_trigger_close(self->server_, fd);
    return ESP_OK;
  }

  if (frame.type == HTTPD_WS_TYPE_CLOSE) {
    int fd = httpd_req_to_sockfd(req);
    auto *fds = &self->client_fds_;
    fds->erase(std::remove(fds->begin(), fds->end(), fd), fds->end());
    ESP_LOGV(TAG, "WS client fd=%d disconnected", fd);
    httpd_sess_trigger_close(self->server_, fd);
    return ESP_OK;
  }

  if (frame.type == HTTPD_WS_TYPE_PING) {
    if (frame.len > 125) {
      int fd = httpd_req_to_sockfd(req);
      ESP_LOGW(TAG, "WS ping frame too large (%u bytes), closing fd=%d", static_cast<unsigned>(frame.len), fd);
      httpd_sess_trigger_close(self->server_, fd);
      return ESP_OK;
    }
    std::vector<uint8_t> ping_buf;
    if (frame.len > 0 && !receive_frame_payload(req, &frame, &ping_buf))
      return ESP_FAIL;
    httpd_ws_frame_t pong = {};
    pong.type = HTTPD_WS_TYPE_PONG;
    pong.final = true;
    pong.payload = frame.payload;
    pong.len = frame.len;
    httpd_ws_send_frame(req, &pong);
    return ESP_OK;
  }

  if (frame.type == HTTPD_WS_TYPE_PONG) {
    std::vector<uint8_t> pong_buf;
    if (frame.len > 0 && !receive_frame_payload(req, &frame, &pong_buf))
      return ESP_FAIL;
    return ESP_OK;
  }

  if (frame.type != HTTPD_WS_TYPE_TEXT) {
    int fd = httpd_req_to_sockfd(req);
    ESP_LOGW(TAG, "Unsupported WS frame type %u, closing fd=%d", static_cast<unsigned>(frame.type), fd);
    httpd_sess_trigger_close(self->server_, fd);
    return ESP_OK;
  }

  if (frame.len == 0)
    return ESP_OK;

  // Allocate and receive payload
  std::vector<uint8_t> buf;
  if (!receive_frame_payload(req, &frame, &buf))
    return ESP_FAIL;

  std::string body(reinterpret_cast<const char *>(frame.payload), frame.len);
  self->json_handler_->apply_body_str(body);
  return ESP_OK;
}

void WLEDWsHandler::on_client_connect_(int fd) {
  if (std::find(this->client_fds_.begin(), this->client_fds_.end(), fd) != this->client_fds_.end()) {
    this->send_to_client_(fd, build_state_json(this->comp_));
    return;
  }
  if (this->client_fds_.size() >= WS_MAX_CLIENTS) {
    ESP_LOGW(TAG, "WS client limit reached (%u), closing fd=%d", static_cast<unsigned>(WS_MAX_CLIENTS), fd);
    httpd_sess_trigger_close(this->server_, fd);
    return;
  }
  this->client_fds_.push_back(fd);
  ESP_LOGV(TAG, "WS client fd=%d connected (%zu total)", fd, this->client_fds_.size());

  // Send current state immediately
  std::string json = build_state_json(this->comp_);
  this->send_to_client_(fd, json);
}

void WLEDWsHandler::broadcast_state() {
  if (this->client_fds_.empty())
    return;

  std::string json = build_state_json(this->comp_);

  httpd_ws_frame_t frame = {};
  frame.final = true;
  frame.type = HTTPD_WS_TYPE_TEXT;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  frame.payload = reinterpret_cast<uint8_t *>(const_cast<char *>(json.c_str()));
  frame.len = json.size();

  std::vector<int> alive;
  alive.reserve(this->client_fds_.size());
  for (int fd : this->client_fds_) {
    esp_err_t ret = httpd_ws_send_frame_async(this->server_, fd, &frame);
    if (ret == ESP_OK) {
      alive.push_back(fd);
    } else {
      ESP_LOGV(TAG, "WS client fd=%d removed (err %d)", fd, ret);
    }
  }
  this->client_fds_ = std::move(alive);
}

void WLEDWsHandler::send_to_client_(int fd, const std::string &json) {
  httpd_ws_frame_t frame = {};
  frame.final = true;
  frame.type = HTTPD_WS_TYPE_TEXT;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  frame.payload = reinterpret_cast<uint8_t *>(const_cast<char *>(json.c_str()));
  frame.len = json.size();

  esp_err_t ret = httpd_ws_send_frame_async(this->server_, fd, &frame);
  if (ret != ESP_OK) {
    ESP_LOGV(TAG, "WS initial send fd=%d failed (err %d), removing", fd, ret);
    auto &fds = this->client_fds_;
    fds.erase(std::remove(fds.begin(), fds.end(), fd), fds.end());
  }
}

}  // namespace wled_bridge
}  // namespace esphome
#endif  // USE_ESP32
