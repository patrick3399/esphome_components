#include "ed047tc1.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/gpio.h" // For InternalGPIOPin
#include <driver/gpio.h>       // For ESP-IDF's gpio_num_t
#include "esp_heap_caps.h"

#include "output_lcd/lcd_driver.h" // For LcdEpdConfig_t

namespace esphome {
namespace ed047tc1 {

static const char *const TAG = "ed047tc1";
ED047TC1Display* ED047TC1Display::instance = nullptr;

static inline gpio_num_t esphome_pin_to_gpio_num(GPIOPin *pin_obj) {
    if (!pin_obj) {
        ESP_LOGE(TAG, "esphome_pin_to_gpio_num: pin_obj is null!");
        return (gpio_num_t)-1;
    }
    InternalGPIOPin *internal_pin = static_cast<InternalGPIOPin*>(pin_obj);
    if (!internal_pin) {
        ESP_LOGE(TAG, "esphome_pin_to_gpio_num: Failed to cast to InternalGPIOPin.");
        return (gpio_num_t)-1;
    }
    return (gpio_num_t)internal_pin->get_pin();
}

static void custom_board_init_callback(uint32_t epd_row_width_param) {
    if (!ED047TC1Display::instance) { ESP_LOGE(TAG, "board_init_cb: Instance is null!"); return; }
    ED047TC1Display* self = ED047TC1Display::instance;
    ESP_LOGI(TAG, "Custom board_init_callback called.");

    if (self->pwr_pin_) self->pwr_pin_->digital_write(false);
    if (self->bst_en_pin_) self->bst_en_pin_->digital_write(false);

    const EpdDisplay_t* display_model = epd_get_display();
    if (!display_model) { ESP_LOGE(TAG, "board_init_cb: epd_get_display() null."); return; }

    lcd_bus_config_t bus_cfg;
    memset(&bus_cfg, 0xFF, sizeof(lcd_bus_config_t));
    for (int i = 0; i < 8; ++i) { bus_cfg.data[i] = esphome_pin_to_gpio_num(self->d_pins_[i]); }

    bus_cfg.clock       = esphome_pin_to_gpio_num(self->pclk_pin_ ? self->pclk_pin_ : self->xstl_pin_);
    bus_cfg.start_pulse = esphome_pin_to_gpio_num(self->xstl_pin_);
    bus_cfg.leh         = esphome_pin_to_gpio_num(self->xle_pin_);
    bus_cfg.stv         = esphome_pin_to_gpio_num(self->spv_pin_);
    bus_cfg.ckv         = esphome_pin_to_gpio_num(self->ckv_pin_);

    LcdEpdConfig_t lcd_epd_cfg;
    lcd_epd_cfg.pixel_clock = (size_t)(display_model->bus_speed > 0 ? display_model->bus_speed * 1000000 : 16000000);
    if (self->pclk_pin_ == nullptr && self->xstl_pin_ == nullptr) { ESP_LOGE(TAG, "Critical: No valid Pixel Clock source!"); }
    else if (self->pclk_pin_ == nullptr) { ESP_LOGW(TAG, "Warning: Using XSTL_PIN as PCLK source."); }

    lcd_epd_cfg.ckv_high_time    = 70;
    lcd_epd_cfg.line_front_porch = 4;
    lcd_epd_cfg.le_high_time     = 4;
    lcd_epd_cfg.bus_width        = 8;
    lcd_epd_cfg.bus              = bus_cfg;

    epd_lcd_init(&lcd_epd_cfg, display_model->width, display_model->height);
    ESP_LOGI(TAG, "epd_lcd_init called. PCLK: %u Hz.", (unsigned int)lcd_epd_cfg.pixel_clock);
}

static void custom_board_deinit_callback(void) {
    ESP_LOGI(TAG, "Custom board_deinit_callback called.");
    epd_lcd_deinit();
    if (!ED047TC1Display::instance) return;
    ED047TC1Display* self = ED047TC1Display::instance;
    if (self->pwr_pin_) self->pwr_pin_->digital_write(false);
    if (self->bst_en_pin_) self->bst_en_pin_->digital_write(false);
}

static void custom_board_set_ctrl_callback(epd_ctrl_state_t *state, const epd_ctrl_state_t *mask) {
    if (!ED047TC1Display::instance) return;
    ED047TC1Display* self = ED047TC1Display::instance;
    if (mask->ep_sth && self->xstl_pin_) self->xstl_pin_->digital_write(state->ep_sth);
    if (mask->ep_stv && self->spv_pin_) self->spv_pin_->digital_write(state->ep_stv);
    if (mask->ep_latch_enable && self->xle_pin_) self->xle_pin_->digital_write(state->ep_latch_enable);
}

static void custom_board_poweron_callback(epd_ctrl_state_t *state) {
    if (!ED047TC1Display::instance) return;
    ED047TC1Display* self = ED047TC1Display::instance;
    ESP_LOGI(TAG, "Custom board_poweron_callback called.");
    if (self->pwr_pin_) self->pwr_pin_->digital_write(true);
    esphome::delayMicroseconds(200);
    if (self->bst_en_pin_) self->bst_en_pin_->digital_write(true);
    esphome::delayMicroseconds(200);
    if (self->spv_pin_) self->spv_pin_->digital_write(true);
    if (self->xstl_pin_) self->xstl_pin_->digital_write(true);
}

static void custom_board_poweroff_callback(epd_ctrl_state_t *state) {
    if (!ED047TC1Display::instance) return;
    ED047TC1Display* self = ED047TC1Display::instance;
    ESP_LOGI(TAG, "Custom board_poweroff_callback called.");
    if (self->bst_en_pin_) self->bst_en_pin_->digital_write(false);
    esphome::delayMicroseconds(50);
    if (self->pwr_pin_) self->pwr_pin_->digital_write(false);
    esphome::delayMicroseconds(200);
    if (self->spv_pin_) self->spv_pin_->digital_write(false);
    if (self->xstl_pin_) self->xstl_pin_->digital_write(false);
    if (self->xle_pin_) self->xle_pin_->digital_write(false);
    if (self->ckv_pin_) self->ckv_pin_->digital_write(false);
    for (int i=0; i<8; ++i) { if (self->d_pins_[i]) self->d_pins_[i]->digital_write(false); }
}

static float custom_board_get_temperature_callback() { return 25.0f; }

static const EpdBoardDefinition esphome_ed047tc1_board_definition = {
    .init = custom_board_init_callback,
    .deinit = custom_board_deinit_callback,
    .set_ctrl = custom_board_set_ctrl_callback,
    .poweron = custom_board_poweron_callback,
    .measure_vcom = nullptr,
    .poweroff = custom_board_poweroff_callback,
    .set_vcom = nullptr,
    .get_temperature = custom_board_get_temperature_callback,
    .gpio_set_direction = nullptr,
    .gpio_read = nullptr,
    .gpio_write = nullptr,
};

void ED047TC1Display::setup() {
    ED047TC1Display::instance = this;
    ESP_LOGCONFIG(TAG, "Setting up ED047TC1 display component...");
    auto setup_pin = [&](GPIOPin* pin_obj, const char* name) {
        if (pin_obj) { pin_obj->setup(); }
        else { ESP_LOGE(TAG, "%s not configured!", name); this->mark_failed(); }
    };
    setup_pin(this->pwr_pin_, "PWR pin");
    setup_pin(this->bst_en_pin_, "BST_EN pin");
    setup_pin(this->xstl_pin_, "XSTL_PIN (EPD CLK/STRD for LCD STH/DE)");
    if (this->pclk_pin_) this->pclk_pin_->setup();
    setup_pin(this->xle_pin_, "XLE_PIN (EPD LE for LCD LEH)");
    setup_pin(this->spv_pin_, "SPV_PIN (EPD SPV for LCD STV)");
    setup_pin(this->ckv_pin_, "CKV_PIN (EPD CKV for LCD CKV)");
    for (int i = 0; i < 8; ++i) { char pin_name[10]; sprintf(pin_name, "D%d pin", i); setup_pin(this->d_pins_[i], pin_name); }
    if (this->is_failed()) return;

    epd_init(&esphome_ed047tc1_board_definition, &ED047TC2, EPD_LUT_64K);
    ESP_LOGI(TAG, "epd_init() called.");

    this->hl_state_ = epd_hl_init(EPD_BUILTIN_WAVEFORM);
    if (epd_hl_get_framebuffer(&this->hl_state_) == nullptr) { ESP_LOGE(TAG, "Failed to init epdiy high-level state!"); this->mark_failed(); return; }
    ESP_LOGI(TAG, "epd_hl_init() successful. EPDiy FB: %p", (void*)epd_hl_get_framebuffer(&this->hl_state_));

    this->esphome_buffer_size_ = (this->get_width_internal() * this->get_height_internal() * this->get_bpp()) / 8;
    this->init_internal_(this->esphome_buffer_size_);
    if (this->buffer_ == nullptr) { ESP_LOGE(TAG, "Could not allocate ESPHome DisplayBuffer!"); this->mark_failed(); return; }
    memset(this->buffer_, 0xFF, this->esphome_buffer_size_);
    ESP_LOGI(TAG, "ESPHome buffer allocated: %p, size %u.", (void*)this->buffer_, (unsigned int)this->esphome_buffer_size_);

    ESP_LOGI(TAG, "Initial full clear...");
    epd_poweron();
    epd_fullclear(&this->hl_state_, epd_ambient_temperature());
    epd_poweroff();
    ESP_LOGI(TAG, "Initial clear complete.");
    ESP_LOGCONFIG(TAG, "ED047TC1 setup finished.");
}

void ED047TC1Display::update() {
    this->do_update_();
    if (!this->buffer_) { ESP_LOGE(TAG, "ESPHome buffer null in update!"); return; }
    uint8_t* epd_fb = epd_hl_get_framebuffer(&this->hl_state_);
    if (!epd_fb) { ESP_LOGE(TAG, "EPDiy FB null in update!"); return; }

    ESP_LOGD(TAG, "Copying ESPHome buffer to EPDiy buffer and updating display...");
    int w = this->get_width_internal();
    int h = this->get_height_internal();
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t esphome_pixel_8bpp = this->buffer_[(y * w) + x];
            uint8_t epdiy_draw_value = (esphome_pixel_8bpp >> 4) << 4;
            epd_draw_pixel(x, y, epdiy_draw_value, epd_fb);
        }
    }
    ESP_LOGD(TAG, "Buffer copy complete. Triggering EPD screen update.");
    epd_poweron();
    EpdRect update_rect = epd_full_screen();
    enum EpdDrawError draw_result = epd_hl_update_area(&this->hl_state_, MODE_GC16, epd_ambient_temperature(), update_rect);
    if (draw_result != EPD_DRAW_SUCCESS) { ESP_LOGE(TAG, "epd_hl_update_area failed: %d", draw_result); }
    epd_poweroff();
    ESP_LOGD(TAG, "ED047TC1 update cycle finished.");
}

void ED047TC1Display::draw_absolute_pixel_internal(int x, int y, Color color) {
    if (x < 0 || x >= get_width_internal() || y < 0 || y >= get_height_internal() || !this->buffer_) return;
    uint32_t r_val = color.r; uint32_t g_val = color.g; uint32_t b_val = color.b;
    uint8_t gray_value = static_cast<uint8_t>((r_val + g_val + b_val) / 3);
    uint8_t inverted_gray_value = 255 - gray_value;

    this->buffer_[(y * get_width_internal()) + x] = inverted_gray_value;
}

void ED047TC1Display::dump_config() {
    LOG_DISPLAY("", "ED047TC1 E-Paper Display", this);
    ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", get_width_internal(), get_height_internal());
    LOG_PIN("  PWR Pin: ", pwr_pin_); LOG_PIN("  BST_EN Pin: ", bst_en_pin_);
    LOG_PIN("  XSTL_PIN (STH/DE): ", xstl_pin_);
    if (pclk_pin_) { LOG_PIN("  PCLK_PIN (LCD Clock): ", pclk_pin_); }
    else { ESP_LOGCONFIG(TAG, "  PCLK_PIN: Not configured (XSTL_PIN attempted as PCLK - risky)"); }
    LOG_PIN("  XLE_PIN (LEH): ", xle_pin_); LOG_PIN("  SPV_PIN (STV): ", spv_pin_);
    LOG_PIN("  CKV_PIN (CKV): ", ckv_pin_);
    for (int i=0; i<8; ++i) {
        #if ESPHOME_VERSION_CODE >= VERSION_CODE(2023, 1, 0)
        if (d_pins_[i] != nullptr) {
            ESP_LOGCONFIG(TAG, "  D%d Pin: Configured (Pin: %d)", i, esphome_pin_to_gpio_num(d_pins_[i]));
        } else { ESP_LOGCONFIG(TAG, "  D%d Pin: NONE", i); }
        #else
        ESP_LOGCONFIG(TAG, "  D%d Pin: %s", i, d_pins_[i] ? "Configured" : "NONE");
        #endif
    }
    LOG_UPDATE_INTERVAL(this);
}

}  // namespace ed047tc1

}  // namespace esphome
