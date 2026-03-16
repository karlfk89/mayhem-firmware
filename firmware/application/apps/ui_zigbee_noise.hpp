/*
 * ZigBee Channel Noise Analyzer
 * For PortaPack Mayhem Firmware
 *
 * Scans all 16 ZigBee channels (11-26) in the 2.4 GHz band
 * and computes average noise floor per channel over a
 * configurable measurement period (default: 15 minutes).
 *
 * ZigBee Channel Frequencies:
 *   Ch 11: 2405 MHz  Ch 19: 2445 MHz
 *   Ch 12: 2410 MHz  Ch 20: 2450 MHz
 *   Ch 13: 2415 MHz  Ch 21: 2455 MHz
 *   Ch 14: 2420 MHz  Ch 22: 2460 MHz
 *   Ch 15: 2425 MHz  Ch 23: 2465 MHz
 *   Ch 16: 2430 MHz  Ch 24: 2470 MHz
 *   Ch 17: 2435 MHz  Ch 25: 2475 MHz
 *   Ch 18: 2440 MHz  Ch 26: 2480 MHz
 *
 * Copyright (C) 2024  PortaPack Mayhem Contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#pragma once

#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "string_format.hpp"
#include "receiver_model.hpp"
#include "message.hpp"
#include "baseband_api.hpp"
#include "portapack.hpp"

namespace ui {

// Number of ZigBee channels (11 through 26)
#define ZIGBEE_NUM_CHANNELS   16
#define ZIGBEE_FIRST_CHANNEL  11
#define ZIGBEE_LAST_CHANNEL   26

// First ZigBee channel center frequency in Hz
#define ZIGBEE_BASE_FREQ_HZ   2405000000ULL
// Channel spacing in Hz
#define ZIGBEE_CHAN_STEP_HZ   5000000ULL

// Sampling rate for 2.4 GHz measurements (5 MHz channel bandwidth)
#define ZIGBEE_SAMPLE_RATE    5000000
// RF bandwidth filter
#define ZIGBEE_BANDWIDTH      2500000

// Frames per channel dwell (at ~60 fps → ~333 ms per dwell per channel)
#define FRAMES_PER_DWELL      20

// Default measurement duration in minutes
#define DEFAULT_DURATION_MINS 15

// Screen layout constants
#define SCREEN_W              240
#define SCREEN_H              304
#define BAR_AREA_TOP          68
#define BAR_AREA_BOTTOM       246
#define BAR_AREA_H            (BAR_AREA_BOTTOM - BAR_AREA_TOP)
#define BAR_W                 14   // width of each channel bar
#define BAR_GAP               1    // gap between bars
#define CHAN_LABEL_Y          248  // y position of channel number labels

class ZigbeeNoiseView : public View {
public:
    ZigbeeNoiseView(NavigationView& nav);
    ~ZigbeeNoiseView();

    std::string title() const override { return "ZigBee Noise"; };
    void focus() override;

    void paint(Painter& painter) override;

private:
    // ---------- State machine ----------
    enum class ScanState {
        IDLE,
        SCANNING,
        COMPLETE
    };

    ScanState scan_state{ScanState::IDLE};

    // Current channel index (0-15 → channels 11-26)
    uint8_t  current_ch_idx{0};

    // Frame counter within the current dwell
    uint8_t  dwell_frame_counter{0};

    // Total elapsed frames across the full measurement
    uint32_t total_frames_elapsed{0};

    // Total frames for the full measurement window (calculated on start)
    uint32_t total_frames_target{0};

    // Per-channel RSSI accumulators
    uint32_t rssi_accum[ZIGBEE_NUM_CHANNELS]{0};
    uint32_t rssi_count[ZIGBEE_NUM_CHANNELS]{0};

    // Per-channel averaged RSSI (0–255 raw ADC scale)
    uint8_t  rssi_avg[ZIGBEE_NUM_CHANNELS]{0};

    // Peak (max ever recorded) RSSI per channel
    uint8_t  rssi_peak[ZIGBEE_NUM_CHANNELS]{0};

    // Latest instantaneous RSSI reading
    uint8_t  rssi_current{0};

    // Whether the bar graph needs a full redraw
    bool bars_dirty{true};

    // ---------- Helpers ----------
    void start_scan();
    void stop_scan();
    void reset_data();
    void advance_channel();
    void update_progress();
    void draw_bars(Painter& painter);
    void draw_channel_labels(Painter& painter);

    static uint64_t channel_to_freq(uint8_t ch_idx) {
        return ZIGBEE_BASE_FREQ_HZ + (uint64_t)ch_idx * ZIGBEE_CHAN_STEP_HZ;
    }

    // Convert raw RSSI (0-255) to approximate dBm-like display value
    // HackRF RSSI ADC: roughly maps 0→−100 dBm, 255→−20 dBm
    static int8_t rssi_to_dbm(uint8_t raw) {
        return (int8_t)(-100 + (int16_t)raw * 80 / 255);
    }

    // Map RSSI raw value (0-255) to bar height in pixels
    static uint16_t rssi_to_bar_h(uint8_t raw) {
        return (uint16_t)((uint32_t)raw * BAR_AREA_H / 255);
    }

    // ---------- Message handlers ----------
    void handle_frame_sync();
    void handle_rssi(const RSSIStatisticsMessage& msg);

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->handle_frame_sync();
        }
    };

    MessageHandlerRegistration message_handler_rssi{
        Message::ID::RSSIStatistics,
        [this](const Message* const p) {
            this->handle_rssi(*reinterpret_cast<const RSSIStatisticsMessage*>(p));
        }
    };

    // ---------- Widgets ----------

    // ── Row 1: scan duration setting ──────────────────────────────────────
    Labels label_duration{
        {
            {{4, 22}, "Duration:", Color::light_grey()},
            {{112, 22}, "min", Color::light_grey()},
        }
    };

    NumberField field_duration{
        {80, 22},           // position
        3,                  // digit width
        {1, 99},            // range 1–99 minutes
        1,                  // step
        ' ',                // fill char
        false               // no loop
    };

    // ── Row 2: scan progress info ──────────────────────────────────────────
    Labels label_status{
        {
            {{4, 40}, "Ch:", Color::light_grey()},
            {{60, 40}, "RSSI:", Color::light_grey()},
            {{136, 40}, "dBm", Color::light_grey()},
        }
    };

    // Current channel display (e.g. "24")
    Text text_current_ch{
        {28, 40, 24, 16},
        "--"
    };

    // Current instantaneous RSSI in dBm
    Text text_current_rssi{
        {96, 40, 32, 16},
        "---"
    };

    // ── Scan progress bar ─────────────────────────────────────────────────
    Labels label_progress{
        {
            {{4, 52}, "Progress:", Color::light_grey()},
        }
    };

    ProgressBar bar_progress{
        {76, 52, 120, 12}
    };

    Text text_progress_pct{
        {200, 52, 36, 12},
        "  0%"
    };

    // ── Bar chart title labels ─────────────────────────────────────────────
    // (painted manually in paint() for flexibility)

    // ── Bottom controls ────────────────────────────────────────────────────
    Button btn_start_stop{
        {4, 264, 100, 28},
        "START"
    };

    Button btn_reset{
        {116, 264, 60, 28},
        "RESET"
    };

    Button btn_exit{
        {188, 264, 48, 28},
        "EXIT"
    };

    // Status text at very bottom
    Text text_status{
        {4, 294, 232, 10},
        ""
    };
};

}  // namespace ui
