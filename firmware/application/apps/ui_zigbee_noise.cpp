/*
 * ZigBee Channel Noise Analyzer
 * For PortaPack Mayhem Firmware
 *
 * Copyright (C) 2024  PortaPack Mayhem Contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#include "ui_zigbee_noise.hpp"
#include "portapack.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "portapack_shared_memory.hpp"
#include <cstring>
#include <algorithm>

using namespace portapack;

namespace ui {

/* =========================================================================
   Constructor / Destructor
   ========================================================================= */

ZigbeeNoiseView::ZigbeeNoiseView(NavigationView& nav) {

    // ── Add all child widgets ─────────────────────────────────────────────
    add_children({
        &label_duration,
        &field_duration,
        &label_status,
        &text_current_ch,
        &text_current_rssi,
        &label_progress,
        &bar_progress,
        &text_progress_pct,
        &btn_start_stop,
        &btn_reset,
        &btn_exit,
        &text_status,
    });

    // ── Initialise progress bar ────────────────────────────────────────────
    bar_progress.set_max(100);
    bar_progress.set_value(0);

    // ── Default duration ───────────────────────────────────────────────────
    field_duration.set_value(DEFAULT_DURATION_MINS);

    // ── Wire up buttons ────────────────────────────────────────────────────
    btn_start_stop.on_select = [this](Button&) {
        if (scan_state == ScanState::SCANNING) {
            stop_scan();
        } else {
            start_scan();
        }
    };

    btn_reset.on_select = [this](Button&) {
        stop_scan();
        reset_data();
        bars_dirty = true;
        set_dirty();
        text_status.set("Data cleared.");
    };

    btn_exit.on_select = [&nav](Button&) {
        nav.pop();
    };

    // ── Initial status ─────────────────────────────────────────────────────
    reset_data();
    text_status.set("Press START to begin scan.");
}

ZigbeeNoiseView::~ZigbeeNoiseView() {
    // Always disable the receiver on exit
    receiver_model.disable();
    baseband::shutdown();
}

/* =========================================================================
   Focus
   ========================================================================= */

void ZigbeeNoiseView::focus() {
    btn_start_stop.focus();
}

/* =========================================================================
   Scan control
   ========================================================================= */

void ZigbeeNoiseView::reset_data() {
    for (int i = 0; i < ZIGBEE_NUM_CHANNELS; i++) {
        rssi_accum[i] = 0;
        rssi_count[i] = 0;
        rssi_avg[i]   = 0;
        rssi_peak[i]  = 0;
    }
    total_frames_elapsed = 0;
    dwell_frame_counter  = 0;
    current_ch_idx       = 0;
    rssi_current         = 0;
    bar_progress.set_value(0);
    text_progress_pct.set("  0%");
    text_current_ch.set("--");
    text_current_rssi.set("---");
}

void ZigbeeNoiseView::start_scan() {
    if (scan_state == ScanState::SCANNING) return;

    // Calculate total frame budget (~60 fps assumption)
    const uint32_t duration_secs = (uint32_t)field_duration.value() * 60;
    total_frames_target  = duration_secs * 60;
    total_frames_elapsed = 0;
    current_ch_idx       = 0;
    dwell_frame_counter  = 0;

    scan_state = ScanState::SCANNING;
    btn_start_stop.set_text("STOP");
    field_duration.set_focusable(false);

    // ── Start the NFM audio baseband image (keeps receiver active at 2.4 GHz) ──
    baseband::run_image(portapack::spi_flash::image_tag_nfm_audio);

    // ── Configure receiver for first channel ──────────────────────────────
    receiver_model.set_target_frequency(channel_to_freq(current_ch_idx));
    receiver_model.set_sampling_rate(ZIGBEE_SAMPLE_RATE);
    receiver_model.set_baseband_bandwidth(ZIGBEE_BANDWIDTH);
    receiver_model.set_lna(24);
    receiver_model.set_vga(24);
    receiver_model.set_rf_amp(false);  // No amp needed at 2.4 GHz typically
    receiver_model.enable();

    bars_dirty = true;
    text_status.set("Scanning...");
    text_current_ch.set(to_string_dec_int(ZIGBEE_FIRST_CHANNEL + (int)current_ch_idx));
}

void ZigbeeNoiseView::stop_scan() {
    if (scan_state != ScanState::SCANNING) return;

    scan_state = ScanState::IDLE;
    receiver_model.disable();
    baseband::shutdown();

    btn_start_stop.set_text("START");
    field_duration.set_focusable(true);
    text_current_ch.set("--");
    text_current_rssi.set("---");
    text_status.set("Scan stopped. Press START to restart.");
    bars_dirty = true;
}

/* =========================================================================
   Channel advance
   ========================================================================= */

void ZigbeeNoiseView::advance_channel() {
    current_ch_idx = (current_ch_idx + 1) % ZIGBEE_NUM_CHANNELS;
    receiver_model.set_target_frequency(channel_to_freq(current_ch_idx));
    dwell_frame_counter = 0;

    // Update channel display
    text_current_ch.set(to_string_dec_int(ZIGBEE_FIRST_CHANNEL + (int)current_ch_idx));
}

/* =========================================================================
   Progress update
   ========================================================================= */

void ZigbeeNoiseView::update_progress() {
    if (total_frames_target == 0) return;

    const uint32_t pct = (uint32_t)((uint64_t)total_frames_elapsed * 100
                                    / total_frames_target);
    bar_progress.set_value((pct > 100) ? 100 : (int)pct);

    // Format percentage string
    const std::string pct_str = to_string_dec_uint(pct > 100 ? 100 : pct) + "%";
    // Right-align in 4 chars
    std::string padded = "    " + pct_str;
    text_progress_pct.set(padded.substr(padded.size() - 4));
}

/* =========================================================================
   Message handlers
   ========================================================================= */

void ZigbeeNoiseView::handle_rssi(const RSSIStatisticsMessage& msg) {
    if (scan_state != ScanState::SCANNING) return;

    // Compute the mean of this statistics block
    if (msg.statistics.count > 0) {
        const uint8_t mean = (uint8_t)(msg.statistics.accumulator / msg.statistics.count);
        rssi_current = mean;

        // Accumulate into current channel bucket
        rssi_accum[current_ch_idx] += mean;
        rssi_count[current_ch_idx]++;

        // Update peak
        if (mean > rssi_peak[current_ch_idx])
            rssi_peak[current_ch_idx] = mean;

        // Update running average
        rssi_avg[current_ch_idx] = (uint8_t)(
            rssi_accum[current_ch_idx] / rssi_count[current_ch_idx]);

        // Update RSSI display
        const int8_t dbm = rssi_to_dbm(mean);
        const std::string rssi_str = to_string_dec_int((int)dbm);
        const std::string padded   = "    " + rssi_str;
        text_current_rssi.set(padded.substr(padded.size() > 4 ? padded.size() - 4 : 0));

        bars_dirty = true;
    }
}

void ZigbeeNoiseView::handle_frame_sync() {
    if (scan_state != ScanState::SCANNING) return;

    total_frames_elapsed++;

    // Check if overall scan is complete
    if (total_frames_elapsed >= total_frames_target) {
        // Final averages already computed; stop scan
        scan_state = ScanState::COMPLETE;
        receiver_model.disable();
        baseband::shutdown();

        btn_start_stop.set_text("START");
        field_duration.set_focusable(true);
        bar_progress.set_value(100);
        text_progress_pct.set("100%");
        text_current_ch.set("--");
        text_current_rssi.set("---");
        text_status.set("Scan complete!");
        bars_dirty = true;
        set_dirty();
        return;
    }

    // Advance channel after dwell expires
    dwell_frame_counter++;
    if (dwell_frame_counter >= FRAMES_PER_DWELL) {
        advance_channel();
        bars_dirty = true;
    }

    // Throttle expensive full redraws – only every ~6 frames (10 Hz)
    if ((total_frames_elapsed % 6) == 0) {
        update_progress();
        if (bars_dirty) {
            set_dirty();
            bars_dirty = false;
        }
    }
}

/* =========================================================================
   Paint (bar chart + dBm axis)
   ========================================================================= */

void ZigbeeNoiseView::paint(Painter& painter) {
    // ── Draw the bar-chart background ──────────────────────────────────────
    const Rect bar_area{0, BAR_AREA_TOP - 16, SCREEN_W, BAR_AREA_H + 16};
    painter.fill_rectangle(bar_area, Theme::getInstance()->bg_darkest->background);

    // ── dBm axis labels on the left (every 20 dBm) ─────────────────────────
    // Map −100 dBm → bar bottom, −20 dBm → bar top
    const Color axis_color = Color::grey();
    const Style& axis_style = *Theme::getInstance()->bg_darkest;

    for (int dbm = -100; dbm <= -20; dbm += 20) {
        // Calculate y position for this dBm level
        // dbm range: −100 to −20 (80 dBm span)
        const int rel      = dbm - (-100);           // 0..80
        const int bar_px   = (int)((uint32_t)rel * BAR_AREA_H / 80);
        const int y        = BAR_AREA_BOTTOM - bar_px;

        // Tick line across full width (subtle)
        painter.draw_hline({0, y}, SCREEN_W, axis_color);

        // Label
        const std::string lbl = to_string_dec_int(dbm);
        painter.draw_string({0, y - 8}, axis_style, lbl);
    }

    // ── Draw bars ──────────────────────────────────────────────────────────
    draw_bars(painter);

    // ── Channel number labels below bars ───────────────────────────────────
    draw_channel_labels(painter);

    // ── Legend header ─────────────────────────────────────────────────────
    painter.draw_string(
        {4, BAR_AREA_TOP - 14},
        *Theme::getInstance()->bg_darkest,
        "Avg");
    const Style style_yellow{
        .font       = Theme::getInstance()->bg_darkest->font,
        .background = Theme::getInstance()->bg_darkest->background,
        .foreground = Color::yellow()
    };
    painter.draw_string(
        {28, BAR_AREA_TOP - 14},
        style_yellow,
        "Peak");

    // ── Scanning indicator (highlight current channel bar outline) ─────────
    if (scan_state == ScanState::SCANNING) {
        const int x = (int)current_ch_idx * (BAR_W + BAR_GAP);
        painter.draw_rectangle(
            {x, BAR_AREA_TOP - 2, BAR_W, BAR_AREA_H + 4},
            Color::white());
    }
}

void ZigbeeNoiseView::draw_bars(Painter& painter) {
    for (int i = 0; i < ZIGBEE_NUM_CHANNELS; i++) {
        const int x = i * (BAR_W + BAR_GAP);

        // ── Clear this bar's column ──────────────────────────────────────
        painter.fill_rectangle(
            {x, BAR_AREA_TOP, BAR_W, BAR_AREA_H},
            Theme::getInstance()->bg_darkest->background);

        // ── Average bar (blue / cyan) ────────────────────────────────────
        if (rssi_count[i] > 0) {
            const uint16_t avg_h = rssi_to_bar_h(rssi_avg[i]);
            if (avg_h > 0) {
                // Choose colour based on noise level
                Color bar_color;
                const int8_t dbm = rssi_to_dbm(rssi_avg[i]);
                if (dbm >= -60)       bar_color = Color::red();
                else if (dbm >= -75)  bar_color = Color::orange();
                else if (dbm >= -85)  bar_color = Color::yellow();
                else                  bar_color = Color::green();

                painter.fill_rectangle(
                    {x, BAR_AREA_BOTTOM - (int)avg_h, BAR_W, (int)avg_h},
                    bar_color);
            }
        }

        // ── Peak tick mark (yellow, 2 px tall) ──────────────────────────
        if (rssi_peak[i] > 0) {
            const uint16_t peak_h = rssi_to_bar_h(rssi_peak[i]);
            if (peak_h > 0) {
                const int peak_y = BAR_AREA_BOTTOM - (int)peak_h;
                painter.draw_hline({x, peak_y}, BAR_W, Color::yellow());
                if (peak_y + 1 < BAR_AREA_BOTTOM)
                    painter.draw_hline({x, peak_y + 1}, BAR_W, Color::yellow());
            }
        }
    }
}

void ZigbeeNoiseView::draw_channel_labels(Painter& painter) {
    // Draw channel numbers 11-26 below the bar area
    // Two rows to fit: odd channels on first row, even on second row
    for (int i = 0; i < ZIGBEE_NUM_CHANNELS; i++) {
        const int ch  = ZIGBEE_FIRST_CHANNEL + i;
        const int x   = i * (BAR_W + BAR_GAP);
        const int row = (i % 2 == 0) ? 0 : 1;
        const int y   = CHAN_LABEL_Y + row * 9;

        // Highlight the active channel in white, all others in light grey
        const Color lbl_color =
            (scan_state == ScanState::SCANNING && (int)current_ch_idx == i)
            ? Color::white()
            : Color::light_grey();

        const Style lbl_style{
            .font       = Theme::getInstance()->bg_darkest->font,
            .background = Theme::getInstance()->bg_darkest->background,
            .foreground = lbl_color
        };

        const std::string ch_str = to_string_dec_int(ch);
        painter.draw_string({x, y}, lbl_style, ch_str);
    }
}

}  // namespace ui
