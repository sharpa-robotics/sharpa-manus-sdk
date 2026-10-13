// C++ counterpart of sample/python/sharpa_tactile_callback.py — tactile stream via
// set_tactile_callback(); main thread composes the same OpenCV grid as the fetch example.
//
// Build: see sample/c++/Makefile (requires OpenCV development packages).

#include "tactile_sample_common.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "SharpaWaveSDK.h"

namespace {

struct ChannelFrame {
  cv::Mat raw;
  cv::Mat deform_bgr;
  bool has_raw{false};
  bool has_deform{false};
};

std::mutex g_mu;
std::array<ChannelFrame, 10> g_by_channel;
std::vector<double> g_latency;

void on_tactile_frame(sharpa::tactile::Frame::Ptr fr) {
  if (!fr) return;
  const int ch = fr->channel;
  if (ch < 0 || ch >= 10) return;

  cv::Mat raw_gray;
  bool got_raw = false;
  auto it_raw = fr->content.find("RAW");
  if (it_raw != fr->content.end() && it_raw->second) {
    got_raw = tactile_sample::raw_from_block(it_raw->second, raw_gray);
  }

  auto it_d = fr->content.find("DEFORM");
  auto it_f6 = fr->content.find("F6");
  auto it_cp = fr->content.find("CONTACT_POINT");
  sharpa::tactile::DataBlock::Ptr dblk, f6blk, cpblk;
  if (it_d != fr->content.end()) dblk = it_d->second;
  if (it_f6 != fr->content.end()) f6blk = it_f6->second;
  if (it_cp != fr->content.end()) cpblk = it_cp->second;

  cv::Mat deform_bgr;
  bool got_deform = false;
  if (dblk && f6blk) {
    got_deform = tactile_sample::deform_bgr_from_blocks(dblk, f6blk, cpblk, deform_bgr);
  }

  const auto now = std::chrono::system_clock::now();
  const double ts_wall =
      std::chrono::duration<double>(now.time_since_epoch()).count();

  std::lock_guard<std::mutex> lk(g_mu);
  auto& dst = g_by_channel[static_cast<size_t>(ch)];
  if (got_raw) {
    dst.raw = std::move(raw_gray);
    dst.has_raw = true;
  }
  if (got_deform) {
    dst.deform_bgr = std::move(deform_bgr);
    dst.has_deform = true;
  }
  g_latency.push_back(ts_wall - fr->ts);
  if (g_latency.size() > 500) g_latency.erase(g_latency.begin(), g_latency.begin() + 250);
}

}  // namespace

int main() {
  using namespace tactile_sample;
  std::cout << "=== Sharpa Tactile Frame Data Acquisition (C++ callback example) ===\n";

  const char* kWin = "Sharpa Tactile Data - LEFT (Left) | RIGHT (Right)";
  cv::namedWindow(kWin, cv::WINDOW_NORMAL);
  cv::resizeWindow(kWin, 1280, 1200);

  HandBuffers buffers[2];
  cv::imshow(kWin, compose_canvas(buffers[0], buffers[1]));
  cv::waitKey(100);

  try {
    auto& manager = sharpa::SharpaWaveManager::get_instance();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::vector<std::reference_wrapper<sharpa::SharpaWave>> waves;
    std::vector<std::string> hand_names;

    while (true) {
      auto devices = manager.get_all_devices();
      const bool any_hand =
          !devices.empty() &&
          std::any_of(devices.begin(), devices.end(),
                      [](const sharpa::DeviceInfo& d) {
                        return d.device_type == sharpa::DeviceType::HAND;
                      });
      if (any_hand) break;
      std::cout << "Waiting for device connection...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto devices = manager.get_all_devices();
    for (const auto& di : devices) {
      if (di.device_type != sharpa::DeviceType::HAND) continue;
      auto& wave = manager.connect(di.sn);
      if (!wave.get_device_info().has_fingertip_tactile()) {
        std::cerr << "Device " << di.sn
                  << " firmware does not support tactile feature, exiting.\n";
        manager.disconnect_all();
        return 1;
      }
      waves.push_back(std::ref(wave));
      hand_names.push_back(wave.get_device_info().hand_side == sharpa::HandSide::LEFT
                               ? "LEFT"
                               : "RIGHT");
      std::cout << "Connected " << di.sn << " hand_side=" << hand_names.back() << "\n";
    }

    if (waves.empty()) {
      std::cerr << "No HAND devices found.\n";
      return 1;
    }

    for (size_t i = 0; i < waves.size(); ++i) {
      waves[i].get().set_tactile_callback(on_tactile_frame);
      std::cout << "Callback set for device " << i << "\n";
    }

    for (size_t i = 0; i < waves.size(); ++i) {
      if (!waves[i].get().start()) {
        std::cerr << "Wave " << i << " start failed; check tactile UDP port "
                  << "(see device / logs for tactile port)\n";
      } else {
        std::cout << "Wave " << i << " started\n";
      }
    }

    {
      std::lock_guard<std::mutex> lk(g_mu);
      g_by_channel = {};
      g_latency.clear();
    }

    const auto start_time = std::chrono::steady_clock::now();
    std::cout << "\nTactile callback stream. Keys: t=calib, j=JPEG on, s=JPEG off, ESC=exit\n";

    int key = 0;
    int ui_frame = 0;
    while (key != 27) {
      key = cv::waitKey(1) & 0xFF;

      if (key == static_cast<int>('t')) {
        std::cout << "Tactile calibration...\n";
        for (size_t i = 0; i < waves.size(); ++i) {
          if (!waves[i].get().calib_tactile())
            std::cout << "Hand " << hand_names[i] << " calib failed\n";
          else
            std::cout << "Hand " << hand_names[i] << " calib ok\n";
        }
      }
      if (key == static_cast<int>('j')) {
        for (size_t i = 0; i < waves.size(); ++i) {
          auto err = waves[i].get().set_parameter(
              R"({"secret_function":"set_tactile_jpeg_enable","enable":true,"channel":-1})");
          std::cout << "Hand " << hand_names[i] << " JPEG enable: code=" << err.code
                    << " msg=" << err.message << "\n";
        }
      }
      if (key == static_cast<int>('s')) {
        for (size_t i = 0; i < waves.size(); ++i) {
          auto err = waves[i].get().set_parameter(
              R"({"secret_function":"set_tactile_jpeg_enable","enable":false,"channel":-1})");
          std::cout << "Hand " << hand_names[i] << " JPEG disable: code=" << err.code
                    << " msg=" << err.message << "\n";
          if (err.code == 0) {
            const sharpa::HandSide side = waves[i].get().get_device_info().hand_side;
            int ch0 = 0, ch1 = 0, disp_off = 0;
            channel_range_for_hand(side, ch0, ch1, disp_off);
            std::lock_guard<std::mutex> lk(g_mu);
            for (int ch = ch0; ch < ch1; ++ch) {
              auto& cf = g_by_channel[static_cast<size_t>(ch)];
              cf.raw = cv::Mat();
              cf.has_raw = false;
            }
            const int hidx = display_hand_index(side);
            for (int c = 0; c < 5; ++c) buffers[hidx].raw[c] = cv::Mat::zeros(240, 320, CV_8UC1);
          }
        }
      }

      std::array<ChannelFrame, 10> snap;
      std::vector<double> lat_copy;
      {
        std::lock_guard<std::mutex> lk(g_mu);
        snap = g_by_channel;
        lat_copy = g_latency;
      }
      ++ui_frame;

      bool image_updated = false;
      for (int ch = 0; ch < 10; ++ch) {
        const auto& cf = snap[static_cast<size_t>(ch)];
        if (!cf.has_raw && !cf.has_deform) continue;

        const int hand_idx = (ch >= 5) ? 0 : 1;
        const int display_ch = (ch >= 5) ? (ch - 5) : ch;

        if (cf.has_raw) {
          cf.raw.copyTo(buffers[hand_idx].raw[display_ch]);
          image_updated = true;
        }
        if (cf.has_deform) {
          cf.deform_bgr.copyTo(buffers[hand_idx].deform[display_ch]);
          image_updated = true;
        }
      }

      if (image_updated) cv::imshow(kWin, compose_canvas(buffers[0], buffers[1]));

      if (ui_frame % 100 == 0 && !lat_copy.empty()) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - start_time).count();
        double s = 0;
        for (double x : lat_copy) s += x;
        std::cout << "--- stats ui_frames=" << ui_frame << " avg_latency=" << std::setprecision(3)
                  << (s / lat_copy.size()) << "s runtime=" << std::setprecision(1) << elapsed
                  << "s ---\n";
      }
    }

    for (size_t i = 0; i < waves.size(); ++i) {
      std::string summary = waves[i].get().tactile_summary();
      std::cout << "\n=== tactile_summary device " << i << " ===\n" << summary << "\n";
    }

    for (size_t i = 0; i < waves.size(); ++i) {
      waves[i].get().stop();
      waves[i].get().destroy();
      std::cout << "Wave " << i << " stopped\n";
    }
    manager.disconnect_all();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    cv::destroyAllWindows();
    return 1;
  }

  cv::destroyAllWindows();
  return 0;
}
