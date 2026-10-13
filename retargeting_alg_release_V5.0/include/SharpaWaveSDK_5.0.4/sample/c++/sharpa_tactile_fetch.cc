// C++ counterpart of sample/python/sharpa_tactile_fetch.py — poll tactile frames via
// fetch_tactile_frame() and show RAW/DEFORM in one OpenCV window.
//
// Build: see sample/c++/Makefile (requires OpenCV development packages).

#include "tactile_sample_common.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include "SharpaWaveSDK.h"

int main() {
  using namespace tactile_sample;
  std::cout << "=== Sharpa Tactile Frame Data Acquisition (C++ fetch example) ===\n";

  const char* kWin = "Sharpa Tactile Data - LEFT (Left) | RIGHT (Right)";
  cv::namedWindow(kWin, cv::WINDOW_NORMAL);
  cv::resizeWindow(kWin, 1280, 1200);

  HandBuffers buffers[2];
  cv::imshow(kWin, compose_canvas(buffers[0], buffers[1]));
  cv::waitKey(100);

  try {
    auto& manager = sharpa::SharpaWaveManager::get_instance();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (manager.get_all_device_sn().empty()) {
      std::cout << "Waiting for device connection...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::vector<std::reference_wrapper<sharpa::SharpaWave>> waves;
    std::vector<std::string> hand_names;
    std::vector<sharpa::HandSide> hand_sides;

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
      hand_sides.push_back(wave.get_device_info().hand_side);
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
      if (!waves[i].get().start()) {
        std::cerr << "Wave " << i << " start failed; check tactile UDP port "
                  << "(see device / logs for tactile port)\n";
      } else {
        std::cout << "Wave " << i << " started\n";
      }
    }

    int frame_count = 0;
    std::vector<double> latencies;
    std::cout << "\nAcquire tactile data (fetch). Keys: t=calib, j=JPEG on, s=JPEG off, ESC=exit\n";

    int key = 0;
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
            const int hidx = display_hand_index(side);
            for (int c = 0; c < 5; ++c) buffers[hidx].raw[c] = cv::Mat::zeros(240, 320, CV_8UC1);
            cv::imshow(kWin, compose_canvas(buffers[0], buffers[1]));
          }
        }
      }

      ++frame_count;
      bool image_updated = false;

      for (size_t wi = 0; wi < waves.size(); ++wi) {
        auto& wave = waves[wi].get();
        const sharpa::HandSide side = wave.get_device_info().hand_side;
        const int hidx = display_hand_index(side);
        const std::string& hname = hand_names[wi];

        int ch0 = 0, ch1 = 0, disp_off = 0;
        channel_range_for_hand(side, ch0, ch1, disp_off);

        for (int ch = ch0; ch < ch1; ++ch) {
          auto fr = wave.fetch_tactile_frame(ch, 0.0);
          if (!fr) continue;

          const auto now = std::chrono::system_clock::now();
          const double ts_wall =
              std::chrono::duration<double>(now.time_since_epoch()).count();
          latencies.push_back(ts_wall - fr->ts);

          const int display_ch = ch + disp_off;
          if (display_ch < 0 || display_ch >= 5) continue;

          auto it_raw = fr->content.find("RAW");
          if (it_raw != fr->content.end() && it_raw->second) {
            cv::Mat raw_gray;
            if (raw_from_block(it_raw->second, raw_gray)) {
              raw_gray.copyTo(buffers[hidx].raw[display_ch]);
              image_updated = true;
            }
          }

          auto it_d = fr->content.find("DEFORM");
          auto it_f6 = fr->content.find("F6");
          auto it_cp = fr->content.find("CONTACT_POINT");
          sharpa::tactile::DataBlock::Ptr dblk, f6blk, cpblk;
          if (it_d != fr->content.end()) dblk = it_d->second;
          if (it_f6 != fr->content.end()) f6blk = it_f6->second;
          if (it_cp != fr->content.end()) cpblk = it_cp->second;
          if (dblk && f6blk) {
            cv::Mat bgr;
            if (deform_bgr_from_blocks(dblk, f6blk, cpblk, bgr)) {
              bgr.copyTo(buffers[hidx].deform[display_ch]);
              image_updated = true;
            }
          }

          if (frame_count % 50 == 0) {
            std::cout << "Hand " << hname << " ch " << ch << " ts=" << std::fixed
                      << std::setprecision(6) << fr->ts << " latency=" << std::setprecision(3)
                      << (ts_wall - fr->ts) << "s\n";
            print_content_shapes(fr);
          }
        }
      }

      if (latencies.size() > 100) latencies.erase(latencies.begin(), latencies.begin() + 50);

      if (frame_count % 200 == 0 && !latencies.empty()) {
        double s = 0;
        for (double x : latencies) s += x;
        std::cout << "--- stats frames=" << frame_count << " avg_latency=" << std::setprecision(3)
                  << (s / latencies.size()) << "s ---\n";
      }

      if (image_updated) cv::imshow(kWin, compose_canvas(buffers[0], buffers[1]));
    }

    for (size_t i = 0; i < waves.size(); ++i) {
      waves[i].get().stop();
      waves[i].get().destroy();
      std::cout << "Wave " << i << " stopped\n";
    }
    manager.disconnect_all();

    std::cout << "\n=== Statistics ===\nTotal loop iterations: " << frame_count << "\n";
    if (!latencies.empty()) {
      double s = 0, mn = latencies[0], mx = latencies[0];
      for (double x : latencies) {
        s += x;
        mn = std::min(mn, x);
        mx = std::max(mx, x);
      }
      s /= latencies.size();
      std::cout << "Latency avg=" << s << " min=" << mn << " max=" << mx << "\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    cv::destroyAllWindows();
    return 1;
  }

  cv::destroyAllWindows();
  return 0;
}
