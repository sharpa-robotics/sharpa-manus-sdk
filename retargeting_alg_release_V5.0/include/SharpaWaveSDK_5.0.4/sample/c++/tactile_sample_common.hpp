#pragma once

#include "SharpaWaveSDK.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

namespace tactile_sample {

inline int display_hand_index(sharpa::HandSide side) {
  return (side == sharpa::HandSide::LEFT) ? 0 : 1;
}

struct HandBuffers {
  cv::Mat raw[5];
  cv::Mat deform[5];
  HandBuffers() {
    for (int i = 0; i < 5; ++i) {
      raw[i] = cv::Mat::zeros(240, 320, CV_8UC1);
      deform[i] = cv::Mat::zeros(240, 240, CV_8UC3);
    }
  }
};

inline void blit_gray_to_bgr_roi(const cv::Mat& gray, cv::Mat& canvas, int x, int y,
                                 int target_w, int target_h) {
  cv::Mat tmp;
  if (gray.cols == target_w && gray.rows == target_h) {
    tmp = gray;
  } else {
    cv::resize(gray, tmp, cv::Size(target_w, target_h));
  }
  cv::Mat bgr;
  cv::cvtColor(tmp, bgr, cv::COLOR_GRAY2BGR);
  bgr.copyTo(canvas(cv::Rect(x, y, target_w, target_h)));
}

inline cv::Mat compose_canvas(const HandBuffers& left, const HandBuffers& right) {
  cv::Mat canvas(1200, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
  auto paint = [&](int hand_idx, const HandBuffers& buf, const char* name) {
    const int ox = hand_idx * 640;
    for (int ch = 0; ch < 5; ++ch) {
      const int raw_y = ch * 240;
      const int raw_x = ox + 0;
      blit_gray_to_bgr_roi(buf.raw[ch], canvas, raw_x, raw_y, 320, 240);
      const int orig_ch = (hand_idx == 0) ? (ch + 5) : ch;
      cv::putText(canvas, cv::format("%sCh%d-RAW", name, orig_ch),
                  cv::Point(raw_x + 5, raw_y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 2);
      const int dy = ch * 240;
      const int dx = ox + 320;
      buf.deform[ch].copyTo(canvas(cv::Rect(dx, dy, 240, 240)));
      cv::putText(canvas, cv::format("%sCh%d-DEFORM", name, orig_ch),
                  cv::Point(dx + 5, dy + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 2);
    }
  };
  paint(0, left, "LEFT");
  paint(1, right, "RIGHT");
  return canvas;
}

inline bool raw_from_block(const sharpa::tactile::DataBlock::Ptr& db, cv::Mat& out_gray) {
  if (!db || db->data() == nullptr || db->size() == 0) return false;
  const auto sh = db->shape();
  const size_t n = db->size();
  const uint8_t* p = static_cast<const uint8_t*>(db->data());

  if (n == 76800) {
    out_gray = cv::Mat(240, 320, CV_8UC1, const_cast<uint8_t*>(p)).clone();
    return true;
  }
  if (n == 57600) {
    cv::Mat g(240, 240, CV_8UC1, const_cast<uint8_t*>(p));
    cv::resize(g.clone(), out_gray, cv::Size(320, 240));
    return true;
  }
  if (sh.dim() >= 2) {
    const int h = static_cast<int>(sh[sh.dim() - 2]);
    const int w = static_cast<int>(sh[sh.dim() - 1]);
    if (h > 0 && w > 0 && static_cast<size_t>(h * w) == n) {
      cv::Mat g(h, w, CV_8UC1, const_cast<uint8_t*>(p));
      cv::resize(g.clone(), out_gray, cv::Size(320, 240));
      return true;
    }
  }
  return false;
}

inline bool deform_bgr_from_blocks(const sharpa::tactile::DataBlock::Ptr& deform_db,
                                   const sharpa::tactile::DataBlock::Ptr& f6_db,
                                   const sharpa::tactile::DataBlock::Ptr& contact_db,
                                   cv::Mat& out_bgr) {
  if (!deform_db || !f6_db || deform_db->data() == nullptr || f6_db->data() == nullptr)
    return false;
  const size_t dz = deform_db->size();
  if (dz != 57600) return false;

  cv::Mat gray(240, 240, CV_8UC1, deform_db->data());
  cv::Mat bgr;
  cv::cvtColor(gray.clone(), bgr, cv::COLOR_GRAY2BGR);

  const float* f6 = static_cast<const float*>(f6_db->data());
  const size_t nf = f6_db->size();
  int y = 40;
  for (size_t i = 0; i < std::min<size_t>(6, nf); ++i) {
    cv::putText(bgr, cv::format("F%zu: %.3f", i, static_cast<double>(f6[i])),
                cv::Point(5, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    y += 20;
  }

  if (contact_db && contact_db->data() && contact_db->size() >= 3) {
    const float* cp = static_cast<const float*>(contact_db->data());
    const size_t n = contact_db->size();
    for (size_t i = 0; i + 2 < n; i += 3) {
      const int x = static_cast<int>(cp[i]);
      const int yy = static_cast<int>(cp[i + 1]);
      cv::circle(bgr, cv::Point(x, yy), 2, cv::Scalar(0, 0, 255), -1);
      cv::putText(bgr, "CONTACT", cv::Point(x + 10, yy + 10), cv::FONT_HERSHEY_SIMPLEX,
                  0.4, cv::Scalar(0, 0, 255), 1);
    }
  }
  out_bgr = bgr;
  return true;
}

inline void channel_range_for_hand(sharpa::HandSide side, int& start_ch, int& end_ch,
                                   int& display_offset) {
  if (side == sharpa::HandSide::LEFT) {
    start_ch = 5;
    end_ch = 10;
    display_offset = -5;
  } else {
    start_ch = 0;
    end_ch = 5;
    display_offset = 0;
  }
}

inline void print_content_shapes(const sharpa::tactile::Frame::Ptr& fr) {
  if (!fr) return;
  for (const auto& kv : fr->content) {
    const auto& name = kv.first;
    const auto& blk = kv.second;
    if (!blk) {
      std::cout << "  " << name << ": NULL\n";
      continue;
    }
    const auto sh = blk->shape();
    std::cout << "  " << name << ": shape[";
    for (size_t i = 0; i < sh.dim(); ++i) {
      if (i) std::cout << ", ";
      std::cout << sh[i];
    }
    std::cout << "], size=" << blk->size() << "\n";
  }
}

}  // namespace tactile_sample
