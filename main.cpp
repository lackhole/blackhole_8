#include <iostream>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "blackhole/camera.h"
#include "blackhole/cv_key.h"

#include "opencv2/opencv.hpp"

#if defined(BH_USE_OPENCL)
  #if defined(__APPLE__) || defined(__MACOSX)
    #include "OpenCL/cl.h"
  #else
    #define CL_USE_DEPRECATED_OPENCL_1_2_APIS
    #include <CL/cl.h>
  #endif
#endif


int main() {
  constexpr const int kScreenWidth = 1600;
  constexpr const int kScreenHeight = 900;

  blackhole::Camera camera(kScreenWidth, kScreenHeight, blackhole::pi / 2);
  camera.Move(0, 0, 10);

  cv::Mat screen1(kScreenHeight, kScreenWidth, CV_8UC3);
  cv::Mat screen2(kScreenHeight, kScreenWidth, CV_8UC3);
  cv::Mat* screen = &screen1;
  std::mutex m;
  std::condition_variable cv;
  bool die = false;
  bool clean = false;

  std::thread clean_thread([&]() {
    std::unique_lock lck(m, std::defer_lock);
    while (true) {
      lck.lock();
      cv.wait(lck, [&]() {
        return die or clean;
      });

      if (die) break;

      if (screen == &screen1) {
        screen = &screen2;
      } else { // screen == &screen2
        screen = &screen1;
      }
      std::memset(screen->data, 0, kScreenWidth * kScreenHeight * 3);
      clean = false;
      lck.unlock();
      cv.notify_all();
    }
  });


  for (;;) {
    {
      std::unique_lock lck(m);
      cv.wait(lck, [&](){ return !clean; });
    }

    for (int x = 0; x < kScreenWidth; ++x) {
      for (int y = 0; y < kScreenHeight; ++y) {
        const auto pv = camera.PixelVector(x, y);
        const auto pv2 = cv::normalize(pv) * 100000; // rough end of a ray

        if (camera.position()[2] == 0)
          continue;

        bool do_collide_with_xy = (camera.focus()[2] * pv2[2]) < 0;
        if (do_collide_with_xy) {
          const auto c1 = std::abs((camera.focus())[2]);
          const auto c2 = std::abs(pv2[2]);
          const auto pq = pv2 - (camera.focus());
          const auto cm = (camera.focus()) + (c1 / (c1 + c2)) * pq;

          const auto x2 = cm[0];
          const auto y2 = cm[1];

          auto mod_20 = [] (auto x) {
            auto x2 = std::abs(x);
            return x2 - ((int)(x2) / 20) * 20;
          };

          if (x2 * y2 > 0) { // same
            const auto x3 = mod_20(x2); // std::abs(x2) % 20;
            const auto y3 = mod_20(y2); // std::abs(y2) % 20;

            if ((x3 <= 10 && y3 <= 10) or (10 <= x3 && 10 <= y3)) {
              screen->data[(y * kScreenWidth + x) * 3] = 255;
              screen->data[(y * kScreenWidth + x) * 3 + 1] = 255;
              screen->data[(y * kScreenWidth + x) * 3 + 2] = 255;
            } else {
              // black
            }
          } else { // different
            const auto x3 = mod_20(x2); // std::abs(x2) % 20;
            const auto y3 = mod_20(y2); // std::abs(y2) % 20;
            if ((x3 <= 10 && y3 <= 10) or (10 <= x3 && 10 <= y3)) {
              // black
            } else {
              screen->data[(y * kScreenWidth + x) * 3] = 255;
              screen->data[(y * kScreenWidth + x) * 3 + 1] = 255;
              screen->data[(y * kScreenWidth + x) * 3 + 2] = 255;
            }
          }
        }


      }
    }

    std::stringstream ss;
    ss << camera.focus();

    cv::putText(*screen, "Position: " + ss.str(), {0, 10}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);

    ss.str(""); ss << camera.VectorX();
    cv::putText(*screen, "VectorX: " + ss.str(), {0, 25}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
    ss.str(""); ss << camera.VectorY();
    cv::putText(*screen, "VectorY: " + ss.str(), {0, 40}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
    ss.str(""); ss << camera.VectorZ();
    cv::putText(*screen, "VectorZ: " + ss.str(), {0, 55}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);

    {
      std::lock_guard lck(m);
      clean = true;
    }
    cv.notify_one();
    cv::imshow("Window", *screen);
    int key = cv::waitKeyEx(0);
    if (key == blackhole::kEscape) break;

    switch (key) {
      case 'w': camera.MoveForward(1); break;
      case 'W': camera.MoveForward(10); break;

      case 'a': camera.MoveSideways(-1); break;
      case 'A': camera.MoveSideways(-10); break;

      case 's': camera.MoveForward(-1); break;
      case 'S': camera.MoveForward(-10); break;

      case 'd': camera.MoveSideways(1); break;
      case 'D': camera.MoveSideways(10); break;

      case 'q': camera.RotateSelf(-0.01); break;
      case 'Q': camera.RotateSelf(-0.1); break;

      case 'e': camera.RotateSelf(0.01); break;
      case 'E': camera.RotateSelf(0.1); break;

      case 'i': camera.RotateVertically(0.01); break;
      case 'I': camera.RotateVertically(0.1); break;

      case 'k': camera.RotateVertically(-0.01); break;
      case 'K': camera.RotateVertically(-0.1); break;

      case 'j': camera.RotateHorizontally(-0.01); break;
      case 'J': camera.RotateHorizontally(-0.1); break;

      case 'l': camera.RotateHorizontally(0.01); break;
      case 'L': camera.RotateHorizontally(0.1); break;

      case 'z': camera.MoveVertical(-1); break;
      case 'Z': camera.MoveVertical(-10); break;

      case 'x': camera.MoveVertical(1); break;
      case 'X': camera.MoveVertical(10); break;
    }

  }

  {
    std::lock_guard lck(m);
    die = true;
  }
  cv.notify_all();

  if (clean_thread.joinable())
    clean_thread.join();

  return 0;
}
