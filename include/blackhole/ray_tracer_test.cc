//
// Created by YongGyu Lee on 2022/11/15.
//

#include <iostream>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "blackhole/camera.h"
#include "blackhole/cv_key.h"
#include "blackhole/object.h"
#include "blackhole/object/object_manager.h"
#include "blackhole/ray_tracer.h"

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
  std::cout <<
            "W: move forward\n"
            "S: move backward\n"
            "A: move left\n"
            "D: move right\n"
            "Z: move up\n"
            "X: move down\n"
            "Q: flip left\n"
            "E: flip right\n"
            "I: look down\n"
            "K: look up\n"
            "J: look left\n"
            "L: look right\n"
            "Press with shift: Move/Flip/Look faster\n";

  constexpr const int kScreenWidth = 1600;
  constexpr const int kScreenHeight = 900;

  blackhole::Camera<double> camera(kScreenWidth, kScreenHeight, blackhole::pi / 2);
  camera.MoveTo(-200, 120, 10);

  cv::VideoWriter out_capture("/Users/yonggyulee/video2.avi",
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 29, cv::Size(kScreenWidth, kScreenHeight), true);

  auto& manager = blackhole::ObjectManager<double>::GetInstance();

  auto id = manager.InsertObject<blackhole::Rectangle<double>>(
    100, 253, 199,
    100, 0, 199,
    100, 0, 0,
    100, 253, 0
  );
  id.second->SetTexture(cv::imread("/Users/yonggyulee/mooni.jpeg"));
  id.second->name("Mooni");

  auto id2 = manager.InsertObject<blackhole::Rectangle<double>>(
    100, 0, 300,
    100, -200, 300,
    100, -200, 0,
    100, 0, 0
  );
  id2.second->SetTexture(cv::imread("/Users/yonggyulee/karina.jpeg"));
  id2.second->name("Karina");

  auto id3 = manager.InsertObject<blackhole::Rectangle<double>>(
    -100, 50, 100,
    -100, -50, 100,
    -100, -50, 0,
    -100, 50, 0
  );
  id3.second->SetTexture(cv::imread("/Users/yonggyulee/winter.jpg"));
  id3.second->name("Winter");

  manager.InsertObject<blackhole::InfinitePlane<double>>(cv::Vec3d(0,0,0), blackhole::ChessPattern2D<double>{10});

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


  int key;
  for (;;) {
    const auto t1 = std::chrono::high_resolution_clock::now();
    {
      std::unique_lock lck(m);
      cv.wait(lck, [&](){ return !clean; });
    }


    const auto fv = camera.focus_vector();
    for (int x = 0; x < kScreenWidth; ++x) {
      for (int y = 0; y < kScreenHeight; ++y) {
        blackhole::RayTracer ray_tracer(camera.focus(), camera.PixelVector(x, y, fv));

        const auto b = ray_tracer.Prograde(manager, screen->data + (y * kScreenWidth + x) * 3, 10);
#ifndef NDEBUG
        if (x % 100 == 99 && y % 100 == 99) {
          cv::imshow("Window", *screen);
          key = cv::waitKey(1);
          if (key != -1) goto kGOTO_LOOP_END;
        }
#endif
        if (b) continue;
      }
    }
    {
      std::stringstream ss;
      ss << camera.focus();

      cv::putText(*screen, "Position: " + ss.str(), {0, 10}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
      ss.str("");
      ss << camera.vector_x();
      cv::putText(*screen, "VectorX: " + ss.str(), {0, 25}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
      ss.str("");
      ss << camera.vector_y();
      cv::putText(*screen, "VectorY: " + ss.str(), {0, 40}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
      ss.str("");
      ss << camera.vector_z();
      cv::putText(*screen, "VectorZ: " + ss.str(), {0, 55}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
      ss.str("");
      ss << camera.fov() * 180.0 / blackhole::pi << " deg";
      cv::putText(*screen, "FoV: " + ss.str(), {0, 70}, cv::FONT_HERSHEY_PLAIN, 1, {0, 255, 0}, 1);
    }
    {
      const auto buf = screen;
      {
        std::lock_guard lck(m);
        clean = true;
      }
      cv.notify_one();
      out_capture.write(*buf);
      const auto t2 = std::chrono::high_resolution_clock::now();
      const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
      std::cout << "Took " << dt << "ms\n";
      cv::imshow("Window", *buf);
    }
    key = cv::waitKeyEx(0);
    kGOTO_LOOP_END:;
    if (key == blackhole::kEscape)
      break;

    const auto move_d = 0.9;
    const auto rotate_d = blackhole::pi / 180.0;
    const auto faster = 10;

    switch (key) {
      case 'w': camera.MoveX(move_d); break;
      case 'W': camera.MoveX(move_d * faster); break;

      case 's': camera.MoveX(-move_d); break;
      case 'S': camera.MoveX(-move_d * faster); break;

      case 'a': camera.MoveY(-move_d); break;
      case 'A': camera.MoveY(-move_d * faster); break;

      case 'd': camera.MoveY(move_d); break;
      case 'D': camera.MoveY(move_d * faster); break;

      case 'z': camera.MoveZ(-move_d); break;
      case 'Z': camera.MoveZ(-move_d * faster); break;

      case 'x': camera.MoveZ(move_d); break;
      case 'X': camera.MoveZ(move_d * faster); break;

      case 'q': camera.RotateX(-rotate_d); break;
      case 'Q': camera.RotateX(-rotate_d * faster); break;

      case 'e': camera.RotateX(rotate_d); break;
      case 'E': camera.RotateX(rotate_d * faster); break;

      case 'i': camera.RotateY(rotate_d); break;
      case 'I': camera.RotateY(rotate_d * faster); break;

      case 'k': camera.RotateY(-rotate_d); break;
      case 'K': camera.RotateY(-rotate_d * faster); break;

      case 'j': camera.RotateZ(-rotate_d); break;
      case 'J': camera.RotateZ(-rotate_d * faster); break;

      case 'l': camera.RotateZ(rotate_d); break;
      case 'L': camera.RotateZ(rotate_d * faster); break;

      case '-': camera.fov(camera.fov() - blackhole::pi / 54); break;
      case '+': camera.fov(camera.fov() + blackhole::pi / 54); break;
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
