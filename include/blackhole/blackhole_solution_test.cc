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
#include "blackhole/config.h"
#include "blackhole/cv_key.h"
#include "blackhole/object.h"
#include "blackhole/object/object_manager.h"
#include "blackhole/ray_tracer.h"
#include "blackhole/blackhole_solution.h"

#include "opencv2/opencv.hpp"

#if defined(BH_USE_OPENCL)
#if defined(__APPLE__) || defined(__MACOSX)
#include "OpenCL/cl.h"
#else
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
    #include <CL/cl.h>
#endif
#endif

void btncbf(int event, int x, int y, int flags, void* userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    std::cout << x << ", " << y << '\n';
  }
}

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

  constexpr const int kScreenWidth = 1920 / 2;
  constexpr const int kScreenHeight = 1080 / 2;
  using value_type = double;
  using point_type = cv::Vec<value_type, 3>;
  using vector_type = cv::Vec<value_type, 3>;
  using matrix_type = cv::Matx<value_type, 3, 3>;

  blackhole::Camera<value_type> camera(kScreenWidth, kScreenHeight, blackhole::pi / 2);
  camera.MoveTo(-2000, 0, 400);
//  camera.RotateY(0.1);
//  camera.fov(camera.fov() / 3);

  namespace fs = std::filesystem;
  namespace bh = blackhole;

  const auto save_dir = blackhole::timed_output_dir();
  fs::create_directories(save_dir);
  cv::VideoWriter out_capture(save_dir/"video.avi",
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 29, cv::Size(kScreenWidth, kScreenHeight), true);

  auto& manager = blackhole::ObjectManager<value_type>::GetInstance();

//  auto id = manager.InsertObject<blackhole::Rectangle>(
//    100, 253, 199,
//    100, 0, 199,
//    100, 0, 0,
//    100, 253, 0
//  );
//  id.second->SetTexture(bh::resource_image("mooni.jpeg"));
//  id.second->name("Mooni");
//
//  auto id2 = manager.InsertObject<blackhole::Rectangle>(
//    500, 200, 300,
//    500, 0, 300,
//    500, 0, 0,
//    500, 200, 0
//  );
//  id2.second->SetTexture(bh::resource_image("karina.jpeg"));
//  id2.second->name("Karina");
//
//  auto id3 = manager.InsertObject<blackhole::Rectangle>(
//    500, 0, 100,
//    500, -100, 100,
//    500, -100, 0,
//    500, 0, 0
//  );
//  id3.second->SetTexture(bh::resource_image("winter.jpg"));
//  id3.second->name("Winter");

  auto id_acc_disc = manager.InsertObject<blackhole::Annulus>(
    point_type{1000, 1000, 0},
    point_type{-1000, 1000, 0},
    point_type{-1000, -0100, 0},
    point_type{1000, -1000, 0},
    1000,
    100
  );
  id_acc_disc.second->SetTexture(bh::resource_image("acc_disc.png"));
  id_acc_disc.second->name("Accretion Disc");


//  manager.InsertObject<blackhole::InfinitePlane>(vector_type(0,0,0), blackhole::ChessPattern2D<value_type>{100}).second->name("Chess");

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

//  blackhole::StaticBlackhole<value_type> blackhole({0, 0, 0}, 4);
  auto& blackhole = *manager.InsertObject<blackhole::StaticBlackhole>(point_type(0,0,0), 10).second;
  blackhole.name("Blackhole");


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
        if (x == 592 && y == 266)
          bool debug = true;

        const auto pv = camera.PixelVector(x, y, fv) - blackhole.position();
        const auto F = camera.focus() - blackhole.position();

        vector_type yv = cv::normalize(F - pv);
        vector_type zv = cv::normalize(pv.cross(yv));
        if (std::isnan(zv[0]))
          zv = vector_type(1, 0, 0);
        vector_type xv = cv::normalize(yv.cross(zv));

        auto matrix_reconversion = matrix_type(
          zv[0], yv[0], xv[0],
          zv[1], yv[1], xv[1],
          zv[2], yv[2], xv[2]);
        auto matrix_conversion = matrix_reconversion.inv();

        auto convertedCameraFocus = matrix_conversion * F;
        auto convertedPixel = matrix_conversion * pv;

        const auto b = convertedCameraFocus[2];
        value_type sol = -1;
        if (b >= blackhole.b_c()) {
          sol = blackhole.SolveG(b);
        }
        const auto periapsis = b < blackhole.b_c() ? 1.0 / (3 * blackhole.mass()) : sol;
        auto r0 = std::sqrt(convertedCameraFocus.dot(convertedCameraFocus));

        value_type phi = atan(convertedCameraFocus[2] / convertedCameraFocus[1]);
        value_type dphi = 0;
        value_type dphi_prev = 0;
        value_type u = 1. / r0;
        value_type r = r0;

        const value_type integrate_begin = u;
        const value_type integrate_end = periapsis;

        int nstep = 20;
        int nstep_safe = nstep - 1;
        value_type du = (integrate_end - integrate_begin) * (1.0 / nstep);
        const value_type du_h = du / 2.;
        const value_type b_invsq = 1. / (b * b);

        auto light_vector = convertedCameraFocus;
        auto light_vector_prev = convertedCameraFocus;
        point_type light_vector_original = camera.focus();
        point_type light_vector_prev_original = camera.focus();

        point_type inter;
        auto color_dst = screen->data + (y * kScreenWidth + x) * 3;

        // trapezoid integration
        for (int i = 0; i < nstep_safe; ++i) {
          u += du;
          dphi = blackhole.InvSqrtG(u, b);

          phi += (dphi_prev + dphi) * du_h;
          r = 1. / u;

          light_vector = { 0, r * cos(phi), r * sin(phi)};
          light_vector_original = (matrix_reconversion * light_vector) + blackhole.position();

          dphi_prev = dphi;

          if (const auto& obj = manager.FindCollision(light_vector_prev_original, light_vector_original, &inter); obj != nullptr) {
            const auto c = obj->color(inter);
            color_dst[0] = c[0];
            color_dst[1] = c[1];
            color_dst[2] = c[2];
            goto kGOTO_PIXEL_LOOP_END;
          }

          light_vector_prev = light_vector;
          light_vector_prev_original = light_vector_original;
        }
        {
          u += du * 0.9;
          dphi = blackhole.InvSqrtG(u, b);

          phi += (dphi_prev + dphi) * du_h;
          r = 1. / u;

          light_vector = { 0, r * cos(phi), r * sin(phi)};
          light_vector_original = matrix_reconversion * light_vector + blackhole.position();

          dphi_prev = dphi;

          if (const auto& obj = manager.FindCollision(light_vector_prev_original, light_vector_original, &inter); obj != nullptr) {
            const auto c = obj->color(inter);
            color_dst[0] = c[0];
            color_dst[1] = c[1];
            color_dst[2] = c[2];
            goto kGOTO_PIXEL_LOOP_END;
          }

          light_vector_prev = light_vector;
          light_vector_prev_original = light_vector_original;
        }

        if (sol < 0) {
          if (const auto& obj = manager.FindCollision(light_vector_prev_original, blackhole.center(), &inter); obj != nullptr) {
            const auto c = obj->color(inter);
            color_dst[0] = c[0];
            color_dst[1] = c[1];
            color_dst[2] = c[2];
          }
          goto kGOTO_PIXEL_LOOP_END;
        }

        for (int i = 0; i < nstep_safe; ++i) {
          u -= du;
          dphi = blackhole.InvSqrtG(u, b);

          phi += (dphi_prev + dphi) * du_h;
          r = 1. / u;

          light_vector = { 0, r * cos(phi), r * sin(phi)};
          light_vector_original = matrix_reconversion * light_vector + blackhole.position();

          if (const auto& obj = manager.FindCollision(light_vector_prev_original, light_vector_original, &inter); obj != nullptr) {
            const auto c = obj->color(inter);
            color_dst[0] = c[0];
            color_dst[1] = c[1];
            color_dst[2] = c[2];
            goto kGOTO_PIXEL_LOOP_END;
          }

          dphi_prev = dphi;

          light_vector_prev = light_vector;
          light_vector_prev_original = light_vector_original;
        }

        kGOTO_PIXEL_LOOP_END:;

#ifndef NDEBUG
        if (x % 33 == 32 && y % 33 == 32) {
          cv::imshow("Window", *screen);
          key = cv::waitKey(1);
          if (key != -1) goto kGOTO_LOOP_END;
        }
#endif
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
      cv::setMouseCallback("Window", btncbf, nullptr);
    }
    key = cv::waitKeyEx(1);
    kGOTO_LOOP_END:;
    if (key == blackhole::kEscape)
      break;

    const auto move_d = 10;
    const auto rotate_d = blackhole::pi / 1800.0;
    const auto faster = 10;
//    blackhole.MoveZ(10);
//
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

      case blackhole::kUp: blackhole.MoveZ(5); break;
      case blackhole::kDown: blackhole.MoveZ(-5); break;
      case blackhole::kLeft: blackhole.MoveY(5); break;
      case blackhole::kRight: blackhole.MoveY(-5); break;
      case ',': blackhole.MoveX(5); break;
      case '.': blackhole.MoveX(-5); break;

//      case '1': id_acc_disc.second->RotateX(rotate_d); break;
//      case '!': id_acc_disc.second->RotateX(-rotate_d); break;
//
//      case '2': id_acc_disc.second->RotateY(rotate_d); break;
//      case '@': id_acc_disc.second->RotateY(-rotate_d); break;
//
//      case '3': id_acc_disc.second->RotateZ(rotate_d); break;
//      case '#': id_acc_disc.second->RotateZ(-rotate_d); break;
    }
    id_acc_disc.second->RotateZ(blackhole::pi / 180);

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
