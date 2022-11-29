//
// Created by YongGyu Lee on 2022/11/29.
//

#include <cstring>
#include <vector>
#include <TColor.h>
#include <TROOT.h>
#include <TF3.h>

#include "opencv2/opencv.hpp"

#include "TRootCanvas.h"
#include "TApplication.h"
#include "TCanvas.h"
#include "TGraph2D.h"
#include "TAxis.h"
#include "TH3F.h"

#include "blackhole/camera.h"
#include "blackhole/config.h"
#include "blackhole/cv_key.h"
#include "blackhole/object.h"
#include "blackhole/object/object_manager.h"
#include "blackhole/ray_tracer.h"
#include "blackhole/blackhole_solution.h"

namespace std {

template<size_t i, typename T, int n, std::enable_if_t<(i < n), int> = 0>
T& get(cv::Vec<T, n>& v) { return v.val[i]; }
template<size_t i, typename T, int n, std::enable_if_t<(i < n), int> = 0>
const T& get(const cv::Vec<T, n>& v) { return v.val[i]; }
template<size_t i, typename T, int n, std::enable_if_t<(i < n), int> = 0>
T&& get(cv::Vec<T, n>&& v) { return std::move(v.val[i]); }
template<size_t i, typename T, int n, std::enable_if_t<(i < n), int> = 0>
const T&& get(const cv::Vec<T, n>&& v) { return std::move(v.val[i]); }

} // namespace std


blackhole::StaticBlackhole<double> bhole(cv::Vec3d(1, 1, 1), 10);

class Plotter {
 public:
  using graph_type = TGraph2D;

  Plotter() {
    ((TRootCanvas *)canvas->GetCanvasImp())
      ->Connect("CloseWindow()", "TApplication", app.get(), "Terminate()");
  }

  template<typename P>
  void draw(const std::vector<P>& points, Color_t color, const char* option = "LINE SAME") {
    colors.push_back(color);
    options_.emplace_back(option);
    auto& g = graphs_.emplace_back(std::make_unique<graph_type>());

    for (int i = 0; i < points.size(); ++i) {
      const auto& point = points[i];

      if      (std::get<0>(point) < x0) x0 = std::get<0>(point);
      else if (std::get<0>(point) > x1) x1 = std::get<0>(point);
      if      (std::get<1>(point) < y0) y0 = std::get<1>(point);
      else if (std::get<1>(point) > y1) y1 = std::get<1>(point);
      if      (std::get<2>(point) < z0) z0 = std::get<2>(point);
      else if (std::get<2>(point) > z1) z1 = std::get<2>(point);

      g->SetPoint(i, std::get<0>(point), std::get<1>(point), std::get<2>(point));
    }
  }

  void show() {
    canvas->cd();
    canvas->SetFillColor(kGray);
    gPad->SetGrid(1, 1); gPad->Update();

//    std::unique_ptr<TH3F> frame_ = std::make_unique<TH3F>(
//      "frame3d", "frame3d",
//      10, x0, x1,
//      10, y0, y1,
//      10, z0, z1);
//    frame_->Draw();
    auto edge_graph = std::make_unique<TGraph2D>();
    edge_graph->SetPoint(0, x0 - 100, y0 - 100, z0 - 100);
    edge_graph->SetPoint(1, x1 + 100, y1 + 100, z1 + 100);
    edge_graph->SetMarkerSize(0);
    edge_graph->Draw("P");
//    auto b = std::make_unique<TF3>("f3", "(x-1)*(x-1) + (y-1)*(y-1) + (z-1)*(z-1) - 100", -9, 11,-9,11,-9,11);
//    b->SetLineColor(kBlack);
//    b->SetFillColor(kBlack);
//    b->SetLineWidth(2);
//    b->Draw("FBBB SAME");

    for (int i = 0; i < graphs_.size(); ++i) {
      auto& graph = graphs_[i];
      auto& color = colors[i];
      auto& option = options_[i];

      graph->SetLineColor(color);
      graph->SetMarkerColor(color);
      graph->SetLineWidth(2);
      graph->SetMarkerSize(2);
      graph->SetMarkerStyle(kFullCircle);
//      if (i == 0)
//        graph->Draw("LINE");
//      else
        graph->Draw(option.c_str());
    }


    canvas->Update();
    app->Run();
  }

  std::unique_ptr<TApplication> app = std::make_unique<TApplication>("app", nullptr, nullptr);
  std::unique_ptr<TCanvas> canvas = std::make_unique<TCanvas>("c1","c1");

  std::vector<std::unique_ptr<TGraph2D>> graphs_;
  std::vector<std::string> options_;
  double x0 = -10;
  double x1 = 10;
  double y0 = -10;
  double y1 = 10;
  double z0 = -10;
  double z1 = 10;
  std::vector<Color_t> colors;
};

int main() {
  constexpr const int kScreenWidth = 1920 / 2;
  constexpr const int kScreenHeight = 1080 / 2;
  using value_type = double;
  using point_type = cv::Vec<value_type, 3>;
  using vector_type = cv::Vec<value_type, 3>;
  using matrix_type = cv::Matx<value_type, 3, 3>;

  blackhole::Camera<value_type> camera(kScreenWidth, kScreenHeight, blackhole::pi / 2);
  camera.MoveTo(-1000, 0, 400);
  camera.RotateY(-0.3);
//  camera.RotateY(0.1);
//  camera.fov(camera.fov() / 3);

  namespace fs = std::filesystem;
  namespace bh = blackhole;


  Plotter plotter;

  const auto fv = camera.focus_vector();
  for (int x = 0; x < kScreenWidth; ++x) {
    for (int y = 0; y < kScreenHeight; ++y) {
//      if (x != 0 || y != 0) continue;
      if (x % 140 != 0 || y % 140 != 0) continue;
      std::cout << x << ", " << y << '\n';

      const auto pv = camera.PixelVector(x, y, fv) - bhole.position();
      const auto F = camera.focus() - bhole.position();

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

      auto b = convertedCameraFocus[2];
      value_type sol = -1;
      if (b > bhole.b_c()) {
        sol = bhole.SolveG(b);
      }
      const value_type periapsis_u = b > bhole.b_c() ? sol : (0.156 * bhole.mass() / b);
      const value_type periapsis_r = 1.0 / periapsis_u;
      const value_type periapsis_r_safe = periapsis_r + (bhole.mass() * 0.0001); // TODO: Use dynamic based on the mass
      const value_type periapsis_u_safe = 1.0 / periapsis_r_safe;

      auto r0 = std::sqrt(convertedCameraFocus.dot(convertedCameraFocus));

      value_type phi = atan(convertedCameraFocus[2] / convertedCameraFocus[1]);
      value_type u = 1. / r0;
      value_type r = r0;

      // TODO: Use dynamic step size
      value_type dr = bhole.mass();

      auto light_pos = convertedCameraFocus; // light_pos_prev + light_vector * dr;
      point_type light_vector_original = camera.focus();

      // Assume r0 > periapsis_r
      std::vector<point_type> traces;
      traces.reserve(400);
      for (int i = 0; i < 150; ++i) {
        auto r1 = r;
        auto r2 = r1 - dr;

        if (r2 < periapsis_r_safe) {
          if (sol == -1) {
            break;
          }

          std::cout << "i = " << i << ", r2 = " << r2 << ", P = " << periapsis_r << '\n';
          r2 = periapsis_r_safe;
          dr = -dr;
//          phi += 0.1;
        }

        const value_type u1 = 1.0 / r1;
        const value_type u2 = 1.0 / r2;
        const auto du = u2 - u1;

        const value_type f_u = bhole.InvSqrtG((u1 + u2) / 2, b);
        phi += f_u * std::abs(du);
        r = r2;

        std::cout << "i = " << i << ", r = " << r << ", phi = " << phi << '\n';

        light_pos = {0, r * std::cos(phi), r * std::sin(phi)};
        light_vector_original = matrix_reconversion * light_pos + bhole.position();

        traces.emplace_back(light_vector_original);
      }

//      const auto pv = camera.PixelVector(x, y, fv) - bhole.position();
//      const auto F = camera.focus() - bhole.position();
//
//      vector_type yv = cv::normalize(F - pv);
//      vector_type zv = cv::normalize(pv.cross(yv));
//      if (std::isnan(zv[0]))
//        zv = vector_type(1, 0, 0);
//      vector_type xv = cv::normalize(yv.cross(zv));
//
//      auto matrix_reconversion = matrix_type(
//        zv[0], yv[0], xv[0],
//        zv[1], yv[1], xv[1],
//        zv[2], yv[2], xv[2]);
//      auto matrix_conversion = matrix_reconversion.inv();
//
//      auto convertedCameraFocus = matrix_conversion * F;
//      auto convertedPixel = matrix_conversion * pv;
//
//      const auto b = convertedCameraFocus[2];
//      value_type sol = -1;
//      if (b >= bhole.b_c()) {
//        sol = bhole.SolveG(b);
//      }
//      const auto periapsis = b < bhole.b_c() ? 0.156 * bhole.mass() / b : sol;
//      auto r0 = std::sqrt(convertedCameraFocus.dot(convertedCameraFocus));
//
//      value_type phi = atan(convertedCameraFocus[2] / convertedCameraFocus[1]);
//      value_type dphi = 0;
//      value_type dphi_prev = 0;
//      value_type u = 1. / r0;
//      value_type r = r0;
//
//      const value_type integrate_begin = u;
//      const value_type integrate_end = periapsis;
//
//      int nstep = 20;
//      int nstep_safe = nstep - 1;
//      value_type du = (integrate_end - integrate_begin) * (1.0 / nstep);
//      value_type du_h = du / 2.;
//      const value_type b_invsq = 1. / (b * b);
//
//      auto light_vector = convertedCameraFocus;
//      auto light_vector_prev = convertedCameraFocus;
//      point_type light_vector_original = camera.focus();
//      point_type light_vector_prev_original = camera.focus();
//
//      point_type inter;
//
//      std::vector<point_type> traces;
//      traces.reserve(nstep * 2);
//      // trapezoid integration
//      for (int i = 0; i < nstep_safe; ++i) {
//        u += du;
//        dphi = bhole.InvSqrtG(u, b);
//
//        phi += (dphi_prev + dphi) * du_h;
//        r = 1. / u;
//
//        light_vector = { 0, r * cos(phi), r * sin(phi)};
//        light_vector_original = (matrix_reconversion * light_vector) + bhole.position();
//
//        dphi_prev = dphi;
//
//        light_vector_prev = light_vector;
//        light_vector_prev_original = light_vector_original;
//        traces.emplace_back(light_vector_original);
//      }
//      {
//        u += du * 0.9;
//        dphi = bhole.InvSqrtG(u, b);
//
//        phi += (dphi_prev + dphi) * du_h;
//        r = 1. / u;
//
//        light_vector = { 0, r * cos(phi), r * sin(phi)};
//        light_vector_original = matrix_reconversion * light_vector + bhole.position();
//
//        dphi_prev = dphi;
//
//        light_vector_prev = light_vector;
//        light_vector_prev_original = light_vector_original;
//        traces.emplace_back(light_vector_original);
//      }
//
//      if (sol < 0) {
//
//      } else {
//        const value_type integrate_begin_2 = periapsis;
//        const value_type integrate_end_2 = (1.0 / r0) / 10.0;
//
//        du = (integrate_end_2 - integrate_begin_2) * (1.0 / nstep);
//        du_h = -du / 2;
//
//        for (int i = 0; i < nstep_safe; ++i) {
//          u += du;
//          dphi = bhole.InvSqrtG(u, b);
//
//          phi += (dphi_prev + dphi) * du_h;
//          r = 1. / u;
//
//          light_vector = { 0, r * cos(phi), r * sin(phi)};
//          light_vector_original = matrix_reconversion * light_vector + bhole.position();
//
//          dphi_prev = dphi;
//
//          light_vector_prev = light_vector;
//          light_vector_prev_original = light_vector_original;
//          traces.emplace_back(light_vector_original);
//        }
//      }
      

      plotter.draw(traces, rand() % 50);
    }
  }

  plotter.draw<point_type>({{1, 1, 1}}, kBlack, "ACP SAME");
  plotter.show();

  return 0;
}
