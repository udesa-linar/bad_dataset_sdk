// Headless counterpart of koide's initial_guess_manual "Estimate" button: reads the 2D-3D correspondences stored in
// <data_path>/calib.json (results.manual_correspondences = [[u, v, x, y, z], ...]), estimates the camera pose with
// koide's PoseEstimation (RANSAC rotation + robust least squares) and writes results.init_T_lidar_camera.
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <camera/create_camera.hpp>
#include <vlcal/common/estimate_pose.hpp>

int main(int argc, char** argv) {
  if (argc < 2) { std::cerr << "usage: initial_guess_from_points <data_path>" << std::endl; return 1; }
  const std::string data_path = argv[1];
  nlohmann::json config;
  { std::ifstream ifs(data_path + "/calib.json"); if (!ifs) { std::cerr << "no calib.json" << std::endl; return 1; } ifs >> config; }
  const std::string model = config["camera"]["camera_model"];
  const std::vector<double> intr = config["camera"]["intrinsics"];
  const std::vector<double> dist = config["camera"]["distortion_coeffs"];
  auto proj = camera::create_camera(model, intr, dist);
  const std::vector<std::vector<double>> pts = config["results"]["manual_correspondences"];
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>> corr;
  for (const auto& p : pts) corr.emplace_back(Eigen::Vector2d(p[0], p[1]), Eigen::Vector4d(p[2], p[3], p[4], 1.0));
  if (corr.size() < 3) { std::cerr << "at least 3 correspondences are necessary" << std::endl; return 1; }
  vlcal::PoseEstimation est;
  std::vector<bool> inliers;
  const Eigen::Isometry3d T_camera_lidar = est.estimate(proj, corr, &inliers);
  const Eigen::Isometry3d T_lidar_camera = T_camera_lidar.inverse();
  const Eigen::Vector3d t = T_lidar_camera.translation(); const Eigen::Quaterniond q(T_lidar_camera.linear());
  config["results"]["init_T_lidar_camera"] = std::vector<double>{t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
  int n_in = 0; for (bool b : inliers) n_in += b;
  std::ofstream(data_path + "/calib.json") << config.dump(2) << std::endl;
  std::cout << "correspondences " << corr.size() << ", inliers " << n_in << std::endl;
  std::cout << "T_lidar_camera t " << t.transpose() << " q " << q.coeffs().transpose() << std::endl;
  return 0;
}
