#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <filesystem>

const double alpha = 1;
const double length_x = 150;
const double length_y = 150;
const int final_time = 100;
const int numXPoints = 150;
const int numYPoints = 150;
const double deltaX = length_x / (numXPoints);
const double deltaY = length_y / (numYPoints);
const double time_step = 0.1;
const int num_time_steps = static_cast<int>(final_time / time_step);

using Matrix = std::vector<std::vector<double>>;

Matrix initialize_temperature(int num_x_points, int num_y_points,
                              double delta_x, double delta_y) {
  Matrix temperature(num_x_points, std::vector<double>(num_y_points, 0.0));
  double centerX = (numXPoints / 2) * deltaX;
  double centerY = (numYPoints / 2) * deltaY;
  double radius = std::min(centerX, centerY) / 2;
  for (int i = 0; i < num_x_points; ++i) {
    for (int j = 0; j < num_y_points; ++j) {
      double x = (i + 1) * delta_x;
      double y = (j + 1) * delta_y;
      double distance =
          sqrt((x - centerX) * (x - centerX) + (y - centerY) * (y - centerY));
      temperature[i][j] = exp(-distance / radius) * 10;
    }
  }
  return temperature;
}

void apply_boundary_conditions(Matrix& temperature) {
  int num_x_points = temperature.size();
  int num_y_points = temperature[0].size();
  for (int i = 0; i < num_x_points; ++i) {
    temperature[i][0] = 0.0;
    temperature[i][num_y_points - 1] = 0.0;
  }
  for (int j = 0; j < num_y_points; ++j) {
    temperature[0][j] = 0.0;
    temperature[num_x_points - 1][j] = 0.0;
  }
}

void solve_heat_equation(Matrix& temperature, double time_step, double alpha,
                         double delta_x, double delta_y) {
  int num_x_points = temperature.size();
  int num_y_points = temperature[0].size();
  Matrix new_temperature = temperature;

  for (int i = 1; i < num_x_points - 1; ++i) {
    for (int j = 1; j < num_y_points - 1; ++j) {
      new_temperature[i][j] =
          temperature[i][j] +
          alpha * time_step *
              ((temperature[i + 1][j] - 2 * temperature[i][j] +
                temperature[i - 1][j]) /
                   (delta_x * delta_x) +
               (temperature[i][j + 1] - 2 * temperature[i][j] +
                temperature[i][j - 1]) /
                   (delta_y * delta_y));
    }
  }
  apply_boundary_conditions(new_temperature);
  temperature = new_temperature;
}

void save_to_file(const Matrix& temperature, int timestep) {
  std::filesystem::path dir_path = "../run";
  if (!std::filesystem::exists(dir_path)) {
      if (!std::filesystem::create_directory(dir_path)) {
          std::cerr << "Error creating directory: " << dir_path.string() << std::endl; // .string()が必要
          exit(1);
      }
  }

  std::string filename = "../run/output_" + std::to_string(timestep) + ".csv";
  std::ofstream file(filename);
  for (const auto& row : temperature) {
    std::ostringstream line;
    for (const auto& value : row) {
      if (std::isnan(value)) {
        throw std::runtime_error(
            "NaN detected in temperature matrix at timestep " +
            std::to_string(timestep));
      }
      line << value << ",";
    }
    std::string line_str = line.str();
    line_str.pop_back();
    file << line_str << "\n";
  }
}

int main() {
  Matrix temperature =
      initialize_temperature(numXPoints, numYPoints, deltaX, deltaY);

  auto start_time = std::chrono::high_resolution_clock::now();
  for (int t = 0; t < num_time_steps; ++t) {
    solve_heat_equation(temperature, time_step, alpha, deltaX, deltaY);
  //  結果の保存関数
   save_to_file(temperature, t);
  }
  auto end_time = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> execution_time = end_time - start_time;
  std::cout << "Total execution time: " << execution_time.count() << " seconds"
            << std::endl;

  return 0;
}
