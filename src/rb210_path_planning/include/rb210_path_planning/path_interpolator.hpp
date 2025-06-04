#include <vector>
#include <cmath>

namespace interpolate{

class Path{
public:
  Path(std::vector<double> start_position,std::vector<double> goal_position){
    if(start_position.size()!=goal_position.size()){
      return;
    }
    start_ = start_position;
    goal_ = goal_position;
  };
  ~Path(){};
  // 通过step_size插值
  std::vector<std::vector<double>> InterpolateWithStepSize(double step_size){
    std::vector<std::vector<double>> interpolate_path;
    if (start_.size() == 0) {
      return interpolate_path;
    }
    int dof = start_.size();
    // 计算 joint space 的距离（欧几里得范数）
    double total_distance = 0.0;
    for (int i = 0; i < dof; ++i) {
        double diff = goal_[i] - start_[i];
        total_distance += diff * diff;
    }
    total_distance = std::sqrt(total_distance);
    // 计算插值点数（至少包含起点和终点）
    int num_steps = std::max(1, static_cast<int>(std::ceil(total_distance / step_size)));
    for (int step = 0; step <= num_steps; ++step) {
      double ratio = static_cast<double>(step) / static_cast<double>(num_steps);
      std::vector<double> point(dof);
      for (size_t i = 0; i < dof; ++i) {
          point[i] = start_[i] + ratio * (goal_[i] - start_[i]);
      }
      interpolate_path.push_back(point);
    }

    return interpolate_path;
  }


private:
  std::vector<double> start_;
  std::vector<double> goal_;


};

};
