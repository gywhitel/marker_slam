# ifndef HELPER
# define HELPER

#include <cmath>
#include <memory>
#include <tf2/LinearMath/Transform.h>

// Degrees to radians
constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Radians to degrees
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }


/// @brief Exponential Moving Average filter on tf2::Transform
class EMA
{
  private:
    uint count_;
    double a_;   // smoothing factor
    tf2::Vector3 pos_;  // smoothed position
    tf2::Quaternion q_; // smoothed quaternion

  public:
    /// @brief Initiate EMA
    /// @param a smoothing factor. Smaller a assigns smalled weight to current data and  yield smoother data 
    EMA(double a){
      a_ = a;
      count_ = 0;
    }

    tf2::Transform update(const tf2::Transform& T){
      tf2::Vector3 p = T.getOrigin();
      tf2::Quaternion q = T.getRotation();

      if (count_ == 0){
        pos_ = p;
        q_ = q;
      }
      else{
        pos_ += a_ * (p - pos_);
        q_ = q_.slerp(q, a_);
      }
      count_++;
      return tf2::Transform(q_, pos_);
    }
   
};

# endif
