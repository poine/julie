#ifndef JULIE_CONTROL__JAC_ODOMETRY_H
#define JULIE_CONTROL__JAC_ODOMETRY_H

namespace julie_controller {
  
  class JulieOdometry {
    public:
      JulieOdometry();
      void update();
    private:
      double steering_angle_;
    };
  
}


#endif // JULIE_CONTROL__JAC_ODOMETRY
