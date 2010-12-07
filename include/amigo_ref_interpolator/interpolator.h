#include <ros/ros.h>
#include <amigo_msgs/ref_point.h>

namespace refgen {
	
	class RefGenerator
	{
	private:
	

    int dir;
    bool reset;
    int signum(double a);
    double x;
    double vel;
    double vel_last;
    bool ready;

    public:
	  RefGenerator();
     ~RefGenerator();
    

      void setRefGen(double x_reset);
      amigo_msgs::ref_point generateReference(double x_desired, double max_vel, double max_acc, double dt, bool stop, double EPS);
   
    };

	
}


