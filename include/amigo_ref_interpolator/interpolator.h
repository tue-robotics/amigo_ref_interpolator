#include <ros/ros.h>
#include <amigo_msgs/ref_point.h>

namespace refgen {
	
	class RefGenerator
	{
	private:
	

  double vel_mag;
  
  int dir;
  int dir_desired;


  double x_desired;

  bool new_cmd;

  bool stl;
  bool acc;
  bool con;
  bool dec;  
  bool decide;

  double EPS;
  double time_of_last_cycle_;

  public:
	 RefGenerator();
    ~RefGenerator();
    
    int signum(double a);

     void traj_controller(double x, double x_desired, double max_vel, double max_acc, int stop);
  
     amigo_msgs::ref_point generateReference(double x, double x_desired, double vel, double max_vel, double max_acc, double dt, int stop);
  
    double x;
    double v;
    double a;
    
    int stop;
	
};

	
}

