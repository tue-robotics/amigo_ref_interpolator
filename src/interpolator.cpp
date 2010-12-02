#include <ros/ros.h>
#include <amigo_ref_interpolator/interpolator.h>
#include <amigo_msgs/ref_point.h>

using namespace controller;


RefGenerator::RefGenerator()
{ x=0.0, x_desired=0.0, EPS=0.0001, dir=1, vel_mag=0.0, decide=false, acc=false,stl=true,con=false, new_cmd=false, time_of_last_cycle_=0.0;
}

RefGenerator::~RefGenerator()
{ 
}

int RefGenerator::signum(double a)
  {
  if (a < 0) 
    return -1;
  if (a >= 0) 
    return 1;
  else
    return 1;
}

void RefGenerator::traj_controller(double x, double x_desired, double max_vel, double max_acc, int stopping){
    
	double delta_x = x_desired - x;
	dir_desired = signum(delta_x);
	
	stop = stopping;
   
	//////////// if in steady phase ////////////////
	if (stl){
	  if (stop == 1){
		///ROS_INFO("stopped");
	  }
	  else if (fabs(delta_x) > EPS){
	     acc=true;
	     stl=false;	
	     ///ROS_INFO("start moving");
	  }
	
	}
	    
	//////////// if in acceleration phase ////////////////   
	else if (acc){
      
	  //if new desired position is in front of x
	  if (dir * delta_x >= 0.0){
	    decide=true;
      }
      //if new desired position is behind x  
      else if (dir * delta_x < 0.0){
	    acc=false;
	    dec=true;
	  }  
    	
    }
	
	////////// if in constant phase /////////////////
	
	else if (con){
	  //if new desired position is in front of x
	  if (dir * delta_x >= 0.0){
	    decide=true;
      }
      //if new desired position is behind x  
      else if (dir * delta_x < 0.0){
	    con=false;
	    dec=true;
	  }  
	}  
	  
	////////// if in deceleration phase ///////////////

    else if (dec){

      //if new desired position is in front of x
	  if (dir * delta_x >= 0.0){
	    decide=true;
	    dec=false;
      }
      //if new desired position is behind x  
      else if (dir * delta_x < 0.0){
	    dec=true;
	  }  		
    }
}


amigo_msgs::ref_point RefGenerator::generateReference(double x,double x_desired, double vel, double max_vel, double max_acc,double dt, int stopping){

   amigo_msgs::ref_point return_ref;
   double v_last = vel;
   double vel_mag = fabs(vel);
   double delta_t1=vel_mag/max_acc; //deceleration segment time
   double delta_x1 = 0.5*max_acc * (delta_t1) * (delta_t1); //deceleration distance
   double delta_x = x_desired - x;
   
   stop = stopping;
   if (stop ==1){
	   decide = false;
	   stl = false;
	   acc = false;
	   con = false;
	   dec = true;
   }
      
    if (decide){
      //if deceleration required
      if (fabs(delta_x) <= delta_x1){
		con=false;
		acc=false;
		dec=true;
      }
      //if max vel reached
	  else if (vel_mag >= max_vel){
        dec=false;
		acc=false;
		con=true;
      }
      else{
		dec=false;
		acc=true;
		con=false;
	  }
    decide=false;
    }  

    //if in steady state
    if (stl){
      if (new_cmd)
        traj_controller(x, x_desired, max_vel, max_acc, stop);
    }

    //if in accereleration state
    if (acc){

      //if deceleration required
      if (fabs(delta_x) <= delta_x1){
		//ROS_ERROR("deze");
		acc=false;
		dec=true;
      }
      //if max vel reached
	  else if (vel_mag >= max_vel){
		acc=false;
		con=true;
      }
      else{
        //compute distance to travel
        delta_x = x_desired - x;
      
        //compute direction
        dir_desired = signum(delta_x);
      
        vel_mag += max_acc *dt; //accelerate
        vel_mag = std::min<double>(vel_mag,max_vel); //limit velocity magnitude

        //update position
        x+=dir_desired * vel_mag * dt;
        dir=dir_desired;
      }
    }
    
    //if in constant velocity state
    else if (con){
      
      delta_t1=max_vel/max_acc; //deceleration segment time
      delta_x1 = 0.5*max_acc * (delta_t1) * (delta_t1); 
      
      //if deceleration required
      if (fabs(delta_x) <= fabs(delta_x1)){
	  dec=true;
          con=false;
	  }
      
      //compute distance to travel
      delta_x = x_desired - x;
            
      //update position      
      x += dir * vel_mag * dt;
     
    }   
    
   //if in deceleration state 
   else if (dec){
	      ///ROS_INFO("dec");
      //if velocity is approximately zero 
      if (vel_mag <=EPS){
		dec=false;
        stl=true;
        vel_mag = 0.0;
        traj_controller(x, x_desired, max_vel, max_acc, stop);
      }
      
      vel_mag += -max_acc *dt; //decelerate
      vel_mag = std::max<double>(vel_mag,0.0); //limit vel_mag
      
      //update position
      x+= dir * vel_mag *dt;
   }
   
   else{} 
   
   v = dir * vel_mag;
   a = std::max<double>(std::min<double>((v - v_last)/dt, max_acc),-max_acc);
   return_ref.pos = x;
   return_ref.vel = v;
   return_ref.acc = a;
   return_ref.pos_d = x_desired;

   
   return return_ref;
   
}
