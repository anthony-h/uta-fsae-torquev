#include <stdio.h>
#include <math.h>
#define g 9.81
#define PI 3.141592

void main(void)
{
  //constants

  float mass_of_car, front_weight_distribution, front_unsprung_cg,
        rear_unsprung_mass, sprung_mass_of_car, distance_front_unsprung_cg,
        distance_rear_unsprung_cg, distance_of_sprung_cg_from_front_axle, 
        distance_sprung_mass_cg_and_roll_axis, cg_height, front_tread,
        rear_tread, wheel_base, distance_of_cg_from_front_axle, distance_of_cg_from_rear_axle,
        distance_between_roll_axis_and_cg, front_roll_center_height, rear_roll_center_height,
		front_roll_rate, rear_roll_rate, modified_front_roll_rate, modified_rear_roll_rate,
		drag_coefficient, center_of_pressure,rack_ratio,s_maxf,s_peakf,t_peakf,
		t_maxf_100,t_maxf,steering_angle_compliance,gear_ratio,wheel_diameter,
		steering_wheel_compliance,s_peakr,t_peakr,t_maxr_100,s_maxr,t_maxr,
        distance_lr_center,distance_rr_center,distance_rf_center,distance_lf_center;

  //Variables

  float torque_mode_selection,front_single_wheel_normal_load,rear_single_wheel_normal_load,front_wheels_lateral_load,rear_wheels_lateral_load, 
        normal_lateral_load_transfer_front,normal_lateral_load_transfer_rear,weight_transfer_rf,weight_transfer_lf,weight_transfer_rr,
        weight_transfer_lr,total_requested_torque, torque_front_left_output, torque_front_right_output, torque_rear_right_output,
		torque_rear_left_output,wheel_speed_rf,wheel_speed_lf,wheel_speed_rr,wheel_speed_lr,
        linear_velocity_rf,linear_velocity_lf,linear_velocity_rr,linear_velocity_lr,average_velocity,radius_of_turn_CG,steering_angle_rad;

  //Inputs
  float accel_long,accel_lat,throttle_position,front_left_motor_rpm,front_right_motor_rpm,rear_left_motor_rpm,rear_right_motor_rpm,chassis_roll_angle,
	    requested_torque_rear_left_motor,requested_torque_rear_right_motor,requested_torque_front_left_motor,requested_torque_front_right_motor,regen_brake_position,
		steering_wheel_angle;
        
  //initialization of constants/starting values
    torque_mode_selection = 1;
    mass_of_car = 326.58;
    front_weight_distribution = 55;
    front_unsprung_cg = 48;
    rear_unsprung_mass = 50;
    sprung_mass_of_car = 228.61;
    distance_front_unsprung_cg = 260.35/1000;
    distance_rear_unsprung_cg = 260.35/1000;
    distance_of_sprung_cg_from_front_axle = 760/1000;
    distance_sprung_mass_cg_and_roll_axis = 228.611/1000;
    cg_height = 292.1/1000;
    front_tread = 1244.6/1000;
    rear_tread = 1193.8/1000;
    wheel_base = 1676.4/1000;
    distance_of_cg_from_front_wheel_center = 778.9164/1000;
    distance_between_roll_axis_and_cg = 207.518;
    front_roll_center_height = 81.026/1000;
    rear_roll_center_height = 86.106/1000;
    front_roll_rate = 16016;
    rear_roll_rate = 19253.07;
    drag_coefficient = 0;
    center_of_pressure = 100/1000;
    accel_long = -1.0;
    accel_lat = 1.1;
    throttle_position = 70;
    front_left_motor_rpm = 9000;
    front_right_motor_rpm = 9000;
    rear_left_motor_rpm = 9000;
    rear_right_motor_rpm = 9000;
    s_peakr = 21715;
    t_peakr = 44.21;
    t_maxr_100 = 30;
    //s_peakf=17674.91; Commented out of matlab file
    //t_peakf=25.01; Commented out of matlab file
    //t_maxf_100=15; Commented out of matlab file
    regen_brake_position = 0;
    steering_wheel_compliance = 0.4;
    rack_ratio = 3.31;
    wheel_diameter = 514.604/1000;
    steering_wheel_angle = 0;
    gear_ratio = 0.076923;
    radius_of_turn_CG = 0;

  //initialization of starting variables. All of these variables are calculations based on the initial variables above.
	distance_of_cg_from_rear_wheel_axle=wheel_base-x_distance_of_cg_from_front_wheel_center;
	modified_front_roll_rate=front_roll_rate-
                             (((wheel_base-distance_of_sprung_cg_from_front_axle)*sprung_mass_of_car*g
                             *perpendicular_distance_between_sprung_mass_cg_and_roll_axis)/wheel_base);
    modified_rear_roll_rate=rear_roll_rate-
                            (((distance_of_sprung_cg_from_front_axle)*sprung_mass_of_car*g
                            *perpendicular_distance_between_sprung_mass_cg_and_roll_axis)/wheel_base);
    weight_transfer_front=(mass_of_car*x_distance_of_cg_from_rear_wheel_center-drag_coefficient*HA-
                        mass_of_car*accel_long*cg_height)/(2*wheel_base);
    weight_transfer_rear=(mass_of_car*x_distance_of_cg_from_front_wheel_center+drag_coefficient*HA+
                       mass_of_car*accel_long*cg_height)/(2*wheel_base);
    /*phi_angle=(sprung_mass_of_car*assumed_difference_between_roll_axis_and_cg_height*accel_lat)/
            (front_roll_rate+rear_roll_rate-sprung_mass_of_car*g*assumed_difference_between_roll_axis_and_cg_height); commented out in matlab file*/
    lateral_load_transfer_front=accel_lat*((sprung_mass_of_car*g/front_tread)*((((perpendicular_distance_between_sprung_mass_cg_and_roll_axis*modified_front_roll_rate)/
    (front_roll_rate+rear_roll_rate-sprung_mass_of_car*perpendicular_distance_between_sprung_mass_cg_and_roll_axis))+
		  (((wheel_base-distance_of_sprung_cg_from_front_axle)/wheel_base)*front_roll_center_height))))+
      ((front_unsprung_mass_front_unsprung_cg*g*front_unsprung_mass_cg)/front_tread);
    lateral_load_transfer_rear=accel_lat*((sprung_mass_of_car*g/rear_tread)*((((perpendicular_distance_between_sprung_mass_cg_and_roll_axis*modified_rear_roll_rate)/
      (front_roll_rate+rear_roll_rate-sprung_mass_of_car*perpendicular_distance_between_sprung_mass_cg_and_roll_axis))+
		  (((distance_of_sprung_cg_from_front_axle)/wheel_base)*rear_roll_center_height))))+
      ((front_unsprung_mass_front_unsprung_cg*g*rear_unsprung_mass_cg)/rear_tread);
	lateral_load_transfer_front=lateral_load_transfer_front/g;
	lateral_load_transfer_rear=lateral_load_transfer_rear/g;
    wheel_speed_rf=front_right_motor_rpm*gear_ratio;
    wheel_speed_lf=front_left_motor_rpm*gear_ratio;
    wheel_speed_rr=rear_right_motor_rpm*gear_ratio;
    wheel_speed_lr=rear_left_motor_rpm*gear_ratio;
	steering_wheel_angle=(steering_wheel_compliance/rack_ratio)*(PI*180);
    linear_velocity_rf = PI*wheel_diameter*wheel_speed_rf/60;
    linear_velocity_lf = PI*wheel_diameter*wheel_speed_lf/60;
    linear_velocity_rr = PI*wheel_diameter*wheel_speed_rr/60;
    linear_velocity_lr = PI*wheel_diameter*wheel_speed_lr/60;
    average_velocity = linear_velocity_lr+linear_velocity_rr+linear_velocity_lf+linear_velocity_rf;
    steering_angle_rad = (steering_wheel_angle/rack_ratio)*(PI/180);
	force_y_front=2;
	force_y_rear=2;


    if((throttle_position==0) && (regen_brake_position==0))
    {
      regen_brake_position=2;
    }
    /*if(throttle_position>0)
    {
      itp=throttle_position;
    }
    else                        itp is an undeclared and unexplained variable.
    {
      itp=regenerative_brake_position;
    }*/ 
    if((steering_angle_rad < -steering_angle_play) && (accel_lat==0))
    {
      distance_lr_center=wheel_base/tan(abs(steering_angle_rad));
	  distance_rr_center=front_tread+distance_lr_center;
	  distance_lf_center=sqrt((distance_lr_center*distance_lr_center)+(wheel_base*wheel_base));
	  distance_rf_center=sqrt((distance_rr_center*distance_rr_center)+(wheel_base*wheel_base));
	  radius_of_turn_CG=sqrt(((distance_lr_center+front_tread*0.5)*(distance_lr_center+front_tread*0.5))+(x_distance_of_cg_from_rear_wheel_center*x_distance_of_cg_from_rear_wheel_center));
    }
	if((steering_angle_rad >= steering_angle_play) && (accel_lat==0))
	{
	  distance_rr_center=wheel_base/tan(abs(steering_angle_rad));
	  distance_lr_center=front_tread+distance_rr_center;
	  distance_lf_center=sqrt((distance_rr_center*distance_rr_center)+(wheel_base*wheel_base));
	  distance_rf_center=sqrt((distance_lr_center*distance_lr_center)+(wheel_base*wheel_base));
	  radius_of_turn_CG=-sqrt(((distance_rr_center+front_tread*0.5)*(distance_rr_center+front_tread*0.5))+(x_distance_of_cg_from_rear_wheel_center*x_distance_of_cg_from_rear_wheel_center));
	}
	/*if((rCG==0) && (accel_lat==0))
	{
	  accel_lat=(average_velocity*average_velocity)/(rCG*g)
	}*/
    if (accel_lat>0)
    {
		weight_transfer_rf=weight_transfer_front-lateral_load_transfer_front;
		weight_transfer_lf=weight_transfer_front+lateral_load_transfer_front;
		weight_transfer_rr=weight_transfer_rear-lateral_load_transfer_rear;
		weight_transfer_lr=weight_transfer_rear+lateral_load_transfer_rear;
    }
	else
    {
		weight_transfer_rf=weight_transfer_front+lateral_load_transfer_front;
		weight_transfer_lf=weight_transfer_front-lateral_load_transfer_front;
		weight_transfer_rr=weight_transfer_rear+lateral_load_transfer_rear;
		weight_transfer_lr=weight_transfer_rear-lateral_load_transfer_rear;
    }
	mass_of_car=weight_transfer_rf+weight_transfer_lf+weight_transfer_rr+weight_transfer_lr;
	if(throttle_position>0)
	{
	    s_peakr=21715;
	    t_peakr=44.21;
	    t_maxr_100=30;
	    throttle_position=throttle_position/100;
	    s_maxr=throttle_position*s_peakr;
	    t_maxr=throttle_position*t_peakr;
	    if(rear_left_motor_rpm<s_maxr)
	    {
	      torque_rear_left_motor=-(t_peakr/s_peakr)*front_left_motor_rpm+t_maxr;
	      if(torque_rear_left_motor>t_maxr_100)
	      {
		    torque_rear_left_motor=t_maxr_100;
	      }
	      else
	      {
		    torque_rear_left_motor=0;
	      }
	    }
	    if(rear_right_motor_rpm<s_maxr)
	    {
	        //t_maxr=throttle_position*t_peakr; Possibly incorrect in matlab program. Commented out in Matlab file.
		    torque_rear_right_motor=-(t_peakr/s_peakr)*rear_left_motor_rpm+t_maxr;
	        if(torque_rear_right_motor>t_maxr_100)
	        {
		        torque_rear_right_motor=t_maxr_100;
	        }
	        else
	        {
		        torque_rear_right_motor=0;
	        }
	    }
	    s_maxf=throttle_position*s_peakf;
	    t_maxf=throttle_position*t_peakf;
	    if(front_left_motor_rpm<s_maxf)
	    {
		    torque_front_left_motor=-(t_peakf/s_peakf)*front_left_motor_rpm+t_maxf;
	        if(torque_front_left_motor>t_maxf_100)
	        {
		        torque_front_left_motor=t_maxf_100;
	        }
	        else
	        {
		        torque_front_left_motor=0;
	        }
	    }
	    if(front_right_motor_rpm<s_maxf)
	    {
			torque_front_right_motor=-(t_peakf/s_peakf)*front_right_motor_rpm+t_maxf;
	        if(torque_front_right_motor>t_maxf_100)
	        {
		        torque_front_right_motor=t_maxf_100;
	        }
	        else
	        {
		        torque_front_right_motor=0;
	        }
	    }
	}
	if(regen_brake_position > 0)
	{
		//s_peakr=21715;
		//t_peakr=-44.21;
		//t_maxr_100=-30;
		regen_brake_position=regen_brake_position/100;
		s_maxr=regen_brake_position*s_peakr;
		t_maxr=regen_brake_position*(-t_peakr);
		if(rear_left_motor_rpm<s_maxr)
		{
			torque_rear_left_motor=-(-t_peakr/s_peakr)*rear_left_motor_rpm+t_maxr;
			if(torque_rear_left_motor<-t_maxr_100)
			{
				torque_rear_left_motor=-t_maxr_100;
			}
			else
			{
				torque_rear_left_motor=0;
			}
		}
		//t_max=regenerative_brake_position*t_peakr;Possibly incorrect. Commented out of matlab file.
		if(rear_right_motor_rpm<s_maxr)
		{
			torque_rear_right_motor=-(-t_peakr/s_peakr)*rear_right_motor_rpm+t_maxr;
			if(torque_rear_right_motor<-t_maxr_100)
			{
				torque_rear_right_motor=-t_maxr_100;
			}
			else
			{
				torque_rear_right_motor=0;
			}
		}
		s_peakf = 17674.91;
		t_peakf = -25.01;
		t_maxf_100 = -15;
		s_maxf=regen_brake_position*s_peakf;
		t_maxf=regen_brake_position*(-t_peakf);
		if(front_left_motor_rpm<s_maxf)
		{
			torque_front_left_motor=-(-t_peakf/s_peakf)*front_left_motor_rpm+t_maxf;
			if(torque_front_left_motor<-t_maxf_100)
			{
				torque_front_left_motor=-t_maxf_100;
			}
			else
			{
				torque_front_left_motor = 0;
			}
		}
		if(front_right_motor_rpm<s_maxf)
		{
			torque_front_right_motor=-(-t_peakf/s_peakf)*front_right_motor_rpm+t_maxf;
			if(torque_front_right_motor<-t_maxf_100)
			{
				torque_front_right_motor=-t_maxf_100;
			}
			else
			{
				torque_front_right_motor = 0;
			}
		}
	}
	if(torque_mode_selection == 1)
	{
		torque_distribution=torque_front_right_motor+torque_front_left_motor+torque_rear_right_motor+torque_rear_left_motor;
		torque_front_left_output=weight_transfer_lf/mass_of_car*torque_distribution;
	    torque_front_right_output=weight_transfer_rf/mass_of_car*torque_distribution;
	    torque_rear_right_output=weight_transfer_rr/mass_of_car*torque_distribution;
	    torque_rear_left_output=weight_transfer_lr/mass_of_car*torque_distribution;
	}
	if(torque_mode_selection == 2)
	{
    absolute_front_torque=torque_front_left_motor+torque_front_right_motor;
    absolute_rear_torque=torque_rear_left_motor+torque_rear_right_motor;
    torque_distribution=absolute_front_torque+absolute_rear_torque;
    torque_front_left_output=weight_transfer_lf/(2*mass_of_car)*absolute_front_torque;
	  torque_front_right_output=weight_transfer_rf/(2*mass_of_car)*absolute_front_torque;
	  torque_rear_right_output=weight_transfer_rr/(2*mass_of_car)*absolute_rear_torque;
	  torque_rear_left_output=weight_transfer_lr/(2*mass_of_car)*absolute_rear_torque;
	}
  if(throttle_position>0)
  {
    if(torque_front_left_output>t_maxf_100)
	  {
			  torque_front_left_output=t_maxf_100;
	  }
    if(torque_front_left_output<0)
    {
      if(torque_front_left_output< -t_maxf_100)
      {
        torque_front_left_output=-t_maxf_100;
      }
    }
	  if(torque_front_right_output>t_maxf_100)
	  {
			  torque_front_right_output=t_maxf_100;
	  }
    if(torque_front_right_output<0)
    {
      if(torque_front_right_output< -t_maxf_100)
      {
        torque_front_right_output=-t_maxf_100;
      }
    }
	  if(torque_rear_left_output>t_maxr_100)
	  {
			  torque_rear_left_output=t_maxr_100;
	  }
    if(torque_rear_left_output<0)
    {
      if(torque_rear_left_output< -t_maxr_100)
      {
        torque_rear_left_output=-t_maxr_100;
      }
    }
    if(torque_rear_right_output>t_maxr_100)
	  {
		  	torque_rear_right_output=t_maxr_100;
	  }
    if(torque_rear_right_output<0)
    {
        if(torque_rear_right_output< -t_maxr_100)
        {
          torque_rear_right_output=-t_maxr_100;
        }
    }
  }
  if(regen_brake_position > 0)
  {
    if(torque_front_left_output<t_maxf_100)
    {
      torque_front_left_output=t_maxf_100;
    }
    if(torque_front_left_output>0)
    {
      if(torque_front_left_output>-t_maxf_100)
      {
        torque_front_left_output=-t_maxf_100;
      }
    }
    if(torque_front_right_output<t_maxf_100)
    {
      torque_front_right_output=t_maxf_100;
    }
    if(torque_front_right_output>0)
    {
      if(torque_front_right_output>-t_maxf_100)
      {
        torque_front_right_output=-t_maxf_100;
      }
    }
    if(torque_rear_left_output<t_maxr_100)
    {
      torque_rear_left_output=t_maxr_100;
    }
    if(torque_rear_left_output>0)
    {
      if(torque_rear_left_output>-t_maxr_100)
      {
        torque_rear_left_output=-t_maxr_100;
      }
    }
    if(torque_rear_right_output<t_maxr_100)
    {
      torque_rear_right_output=t_maxr_100;
    }
    if(torque_rear_right_output>0)
    {
      if(torque_rear_right_output>-t_maxr_100)
      {
        torque_rear_right_output=-t_maxr_100;
      }
    }
  }
  torque_front_left_ratio=(torque_front_left_output-torque_front_left_motor)/torque_front_left_motor;
  torque_front_right_ratio=(torque_front_right_output-torque_front_right_motor)/torque_front_right_motor;
  torque_rear_left_ratio=(torque_rear_left_output-torque_rear_left_motor)/torque_rear_left_motor;
  torque_rear_right_ratio=(torque_rear_right_output-torque_rear_right_motor)/torque_rear_right_motor;
  if(torque_front_left_output==0)
  {
    torque_front_left_ratio=0;
  }
  if(torque_front_right_output==0)
  {
    torque_front_right_ratio=0;
  }
  if(torque_rear_left_output==0)
  {
    torque_rear_left_ratio=0;
  }
  if(torque_rear_right_output==0)
  {
    torque_rear_right_ratio=0;
  }
  lambda_tv_fl=(1+torque_front_left_ratio)*itp;
  lambda_tv_fr=(1+torque_front_right_ratio)*itp;
  lambda_tv_rl=(1+torque_rear_left_ratio)*itp;
  lambda_tv_rr=(1+torque_rear_right_ratio)*itp;
	printf("Front right= %lf\n",torque_front_right_output);
	printf("Front left= %lf\n",torque_front_left_output);
	printf("Rear right= %lf\n",torque_rear_right_output);
	printf("Rear left= %lf\n",torque_rear_left_output);
	while(1);
}