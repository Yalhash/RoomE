#include "DriveTrain.h"
#include <iostream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>


DriveTrain::DriveTrain() {
    serial.usb_open();
}

DriveTrain::~DriveTrain() {
    serial.usb_close();
}



namespace { 
// function to calculate distance and angle 
    std::vector<double> get_turn_angle(double xi,double yi, double curr_angle, double xf, double yf){

            std::vector<double> vec_str;    

            //Goal point is current location
            if(xi == xf && yi == yf){
                vec_str.push_back(0);
                vec_str.push_back(0);
                return vec_str;
            }


            double delta_x= math.std::abs(xf-xi); 
            double delta_y= math.std::abs(yf-yi); 

            //Distance the roome will physically move
            double dist= sqrt(delta_x*delta_x+delta_y*delta_y);

            //Angle we want to face towards to move
            double goal_angle = acos(delta_x/dist)* (180/M_PI);

            if (xi>xf){
                goal_angle = 180 - goal_angle;
            }

            if (yi > yf){
                goal_angle = -goal_angle;
            }

            

            //How much to turn from current angle to get to the correct angle
            double turn_angle = goal_angle - (curr_angle * (180/M_PI));

            //Ensure we take the shortest turn direction
            if(turn_angle > 180){
                 turn_angle= -(360-turn_angle);
            }       

            vec_str.push_back(dist);
            vec_str.push_back(turn_angle);
            return vec_str;
    }


    // function to parse string to vector 

    std::vector<float>  parse_str_between_two_markers(const std::string &s,
            const std::string &start_delim,const std::string &stop_delim)    
    {
        unsigned first_delim_pos = s.find(start_delim);                    
        unsigned end_pos_of_first_delim = first_delim_pos + start_delim.length();
        unsigned last_delim_pos = s.find(stop_delim);
     
        std::string line =  s.substr(end_pos_of_first_delim,
                last_delim_pos - end_pos_of_first_delim);
                
        std::vector<float> vect;
        int i = 0;
        std::stringstream ss(line);
        for (int i; ss >> i;) {
            vect.push_back(i);    
            if (ss.peek() == ',')
                ss.ignore();
        }        
           return vect;     
    }
}

mrpt::poses::CPose2D DriveTrain::move(mrpt::poses::CPose2D start, mrpt::math::TPoint2D finish) {

    auto coords = start.m_coords;
    std::vector travel_data =  get_turn_angle(coords[0],coords[1],start.phi(), finish.x, finish.y);  

    std::cout << "current pose: " << start << std::endl;
    std::cout << "travel data: " << travel_data[0] << ", " << travel_data[1] << std::endl;

    std::string data_send_arduino_turning ;
    std::string data_send_arduino_stright ;
    std::cout << "SPONGE0" << std::endl;
   
    // combine data for transfer  
    if (travel_data[1]>0){    //right trun
        data_send_arduino_turning = "<1,0,1,0,0,"+ std::to_string(travel_data[1]) + ",1>" ;   
    }
    else {                     //left turn
        data_send_arduino_turning = "<1,0,0,0,0," +  std::to_string(-travel_data[1]) + ",1>" ; 
    }

    data_send_arduino_stright =  "<2,0,1,0," + std::to_string(travel_data[0]) + ",0,1>" ;  // going forward

    std::cout << "trying to send?: " << data_send_arduino_turning << std::endl;




    serial.write_string(data_send_arduino_turning);

    auto Turn_odm_str = serial.read_string();
    std::cout << "Turn received: " << Turn_odm_str << std::endl;

    std::cout << "SPONGE4" << std::endl;
        std::cout << "now send: " << data_send_arduino_stright << std::endl;
    serial.write_string(data_send_arduino_stright);
    std::cout << "SPONGE5" << std::endl;
    
    auto drive_odm_str = serial.read_string();
    printf("drive odm: '%s'\n", drive_odm_str.c_str());

    std::vector<float> turn_odm_vector = parse_str_between_two_markers(Turn_odm_str, "<",">");
    std::vector<float> drive_odm_vector = parse_str_between_two_markers(drive_odm_str, "<",">");



    // angle changed due to turning 
    float angle_changed;
    if (travel_data[1]>0){  // right turn
     angle_changed = start.phi()- ((turn_odm_vector[0] ) *(360/ 20));
     }
    else{   // left turn 
     angle_changed = start.phi() - ((turn_odm_vector[1] ) *(360/ 20));   
    }

    
    std::cout << "SPONGE6" << std::endl;
   float distance_travel_left_wheel =  (drive_odm_vector[0]/20)* 0.3798318779;
   float distance_travel_right_wheel =  (drive_odm_vector[1]/20)* 0.3798318779;
   float res_distance_wheel =  distance_travel_right_wheel - distance_travel_left_wheel ;
   float angle_change_drive =   res_distance_wheel/ 0.25;
   float total_distance_travel =  (distance_travel_left_wheel + distance_travel_right_wheel)/2;
   float final_angle;
   
   if(angle_change_drive<0){
        final_angle = angle_changed + angle_change_drive;  // right turn 
   }
    else{
         final_angle = angle_changed + angle_change_drive; // left turn 
    }

    float y_new,x_new ;
    if(final_angle <90){
        y_new= sin(final_angle)* total_distance_travel + coords[1];
        x_new = cos(final_angle)*total_distance_travel + coords[0];
    }
    else if(final_angle >90 && final_angle <180){
        y_new= sin(180-final_angle)* total_distance_travel + coords[1];
        x_new = -cos(180-final_angle)* total_distance_travel + coords[0];
    }
    else if(final_angle >180 && final_angle <270){
        y_new= -sin(final_angle-180) * total_distance_travel + coords[1];
        x_new = -cos(final_angle-180)* total_distance_travel+ coords[0];

    }
    else{
        y_new= -sin(360-final_angle) * total_distance_travel+ coords[1];
        x_new = cos(360-final_angle) *total_distance_travel + coords[0];
    }
 
 
    std::cout << "SPONGE7" << std::endl;
    // TODO move RoomE and fill left wheel and right wheel with the l/r odometry info

    return mrpt::poses::CPose2D(x_new, y_new, final_angle);
}
