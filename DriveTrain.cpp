#include "DriveTrain.h"
#include <iostream>
#include <unistd.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>

DriveTrain::DriveTrain() {
    serial.usb_open();
    movement_calculated = false;
}

DriveTrain::~DriveTrain() {
    serial.usb_close();
}



namespace { 
// function to calculate distance and angle  to move from (xi,yi) to (xf,yf)
    std::vector<double> get_turn_angle(double xi,double yi, double curr_angle, double xf, double yf){

            std::vector<double> vec_str;    

            //Goal point is current location
            if(xi == xf && yi == yf){
                vec_str.push_back(0);
                vec_str.push_back(0);
                return vec_str;
            }


            double delta_x= abs(xf-xi); 
            double delta_y= abs(yf-yi); 

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
            }else if(turn_angle < -180){
                 turn_angle = 360 + turn_angle;
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

//calculates the required turn and drive vectors and then turns. Call post_scan_drive() afterwards to move the calculated amount
mrpt::poses::CPose2D DriveTrain::calculate_and_turn(mrpt::poses::CPose2D start, mrpt::math::TPoint2D finish) {

    auto coords = start.m_coords;
    std::vector travel_data =  get_turn_angle(coords[0],coords[1],start.phi(), finish.x, finish.y);  

    std::cout << "current pose: " << start << std::endl;
    std::cout << "travel data: " << travel_data[0] << ", " << travel_data[1] << std::endl;

    std::string data_send_arduino_turning ;
   
    //Arduino follows format <State,forwards/backwards,left/right,emergencystop,distance,angle,piconfirmation>
    //State can be 0 for standby, 1 for turn, and 2 for drive straight
    // combine data for transfer  
    if (travel_data[1]>0){    //left turn
        data_send_arduino_turning = "<1,0,0,0,0,"+ std::to_string(travel_data[1]) + ",1>" ;   
    }
    else {                     //right turn
        data_send_arduino_turning = "<1,0,1,0,0," +  std::to_string(-travel_data[1]) + ",1>" ; 
    }

    calculated_straight_movement = "<2,0,1,0," + std::to_string(travel_data[0]) + ",0,1>" ; 

    std::cout << "trying to send: " << data_send_arduino_turning << std::endl;




    serial.write_string(data_send_arduino_turning);
    auto Turn_odm_str = serial.read_string();
    std::cout << "Turn received: " << Turn_odm_str << std::endl;

    //sleep(2); // Give enough time to come to a stop after moving

    


    std::vector<float> turn_odm_vector = parse_str_between_two_markers(Turn_odm_str, "<",">");






    double left_encoder_ticks = turn_odm_vector[0];
    double right_encoder_ticks = turn_odm_vector[1];
    double distance_travel_left_wheel =  (left_encoder_ticks/20)* 0.3798318779;
    double distance_travel_right_wheel =  (right_encoder_ticks/20)* 0.3798318779;

    
    double phi = (distance_travel_right_wheel - distance_travel_left_wheel )/ 0.25;


    double total_distance_travel = (distance_travel_left_wheel + distance_travel_right_wheel)*0.5;

    double delta_x,delta_y;
    if (phi != 0)
    {
        const double R = phi/total_distance_travel;
        delta_x = R*sin(phi);
        delta_y = R*(1 - cos(phi));
    }
    else
    {
        delta_x = total_distance_travel;
        delta_y = 0;
    }

    movement_calculated = true;

    return mrpt::poses::CPose2D(delta_x, delta_y, phi); 
}


// Moves the roome straight by the previously calculated amount. ONLY CALL AFTER calculate_and_turn HAS BEEN RUN
mrpt::poses::CPose2D DriveTrain::post_scan_drive(){
    if (!movement_calculated){
        return mrpt::poses::CPose2D(0,0,0);
    }
    std::cout << "send: " << calculated_straight_movement << std::endl;
    serial.write_string(calculated_straight_movement);


    auto drive_odm_str = serial.read_string();
    printf("drive odm: '%s'\n", drive_odm_str.c_str());
    std::vector<float> drive_odm_vector = parse_str_between_two_markers(drive_odm_str, "<",">");

    double left_encoder_ticks = drive_odm_vector[0];
    double right_encoder_ticks = drive_odm_vector[1];
    double distance_travel_left_wheel =  (left_encoder_ticks/20)* 0.3798318779;
    double distance_travel_right_wheel =  (right_encoder_ticks/20)* 0.3798318779;

    
    double phi = (distance_travel_right_wheel - distance_travel_left_wheel )/ 0.25;


    double total_distance_travel = (distance_travel_left_wheel + distance_travel_right_wheel)*0.5;

    double delta_x,delta_y;
    if (phi != 0)
    {
        const double R = phi/total_distance_travel;
        delta_x = R*sin(phi);
        delta_y = R*(1 - cos(phi));
    }
    else
    {
        delta_x = total_distance_travel;
        delta_y = 0;
    }


    movement_calculated = false;

    return mrpt::poses::CPose2D(delta_x, delta_y, phi); 
}
