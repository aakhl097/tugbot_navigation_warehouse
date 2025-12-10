#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "math.h"
#include "string.h"
#include <iomanip>
// #include "unistd.h"
using namespace std;
using std::placeholders::_1;

#define PI 3.14

class Speed_Controller_Node : public rclcpp::Node
{
    public:
    Speed_Controller_Node() : Node("Driver_Node") 
    {
        position_listener = this->create_subscription<geometry_msgs::msg::Pose>("/my_tugbot_pose", rclcpp::SystemDefaultsQoS(), std::bind(&Speed_Controller_Node::pose_callback, this, _1));
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("/model/tugbot/cmd_vel", rclcpp::SystemDefaultsQoS());
        orientation_listener = this->create_subscription<geometry_msgs::msg::Pose>("/quaternion_parameters",rclcpp::SystemDefaultsQoS(),std::bind(&Speed_Controller_Node::quaternion_call, this, _1));
        lidar_listener = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar_scan", rclcpp::SystemDefaultsQoS(), std::bind(&Speed_Controller_Node::scan_call, this, _1));
        cout << endl << "Driver Node Activated!" << endl;
        timer = this->create_wall_timer(100ms, std::bind(&Speed_Controller_Node::timer_callback, this));
    }

    private:
    
    float current_position_x;
    float current_position_y;
    bool turn_right = false;
    bool turn_left_shelf = false;
    bool between_shelves = false;
    bool entering_shelves = false;
    bool exiting_first_shelf = false;
    bool adjustment_after_shelf_completed = false;
    bool heading_to_second_shelf = false;
    bool distance_adjustment = false;
    bool heading_to_third_shelf = false;
    bool drive_straight = false;
    bool second_adjustment = false;
    bool entering_third_shelf = false;
    bool final_adjusment = false;
    bool heading_to_skid = false;
    bool approaching_destination_zone = false;
    bool reached_destination_zone = false;
    bool trip_over = false;
    bool object_ahead = false;

    float term_first_inside;
    float term_first_outside;
    float term_second_inside;
    float term_second_outside;
    float euler_yaw;

    float angle_minimum;
    float angle_maximum;
    float increment;
    float distances;
    int i = 0;
    int size, actual_size;

    float distances_within_threshold_right[670];
    float distances_within_threshold_left[670];
    int j=0;
    int k=0;
    int countdown=10;

    float lidar_angle[670];
    bool right_alert = false;
    bool left_alert = false;
    bool consider_lidar_data = false; //set to true for debugging purposes but not in final simulation.
    char alert_message[25];

    void timer_callback()
    {
        auto desired_speed = geometry_msgs::msg::Twist();

        if (current_position_x >= 10.50 && current_position_x <= 10.77 && current_position_y >= -8.08 && current_position_y <= -7.015) //x lower was 10.50
        { 
            turn_right = true;
        }
        
        if (turn_right == false && turn_left_shelf == false && consider_lidar_data == false && exiting_first_shelf == false)
        {
            
            if (entering_shelves == false)
            {
                desired_speed.linear.x = -1.7; //-0.7
                desired_speed.angular.z = -0.7; //-0.2
            }

            else
            {
                desired_speed.linear.x = 1.0;
                desired_speed.angular.z = 0.0;
                cout << "Activating Lidar in " << countdown << endl;
                countdown = countdown - 0.01;
                if (countdown <= 0)
                {
                    consider_lidar_data = true;
                    entering_shelves = false;
                }
            }
        
        }
        
        if (turn_right == true && entering_shelves == false)
        {
            desired_speed.linear.x = -1.5; //-0.7
            desired_speed.angular.z = 0.29; //0.09
        }

        if (current_position_x >= 2.83 && current_position_x <= 3.40 && current_position_y >= -3.1 && current_position_y <= -2.19) //y lowr was -2.24; x lower was 2.6
        {
            if (countdown == 10)
            {
            turn_left_shelf = true;
            turn_right = false;
            }
        }

        if (turn_left_shelf == true)
        {
            if ((euler_yaw >= -60) && (euler_yaw <= -50))
            {
                desired_speed.linear.x = 1.0;
                desired_speed.angular.z = 0.0;
                publisher->publish(desired_speed);
                countdown --; 
                between_shelves = true;
                turn_left_shelf = false;
                entering_shelves = true;
            } 
            else 
            {
                if (between_shelves == false)
                {
                    desired_speed.linear.x = -0.3;
                    desired_speed.angular.z = -0.3;
                    cout << euler_yaw << endl;
                }
            }
        }

        if (exiting_first_shelf == true)
        {
            consider_lidar_data = false;
               
            if (euler_yaw >=-170 && euler_yaw <= -168)
            {
                //exiting_first_shelf = false;
                adjustment_after_shelf_completed = true;
                desired_speed.linear.x = 0.7;
                desired_speed.angular.z = 0.0;
                publisher->publish(desired_speed);
            }
            else 
            {
                if (adjustment_after_shelf_completed == false)
                {  
                desired_speed.linear.x = -0.0;
                desired_speed.angular.z = -0.3;
                publisher->publish(desired_speed);
                }
            }
            
        }

        if (adjustment_after_shelf_completed == true)
        {
            if (heading_to_second_shelf == false)
            {
                desired_speed.linear.x = 2.0;
                desired_speed.angular.z = 0.0;
                publisher -> publish(desired_speed);

                if (current_position_x >= -13.0 && current_position_x <= -12.0)
                {
                    heading_to_second_shelf = true;
                }
            }
            else 
            {
                desired_speed.linear.x = 0.0;
                desired_speed.angular.z = -0.3;    
                if (euler_yaw >= 85 && euler_yaw <= 90)
                {
                    desired_speed.linear.x = 2.0;
                    desired_speed.angular.z = 0.0;  
                    
                    consider_lidar_data = true;
                    adjustment_after_shelf_completed = false;
                    distance_adjustment = true;
                    exiting_first_shelf = false;
                } 
                publisher -> publish(desired_speed);
            }
                
        }

        if (heading_to_third_shelf == true)
        {
            consider_lidar_data = false;
            desired_speed.linear.x = 0.0;
            desired_speed.angular.z = 0.3;
            publisher->publish(desired_speed);
            if (euler_yaw <= 83 && euler_yaw >= 80)
            {
                drive_straight = true;
                heading_to_third_shelf = false;
            }
        }

        if (drive_straight==true)
        {
            consider_lidar_data = false;
            desired_speed.linear.x = 1.5;
            desired_speed.angular.z = 0.0;
            publisher->publish(desired_speed);
            if (current_position_x > -14.5 && current_position_x < -8.7 && current_position_y > 6.18 && current_position_y < 6.34)
            {
                second_adjustment = true;
                drive_straight = false;
            }
        }

        if (second_adjustment == true)
        {
            consider_lidar_data = false;
            distance_adjustment = false;
            desired_speed.linear.x = 0.0;
            desired_speed.angular.z = -0.3;
            publisher->publish(desired_speed);
            if (euler_yaw >= 7.5 && euler_yaw <= 9)
            {
                entering_third_shelf = true;
                second_adjustment = false;
            }
        }

        if (entering_third_shelf == true)
        {
            desired_speed.linear.x = 2.0;
            desired_speed.angular.z = 0.0;
            publisher->publish(desired_speed);
            final_adjusment = true;
            consider_lidar_data = true;
        }

        if (object_ahead == true)
        {
            entering_third_shelf = false;
            consider_lidar_data = false;
            desired_speed.angular.z = 0.30;
            desired_speed.linear.x = 0.0;
            publisher->publish(desired_speed);
            if (euler_yaw <= 83 && euler_yaw >= 80)
            {
                object_ahead = false;
                approaching_destination_zone = true;
            }

        }

        if (trip_over == true)
        {
            desired_speed.angular.z = 0.0;
            desired_speed.linear.x = 0.0;
            publisher->publish(desired_speed);
            reached_destination_zone = false;
        }

        if (approaching_destination_zone == true)
        {
            desired_speed.angular.z = 0.0;
            desired_speed.linear.x = 2.0;
            publisher->publish(desired_speed);
            if (current_position_x <= 14.0 && current_position_x >= 8.9 && current_position_y <= 13.8 && current_position_y >= 12.6)
            {
                approaching_destination_zone = false;
                heading_to_skid = true;
            }
        } 

        if (heading_to_skid == true)
        {
            desired_speed.angular.z = 0.26;
            desired_speed.linear.x = 0.0;
            publisher->publish(desired_speed);
            if (euler_yaw >=172 && euler_yaw <= 175)
            {
                heading_to_skid = false;
                reached_destination_zone = true;
            }
        }

        if (reached_destination_zone == true)
        {
            desired_speed.angular.z = 0.0;
            desired_speed.linear.x = 2.0;
            publisher->publish(desired_speed);
            consider_lidar_data = true;
        }
        
        if (consider_lidar_data == false)
        {
            publisher->publish(desired_speed);
        }

    }
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_position_x = msg->position.x;
        current_position_y = msg->position.y;
        if (trip_over == false)
        {
            cout << "X: " << current_position_x << fixed << setprecision(2) << " m" << "    Y: " << current_position_y << fixed << setprecision(2) << " m" << "    Yaw: " << euler_yaw << fixed << setprecision(2) << " deg"<< "    LiDAR Alert: " << alert_message << endl;
            cout << endl;
        }
        else 
        {
            cout << "Tugbot has reached its destination!" << endl;
        }
    }

    void quaternion_call(const geometry_msgs::msg::Pose::SharedPtr xyzw_parameters)
    {
        term_first_inside = (xyzw_parameters->orientation.w * xyzw_parameters->orientation.z) + (xyzw_parameters->orientation.x * xyzw_parameters->orientation.y);
        term_first_outside = 2 * term_first_inside;

        term_second_inside = pow(xyzw_parameters->orientation.y,2) + pow(xyzw_parameters->orientation.z,2);
        term_second_outside = 1 - (2 * term_second_inside);

        euler_yaw = (atan2(term_first_outside, term_second_outside)) * (180 / PI);
    }

    void scan_call(const sensor_msgs::msg::LaserScan::SharedPtr scan_message)
    {
        angle_minimum = scan_message->angle_min;
        angle_maximum = scan_message->angle_max;
        increment = scan_message->angle_increment;
        std::vector<float> distances = scan_message->ranges; 
        auto angular_speed_lidar = geometry_msgs::msg::Twist();
        j=0;
        k=0;

        if (consider_lidar_data == true)
        
        {
            for (i=0; i<650; i++) //assume i to be only 120 and 360
            {
                lidar_angle[i] = (i*increment) + angle_minimum;         
            
            if (lidar_angle[i] < 0)
                {
                    if (distance_adjustment == false && final_adjusment == false)
                    {
                        if (distances[250] < 2.0 && distances[250] > 0.1) //220, 
                        {
                            right_alert = true;
                            //distances_within_threshold_right[j] = distances[i];
                            angular_speed_lidar.angular.z = 0.40;
                            angular_speed_lidar.linear.x = 0.9;
                            strcpy(alert_message, "Obstacle on Right Side!");
                            //cout << "---------------|-----------------
                            // 
                            // -------|---------------------|" << endl;
                            publisher->publish(angular_speed_lidar);
                            
                        }
                        else 
                        {
                        distances_within_threshold_right[j] = 0.0;
                        }
                    }

                    else if (distance_adjustment == false && final_adjusment == true)
                    {
                        //if (current_position_x >= 7.45 && current_position_x <= 10.9 && current_position_y >= 6.0 && current_position_y <= 6.8)
                        //{
                            if (distances[315] < 5.0 && distances[315] > 0.1) 
                            {
                                entering_third_shelf = false;
                                strcpy(alert_message, "Object Straight Ahead!");
                                object_ahead = true;
                                if (reached_destination_zone == true && distances[315] < 2.5)
                                {
                                    trip_over = true;
                                }
                            }
                        //}
                    }

                    else //distance_adjustment = true && final_adjustment = false
                    {
                        if (distances[80] < 2.0 && distances[80] > 0.1) //220, 
                            {
                            right_alert = true;
                            //distances_within_threshold_right[j] = distances[i];
                            angular_speed_lidar.angular.z = 0.60;
                            angular_speed_lidar.linear.x = 0.7;
                            publisher->publish(angular_speed_lidar);
                            strcpy(alert_message, "Obstacle on Right Side!");
                            }
                        else 
                            {
                        distances_within_threshold_right[j] = 0.0;
                            }

                    }
                    
                //j=j+1;
                }

                else if (lidar_angle[i] >= 0)
                {
                    if (distance_adjustment == false)

                        if (distances[450] < 1.6 && distances[450] > 0.1)
                        {
                            left_alert = true;
                            //distances_within_threshold_left[k] = distances[i];
                            angular_speed_lidar.linear.x = 0.8;
                            angular_speed_lidar.angular.z = -0.6;
                            //cout << distances[450] << endl;
                            strcpy(alert_message, "Obstacle on Left Side!");
                            //cout << "---------------|------------------------|---------------------|" << endl;
                            publisher->publish(angular_speed_lidar);
                            
                        }
                        else 
                        {
                            distances_within_threshold_left[k] = 0.0;
                        }
                        //k=k+1;

                    else if (distance_adjustment == false && final_adjusment == true)
                    {
                            if (distances[345] < 5.0 && distances[345] > 0.1) 
                            {
                                entering_third_shelf = false;
                                strcpy(alert_message, "Object Straight Ahead!");
                                object_ahead = true;
                                if (reached_destination_zone == true && distances[345] < 2.5)
                                {
                                    trip_over = true;
                                }
                            }
                    }
                    
                    else 
                    {
                        if (distances[400] < 2.7 && distances[400] > 0.1) //Widge margin between wall and shelf. Increasing the distance threshold helped. 
                        {
                            left_alert = true;
                            angular_speed_lidar.linear.x = 0.8;
                            angular_speed_lidar.angular.z = -0.5;
                            publisher->publish(angular_speed_lidar);
                            strcpy(alert_message, "Obstacle on Left Side!");
                        }
                        else 
                        {
                            distances_within_threshold_left[k] = 0.0;
                        }
                    }

                }

                if (current_position_x <= 4.0 && current_position_x >= 1.8 && current_position_y <= -22.9 && current_position_y >= -24.3) //y greater was -23.1. y lower was -24.0; x low was 4.0
                {
                    consider_lidar_data = false;
                    exiting_first_shelf = true;
                    // 1.5 < x < 4.5        -23.8 < y < -22.7  
                }

                if (current_position_y >= -3.6 && current_position_x >= -14.5 && current_position_x <= -10.9)
                {
                    consider_lidar_data = false;
                    heading_to_third_shelf = true;
                }
            }
        //actual_size = sizeof(lidar_angle)/sizeof(lidar_angle[0]);
        consider_lidar_data = true;
        }
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher; 
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr position_listener;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr orientation_listener;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_listener;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Speed_Controller_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
