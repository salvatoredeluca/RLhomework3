// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp" //in order to subscribe to the aruco topic
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;



class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "velocity"); // defaults to velocity
            declare_parameter("task","positioning"); //default to positioning
            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("task",task_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
            RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());

            if (!(cmd_interface_ == "velocity" || cmd_interface_=="effort"))
            { 
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }
            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_INFO(get_logger(),"Selected task  is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 
            aruco_available_= false;
            flag_ = true;
           

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  

            //Initialize controller
            controller_=KDLController(*robot_);
        
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_efforts_.resize(nj);
            des_joint_positions_.resize(nj);
            des_joint_velocities_.resize(nj);
            des_joint_accelerations_.resize(nj);
            joint_initial_positions_.resize(nj);
            old_joint_velocities_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Subscriber to aruco_single topic
            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_subscriber, this, std::placeholders::_1));
          

            //Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            joint_initial_positions_=joint_positions_; //retrieve initial position of the joints

            
            
            if (cmd_interface_=="velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));              
            }
            else if (cmd_interface_=="effort")
            {
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));           
            }
               

    }

  
    private:

       
        void cmd_publisher(){
            
            iteration_ = iteration_ + 1;          
            double total_time = 3; //
            double traj_duration = 3; 
            int trajectory_len = 150; // 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate; 
            t_+=dt;

            aruco_pos_offset_[0]=0;
            aruco_pos_offset_[1]=0;
            aruco_pos_offset_[2]=0.8;
            //Morale della storia: ricordarsi di fare la trasformazione intermedia e fidarsi di Genny
            KDL::Frame fromLinkToOpticalLink=KDL::Frame::Identity();
            fromLinkToOpticalLink.M=KDL::Rotation::Quaternion(-0.5,0.5,-0.5,0.5);
            KDL::Frame fromEEtoLink=KDL::Frame::Identity();
            fromEEtoLink.M=KDL::Rotation::Quaternion(0.7,0,0.7,0);              
            KDL::Frame T_offset(KDL::Rotation::RotX(-3.14)*KDL::Rotation::RotZ(3.14), toKDL(aruco_pos_offset_));
             
            if(task_=="positioning")
            {
                
                 
                //CREATE PLANNER
                if(flag_ && aruco_available_)
                {
                    
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));                   
                    init_cart_pose_=robot_->getEEFrame();  

                                               
                    desFramefinale_=init_cart_pose_*fromEEtoLink*fromLinkToOpticalLink*arucoFrame_*T_offset; 

                               
                    Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));    
                    Eigen::Vector3d end_position(Eigen::Vector3d(desFramefinale_.p.data)); 
                                   
                    
                    planner_ = KDLPlanner(traj_duration, init_position, end_position);
                    flag_=false;
                }
                // std::cout<<"DES FRAME"<<std::endl;
                // printKDLFrame(desFramefinale_); 
                
                
                       
                if ( t_ < total_time && !flag_){
            
                    
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                    trajectory_point p = planner_.compute_trajectory(t_, "linear_trajectory");

                    // Compute EE frame
                    KDL::Frame cartpos = robot_->getEEFrame();           
                    // compute errors
                    Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                    Eigen::Vector3d o_error = computeOrientationError(toEigen(desFramefinale_.M), toEigen(cartpos.M));
                    std::cout << "The error position norm is : " << error.norm() << std::endl;
                    std::cout << "The error orientation norm is : " << o_error.norm() << std::endl;

                    //select the command interface and compute the corresponding law (position/velocity/effort)
                    if (cmd_interface_=="velocity"){
                        
                        //std::cout<<"EXECUTING POSITIONING TASK";
                        Vector6d cartvel; cartvel <<p.vel+ 5*error, 2*o_error;
                            
                        Eigen::MatrixXd Jcross=pseudoinverse(robot_->getEEJacobian().data);
                        Eigen::MatrixXd J=robot_->getEEJacobian().data;
                            
                        joint_velocities_.data = Jcross*cartvel;
                    }                 
                     //assign the commands 
                    if(cmd_interface_=="velocity"){
                            // Send joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = joint_velocities_(i);                     
                        }
                    }


                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);

                    

                }

                else if(t_>traj_duration)
                { 
                    if(cmd_interface_=="velocity")
                    {         
                        RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i){
                                    desired_commands_[i] = 0;
                        }

                        
                        std_msgs::msg::Float64MultiArray cmd_msg;
                        cmd_msg.data = desired_commands_;
                        cmdPublisher_->publish(cmd_msg);   
                    }               
               
                }    

            }  
            else if (task_=="look-at-point" && aruco_available_)
            {
                    
                    //std::cout<<"STO ESEGUENDO"<<std::endl;                
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));  
                    KDL::Frame Fcamera=KDL::Frame::Identity();
                    KDL::Frame cartpos = robot_->getEEFrame(); 
                    Fcamera=cartpos*fromEEtoLink*fromLinkToOpticalLink;
                    Eigen::Matrix<double,6,6> R=Eigen::Matrix<double, 6, 6>::Zero();

                    R.block(0,0,3,3)= toEigen(Fcamera.M);
                    R.block(3,3,3,3)= toEigen(Fcamera.M);                        

                    
                    Eigen::MatrixXd J=robot_->getEEJacobian().data;
                    Eigen::MatrixXd Jcross=pseudoinverse(J);
                    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
                    Eigen::MatrixXd I7 = Eigen::MatrixXd::Identity(7, 7);
                    Eigen::Vector3d s=toEigen((arucoFrame_.p)).normalized();                
                    Eigen::Vector3d sd;
                    sd << 0,0,1;

                    Eigen::Matrix<double,3,3> L1=-(1/(toEigen((arucoFrame_.p)).norm()))*(I3-(s*s.transpose()));
                    Eigen::Matrix<double,3,3> L2=skew(s);
                    Eigen::Matrix<double,3,6> L=Eigen::Matrix<double, 3, 6>::Zero();
                    L << L1,L2;
                    
                    
                    L=L*R.transpose();
                    Eigen::MatrixXd LJ=L*J;
                    Eigen::MatrixXd LJcross=pseudoinverse(LJ);
                    des_joint_velocities_.data=2*LJcross*sd+3*(Eigen::Matrix<double,7,7>::Identity() - LJcross*LJ)*
                                                            (joint_initial_positions_.data-joint_positions_.data);


                    if(cmd_interface_=="velocity"){
                        // Send joint velocity commands
                        for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                            desired_commands_[i] = des_joint_velocities_(i);                     
                        }
                    }

                   

                    // Create msg and publish
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);
                                        
                
        }

       
          
} 
     
       
    
            
        

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
               
            
            }
        }


        void aruco_subscriber(const geometry_msgs::msg::PoseStamped& aruco_msg)
        {
            
            aruco_available_=true;
            Eigen::Vector3d v; 
            v << aruco_msg.pose.position.x,aruco_msg.pose.position.y,aruco_msg.pose.position.z;
            arucoFrame_.p=toKDL(v);
            arucoFrame_.M= KDL::Rotation::Quaternion(aruco_msg.pose.orientation.x,aruco_msg.pose.orientation.y,
                                                      aruco_msg.pose.orientation.z,aruco_msg.pose.orientation.w);

            //std::cout<<arucoFrame_;

        }


        void printKDLFrame(const KDL::Frame& frame) {
            const KDL::Vector& t = frame.p;    // Posizione
            const KDL::Rotation& r = frame.M; // Rotazione

            
            std::cout << "[ " << r(0, 0) << " " << r(0, 1) << " " << r(0, 2) << " " << t.x() << " ]\n";
            std::cout << "[ " << r(1, 0) << " " << r(1, 1) << " " << r(1, 2) << " " << t.y() << " ]\n";
            std::cout << "[ " << r(2, 0) << " " << r(2, 1) << " " << r(2, 2) << " " << t.z() << " ]\n";
            std::cout << "[ 0   0   0   1 ]\n";
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;

        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        KDL::Frame arucoFrame_;

        bool flag_;

        
        KDL::JntArray joint_initial_positions_;
        
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_efforts_;

        KDL::JntArray des_joint_positions_;
        KDL::JntArray des_joint_velocities_;
        KDL::JntArray des_joint_accelerations_;
        KDL::JntArray old_joint_velocities_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        KDLController controller_;
     
        KDL::Frame desFramefinale_;


        Eigen::Vector3d aruco_pos_offset_;

        int iteration_;
        bool joint_state_available_;
        bool aruco_available_;

        double t_;
        std::string cmd_interface_;
        std::string task_;
        KDL::Frame init_cart_pose_;

        double alpha_;
        double k_;
        double N_;


};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}