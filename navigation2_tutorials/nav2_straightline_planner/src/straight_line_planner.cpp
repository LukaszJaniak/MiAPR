/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include <random>
#include <iostream>
#include "nav2_util/node_utils.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}


double distance(double x1, double y1, double x2, double y2)
{
    // Calculating distance
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}


nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;


  // std::cout <<"costmap_========================================================" << std::endl;
  unsigned int cost_r = costmap_->getCost(4, 5);
  double resolution = costmap_->getResolution();// wartość po jakiej poruszamy się po mapie 
  unsigned int size_x = costmap_->getSizeInCellsX();//rozmiar mapy x
  unsigned int size_y = costmap_->getSizeInCellsY();//rozmiar mapy y
  std::cout <<"costmap_ ::::" << cost_r <<" end_costmap" <<size_y <<"resolution"<< resolution<< std::endl;



  //do losowania punktów
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_x(0, size_x/resolution); // define the range
  std::uniform_int_distribution<> distr_y(0, size_y/resolution); // define the range

  int amount_of_random_points = 100;
  double random_poins_tab[amount_of_random_points][2];
  

  // unsigned int costa = costmap_->getCost(distr_x(gen)*resolution,distr_y(gen)*resolution);
  // std::cout << costa<< ' '<<"next========================="<< distr_x(gen)*resolution<<distr_y(gen)*resolution; // generate numbers 
  int size_of_graph=0;
  
  for(int n=0; n <amount_of_random_points; ++n){//losowanie punktów
    double rand_x = distr_x(gen)*resolution;
    double rand_y = distr_y(gen)*resolution;

    unsigned int cost = costmap_->getCost(rand_x,rand_y);//wartość mapy w wylosowanym punkcie 

    double target[] = {rand_x, rand_y};
    int size_n = sizeof(random_poins_tab ) / sizeof(*random_poins_tab );
    bool exists = std::find(random_poins_tab, random_poins_tab + size_n, target) != random_poins_tab + size_n;//do sprawdzenia czy punkt się nie powtarza w tablicy wylosowanych punktów

    if( cost<255 && exists==false ) 
    {
      
      // std::cout <<typeid(rand_x).name()<< "  "<<"next=========================";
      random_poins_tab[size_of_graph][0] = rand_x;
      random_poins_tab[size_of_graph][1] = rand_y;
      size_of_graph=size_of_graph+1;
    }
  }

// //tworzenie grafu 
// // jescze nie gotowe !!!!!!!!!!!!!
  double radious = 90;// obszar w którym punkty zostaną połączone 
  int graph[sizeof(size_of_graph)][size_of_graph]={-1};//tablica przechowująca graf 
  int index_graph_1=0;
  // std::cout<<"size)graph "<<size_of_graph<<"gah";
  for(int i=0; i <size_of_graph; ++i){
    int index_graph_2=0;
    for(int j=0; j <size_of_graph; ++j){
      double distans=distance(random_poins_tab[i][0], random_poins_tab[i][1], random_poins_tab[j][0], random_poins_tab[j][1]);

    
      if(distans< radious){
        // std::cout <<"distans< radious)" << "  "<<"<<<<<<<<<<<<<";

        // std::cout <<random_poins_tab[i][0]<<"------"<<random_poins_tab[j][0] << "  "<<i<<" , "<<j<<" next=========================";


        if (random_poins_tab[i][0]!=random_poins_tab[j][0]){
                  // równanie lini
        //sprawdzenie czy między dwoma punktami nie ma przeszkody
          double a=((random_poins_tab[i][1]/resolution-random_poins_tab[j][1]/resolution)/(random_poins_tab[i][0]/resolution- random_poins_tab[j][0]/resolution));// wartośc do równania prostej między dowma 
          double b=(-a)*random_poins_tab[i][0]/resolution+random_poins_tab[i][1]/resolution;// wartośc do równania prostej między dwoma punktami
          double start = 0;
          double stop = 0;

          if(random_poins_tab[i][0]>random_poins_tab[j][0]){
            // std::cout<<"  ==ifffffffff1=====  ";
            start = random_poins_tab[j][0] /resolution;
            stop = random_poins_tab[i][0]/resolution;
          }
          else{
            // std::cout<<"  ==ifffffffff2=====  ";
            start = random_poins_tab[i][0] /resolution;
            stop = random_poins_tab[j][0]/resolution;
          }
          // std::cout <<start<<"------"<<stop << "  "<<i<<" , "<<j<<" next=========================";
          
          bool clear= true; 
          for(double index_x=start; index_x <stop ; ++index_x){// indeksowanie po prostej do prawdzenia czy nie przecina zajętego punktu
            double index_y=  round(a * index_x + b);      
            unsigned int cost_between= costmap_->getCost(index_x*resolution,index_y*resolution);
    
            if (cost_between>=255){
              clear=false;
              break;
            }

          }
          // std::cout <<"graph"<< clear<<"next=========================";
          if (clear==true){
            if (index_graph_2==0){
              graph[index_graph_1][index_graph_2]=i;
              index_graph_2=index_graph_2+1;  
            }
            
            std::cout <<"index1:: "<< index_graph_1<<" index_graph_2:: "<<index_graph_2;
            graph[index_graph_1][index_graph_2]=j;
            index_graph_2=index_graph_2+1;
          }

        }
      }
      // std::cout <<random_poins_tab[i][1]<< "  "<<"next=========================";
    }

    if (index_graph_2>0){
      index_graph_1=index_graph_1+1;
    }
    
  }





  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)