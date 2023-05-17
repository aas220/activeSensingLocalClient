//local client
// Created by tipakorng on 3/3/16.
//

#include <visualization_msgs/MarkerArray.h>
#include "simulator.h"
#include "active_sensing_continuous_local/ObsrvBack.h"
#include "active_sensing_continuous_local/ReqObsrv.h"
#include "active_sensing_continuous_local/UpdateInfo.h"
#include "active_sensing_continuous_local/CT.h"
#include "active_sensing_continuous_local/action_message.h"
#include "active_sensing_continuous_local/action_message2.h"
#include "rng.h"
#include<iostream>
#include<fstream>
#include<thread>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>


int allowOb = 0;
using namespace std;
unsigned int sensing_action_global;
double observation_global;
int req_obversation_flag = 0;
int update_flag = 0;
int total_trips = 0;

int server_communication_count = 0;
int server_n2 = 0;

double update_location_x;
double update_location_y;
double update_location_z;
int communication_count = 0;
int sensing_action_local;
int ncount = 0;
int source = 0;
unsigned int round_;
unsigned int sensing_action;
Eigen::VectorXd observation(1);
Eigen::VectorXd task_action(3);
std::vector<double> all_rewards;
std::vector<int> reward_source;
int Break_Point = 0;
int Observation_point = 0;
int timeout = 0;
double taskaction_time = 0;
double time_to_recovery = 0;
double rtt_time = 0;
double rtt_time_average = 0;
bool run = false;
enum serverResponses { observationSig = 2, keepAliveSig = 20, continueSig = 4, endSleep  };
enum statuses { standard =0, localRecovery = 1, keepAlive = 2, wake =3, localRecoveryStart = 4};
enum sources { local = 0, cloud = 1 };
statuses status = standard;
fstream file;
std::time_t raw_time;
struct tm* time_info;
char buffer[90];
std::chrono::high_resolution_clock::time_point last_message;


void handleTimeOut() {
    if (status == standard && communication_count == 0) {
        cout << "In standard state";
        status = localRecovery;
        system("./serverRecovery.sh &>/dev/null");
        ros::Duration(1).sleep();
    }

    if (status == standard && communication_count != 0) {
        cout << "In standard state";
        status = localRecoveryStart;
        // system("./serverRecovery.sh &>/dev/null");
        system("./serverRecovery.sh");
        ros::Duration(1).sleep();
    }

    else if (status == keepAlive) {
        cout << "State is keep alive";
        ROS_INFO("State is keep alive");

        status = wake;
    }
}


Simulator::Simulator(Model &model, BeliefSpacePlanner &planner, unsigned int sensing_interval) :
        model_(model),
        planner_(planner),
        sensing_interval_(sensing_interval)
{
    has_publisher_ = false;
}

Simulator::Simulator(Model &model, BeliefSpacePlanner &planner, ros::NodeHandle *node_handle,
                     unsigned int sensing_interval) :
        model_(model),
        planner_(planner),
        sensing_interval_(sensing_interval),
        node_handle_(node_handle)
{
    publisher_ = node_handle_->advertise<visualization_msgs::Marker>("simulator", 1);
    has_publisher_ = true;
}

Simulator::~Simulator()
{}

void Simulator::initSimulator()
{
    // Clear stuff
   // status = standard;
    taskaction_time = 0;
    states_.clear();
    all_rewards.clear();
    reward_source.clear();
    task_actions_.clear();
    sensing_actions_.clear();
    observations_.clear();
    cumulative_reward_ = 0;
    // Reset planner
    planner_.reset();
    // Reset active sensing time
    active_sensing_time_ = 0;
    rtt_time_average = 0;
     rtt_time = 0;

   
}

void Simulator::updateSimulator(unsigned int sensing_action, Eigen::VectorXd observation, Eigen::VectorXd task_action)
{

   // double currentReward = 0;
	globalcheckflag = 1;
	nextflag=1;
    ROS_INFO("update simulator");
    Eigen::VectorXd new_state = model_.sampleNextState(states_.back(), task_action);
    //ROS_ERROR("current state is", states_.back());
    //ros::Duration(2).sleep();
    ROS_INFO("local new state0 is %f at round %d ",new_state(0), communication_count);
    ROS_INFO("local new state1 is %f at round %d",new_state(1), communication_count);
    ROS_INFO("local new state2 is %f at round %d", new_state(2), communication_count);
	nextflag=0;
    states_.push_back(new_state);
    sensing_actions_.push_back(sensing_action);
    observations_.push_back(observation);
    task_actions_.push_back(task_action);
   //currentReward = model_.getReward(new_state, task_action);
   cumulative_reward_ += model_.getReward(new_state, task_action);
  // all_rewards.push_back(currentReward);
   //reward_source.push_back(source);
   time(&raw_time);
   time_info = localtime(&raw_time);
   strftime(buffer, 80, "%Y-%m-%d", time_info);
   std::string file_name(buffer);
   std::string output_path = round_ + "averaged_rewards_local" + file_name +".txt";
  globalcheckflag = 0;
  timeout = 0;
}

void Simulator::updateSimulator(Eigen::VectorXd task_action)
{
    timeout = 0;
	globalcheckflag = 1;
	nextflag=1;
    ROS_INFO("updateelse simulator");
    Eigen::VectorXd new_state = model_.sampleNextState(states_.back(), task_action);
    ROS_INFO("local new state0 is %f, at round %d",new_state(0), communication_count);
    ROS_INFO("local new state1 is %f at round %d",new_state(1), communication_count);
	nextflag=0;
    states_.push_back(new_state);
    task_actions_.push_back(task_action);
    cumulative_reward_ += model_.getReward(new_state, task_action);
	globalcheckflag = 0;
    timeout = 0;
}


std::chrono::high_resolution_clock::time_point taskaction_start;
std::chrono::high_resolution_clock::time_point taskaction_finish;
std::chrono::duration<double> taskaction_elapsed_time;

std::chrono::high_resolution_clock::time_point active_sensing_start;
std::chrono::high_resolution_clock::time_point active_sensing_finish;
std::chrono::duration<double> active_sensing_elapsed_time;

std::chrono::high_resolution_clock::time_point rtt_start;
std::chrono::high_resolution_clock::time_point rtt_finish;
std::chrono::duration<double> rtt_elapsed_time;




void Simulator::simulate(const Eigen::VectorXd &init_state, unsigned int num_steps, unsigned int verbosity, unsigned int particles, unsigned int round)
{
    //status = standard;
    round_ = round;
    initSimulator();
    ncount=0;
    run = true;
    
  
    
    double active_sensing_time = 0;
    double observation_time = 0;
    double updatebelief_time =0;
    double predictbelief_time = 0;
    double total_updatebelief_time = 0;
    double total_predictbelief_time = 0;

    double avg_observation_time = 0;
    double avg_updatebelief_time = 0;
    double avg_total_updatebelief_time=0;
    double avg_taskaction_time = 0;
    double avg_predictbelief_time = 0;
    double avg_total_predictbelief_time = 0;
    states_.push_back(init_state);

    std::chrono::high_resolution_clock::time_point observation_start;
    std::chrono::high_resolution_clock::time_point observation_finish;
    std::chrono::high_resolution_clock::time_point updatebelief_start;
    std::chrono::high_resolution_clock::time_point updatebelief_finish;


    std::chrono::high_resolution_clock::time_point predictbelief_start;
    std::chrono::high_resolution_clock::time_point predictbelief_finish;
    std::chrono::high_resolution_clock::time_point recovery_start;
    std::chrono::high_resolution_clock::time_point recovery_finish;
    std::chrono::duration<double> observation_elapsed_time;
    std::chrono::duration<double> updatebelief_elapsed_time;
    std::chrono::duration<double> predictbelief_elapsed_time;
    std::chrono::duration<double> total_updatebelief_elapsed_time;
    std::chrono::duration<double> total_predictbelief_elapsed_time;
    std::chrono::duration<double> recovery_elapsed_time;
    if (verbosity > 0)
        std::cout << "state = \n" << states_.back().transpose() << std::endl;

    planner_.publishParticles();


    ros::NodeHandle nh;
   
    
 
  

    ros::ServiceClient client = nh.serviceClient<active_sensing_continuous_local::action_message2>("cloud_active_sensing");
    ros::ServiceClient client2 = nh.serviceClient<active_sensing_continuous_local::action_message2>("cloud_active_sensing_recovery");
    active_sensing_continuous_local::action_message2 srv;
  
   
 
    while(ros::ok&&!model_.isTerminal(states_.back()) && ncount < num_steps)
    {
        planner_.normalizeBelief();
        ROS_INFO("communication round is %d",communication_count);
	    ROS_INFO("n is %d",ncount);
        Observation_point=0;
        if (ncount % (sensing_interval_ + 1) == 0)
        {
           
                sensing_action = planner_.getSensingAction();
                active_sensing_start = std::chrono::high_resolution_clock::now();
                rtt_start = std::chrono::high_resolution_clock::now();
                ROS_INFO("Request sensing action");
                srv.request.type1 = 1;
             
            
      
            while (ros::ok)
            {
               Break_Point = 0;
              // ros::spinOnce();
               if (status == standard || status == keepAlive) {
                   recovery_start = std::chrono::high_resolution_clock::now();
                   if (client.waitForExistence(ros::Duration(5))) {
                       client.call(srv);
                           switch (srv.response.type)
                           {
                           case 1:
                          
                               if(allowOb == 0){ 
                               ROS_INFO("RECEIVE SENSING ACTION.");
                                   ROS_INFO("GETTING OBSERVATION.");
                                   rtt_finish = std::chrono::high_resolution_clock::now();
                                   rtt_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                                       (rtt_finish - rtt_start);
                                   rtt_time += rtt_elapsed_time.count();
                                   total_trips++;
                                   sensing_action_local = srv.response.x;
                                   
                                   active_sensing_finish = std::chrono::high_resolution_clock::now();
                                   if (Observation_point == 0)
                                   {
                                       observation = model_.sampleObservation(states_.back(), sensing_action_local);
                                       planner_.updateBelief(sensing_action, observation);
                                       task_action = planner_.getTaskAction();
                                       Observation_point = 1;
                                   }
                               server_communication_count = srv.response.serverCC;
                               server_n2 = srv.response.servern;
                               srv.request.x1 = observation(0);
                               srv.request.type1 = 2;
                               srv.request.recovery = 0;
                               //res.recovery = 0;
                               //observation = model_.sampleObservation(states_.back(), sensing_action);
                               //planner_.updateBelief(sensing_action, observation);
                               //task_action = planner_.getTaskAction(); 
                               taskaction_start = std::chrono::high_resolution_clock::now();
                               allowOb = 1;
                              
                               
                               }

                               break;
                           case 3:
                               //rtt_finish = std::chrono::high_resolution_clock::now();
                               //rtt_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                                  // (rtt_finish - rtt_start);
                               //rtt_time += rtt_elapsed_time.count();
                               //total_trips++;
                               ROS_INFO("RECEIVE TASK ACTION.");
                               ROS_INFO("PERFORMING");
                               ROS_INFO("TELL AWS CONTINUE");
                               allowOb = 0;
                               server_communication_count = srv.response.serverCC;
                               server_n2 = srv.response.servern;
                               task_action(0) = srv.response.x;
                               task_action(1) = srv.response.y;
                               task_action(2) = srv.response.z;
                               taskaction_finish = std::chrono::high_resolution_clock::now();
                               srv.request.recovery = 0;
                               srv.request.type1 = 4;

                               planner_.predictBelief(task_action);
                               updateSimulator(sensing_action, observation, task_action);
                               ncount++;
                               communication_count++;
                               server_n2++;
                               server_communication_count++;
                               Break_Point = 1;
                             //  rtt_start = std::chrono::high_resolution_clock::now();
                               client.call(srv);
                               break;

                           case 0:
                               server_communication_count = srv.response.serverCC;
                               server_n2 = srv.response.servern;
                             

                            
                               break;
                           case 7:
                               recovery_finish = std::chrono::high_resolution_clock::now();
                               recovery_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                                   (recovery_finish - recovery_start);
                               time_to_recovery = recovery_elapsed_time.count();
                               ROS_ERROR("Time to recover was %f ", time_to_recovery);
                               ros::Duration(1).sleep();
                                   break;

                       
                           }
                   }

               
                   else {

                       if (status == standard) {
                           ROS_ERROR("failed to call service from standard, launching back up");
                           ROS_ERROR("We're sending this n count %d ", server_n2);
                           status = localRecovery;
                           system("./serverRecovery.sh &>/dev/null");
                           client2.waitForExistence();
                           recovery_finish = std::chrono::high_resolution_clock::now();
                           recovery_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                               (recovery_finish - recovery_start);
                           time_to_recovery = recovery_elapsed_time.count();
                           rtt_start = std::chrono::high_resolution_clock::now();
                           srv.request.recovery = 1;
                           
                           srv.request.serverCCReply = communication_count;
                           srv.request.n = communication_count;
                         
                           if(communication_count > 0){
                               /*
                           srv.request.state1 = states_.end()[-2](0);
                           srv.request.state2 = states_.end()[-2](1);
                           srv.request.state3 = states_.end()[-2](2);
                            */
                           srv.request.state1 = states_.back()(0);
                           srv.request.state2 = states_.back()(1);
                           srv.request.state3 = states_.back()(2);

                           srv.request.observation = observation(0);
                           srv.request.t1 = task_action(0);
                           srv.request.y1 = task_action(1);
                           srv.request.z1 = task_action(2);
                          
                        
                           srv.request.sense = sensing_action_local;
                           
                           //client2.call(srv);

                           }
                       }
                       if (status == keepAlive) {
                           ROS_ERROR("failed to call service from keepAlive, switching which client we use");
                           status = localRecovery;
                       }
                   }
               }
               else {
                   if (client.call(srv)) {
                       status =keepAlive;
                      srv.request.recovery = 1;
                       ROS_ERROR("Called server while running loca");
                       //Party in here do something with recover switch which thing its chatting too all the good stuff

                   }
                   else {
                       ROS_ERROR("Local is sesnding %d", srv.request.type1);
                       //The stuff but with client2 calling it we party
                       if (client2.call(srv)) {
                           ROS_INFO("%d", srv.response.type);
                           switch (srv.response.type)
                           {
                           case 1:
                               ROS_INFO("RECEIVE SENSING ACTION IN BACKUP.");
                               ROS_INFO("GETTING OBSERVATION.");
                               rtt_finish = std::chrono::high_resolution_clock::now();
                               rtt_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                                   (rtt_finish - rtt_start);
                               rtt_time += rtt_elapsed_time.count();
                               total_trips++;
                               sensing_action_local = srv.response.x;
                               active_sensing_finish = std::chrono::high_resolution_clock::now();
                               if (Observation_point == 0)
                               {
                                   observation = model_.sampleObservation(states_.back(), sensing_action_local);
                                   planner_.updateBelief(sensing_action, observation);
                                   task_action = planner_.getTaskAction();
                                   Observation_point = 1;
                               }
                               srv.request.recovery = 0;
                               srv.request.x1 = observation(0);
                               srv.request.type1 = 2;
                               //res.recovery = 0;
                               //observation = model_.sampleObservation(states_.back(), sensing_action);
                               //planner_.updateBelief(sensing_action, observation);
                               //task_action = planner_.getTaskAction(); 
                               taskaction_start = std::chrono::high_resolution_clock::now();
                              // rtt_start = std::chrono::high_resolution_clock::now();
                               break;
                           case 3:
                              // rtt_finish = std::chrono::high_resolution_clock::now();
                               //rtt_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                                  // (rtt_finish - rtt_start);
                              // rtt_time += rtt_elapsed_time.count();
                               //total_trips++;
                               ROS_INFO("RECEIVE TASK ACTION.");
                               ROS_INFO("PERFORMING");
                               ROS_INFO("TELL AWS CONTINUE");

                               task_action(0) = srv.response.x;
                               task_action(1) = srv.response.y;
                               task_action(2) = srv.response.z;
                               taskaction_finish = std::chrono::high_resolution_clock::now();
                               srv.request.type1 = 4;
                               srv.request.recovery = 0;

                               planner_.predictBelief(task_action);
                               updateSimulator(sensing_action, observation, task_action);
                               ncount++;
                               communication_count++;
                               Break_Point = 1;

                               client2.call(srv);
                               //rtt_start = std::chrono::high_resolution_clock::now();
                               break;

                           case 0:
                               ROS_INFO("SUCCESSFUL CONTINUE IN CASE ZERO Not printing next thing");
                               

                               break;

                           case 5:
                               srv.request.type1 = srv.request.type1;
                               srv.request.recovery = 0;
                               break;
                           case 7: 
                               recovery_finish = std::chrono::high_resolution_clock::now();
                               recovery_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                                   (recovery_finish - recovery_start);
                               time_to_recovery = recovery_elapsed_time.count();
                               ROS_ERROR("Time to recover was %f ", time_to_recovery);
                               ros::Duration(1).sleep();
                                   break;
                           }
                       }
                       else {
                           ROS_ERROR("somehow we have failed to call service");
                       }
                   }
               }
           
                //observation = model_.sampleObservation(states_.back(), sensing_action);
                //planner_.updateBelief(sensing_action, observation);
                //task_action = planner_.getTaskAction();
                if (Break_Point == 1)
                {
                    break;
                }
            }
            taskaction_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                (taskaction_finish - taskaction_start);
            taskaction_time += taskaction_elapsed_time.count();

            active_sensing_elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                (active_sensing_finish - active_sensing_start);
            active_sensing_time += active_sensing_elapsed_time.count();
                if (verbosity > 0)
                {
                    std::cout << "n = " << ncount << std::endl;
                    std::cout << "sensing_action = " << sensing_action << std::endl;
                    std::cout << "observation = " << observation.transpose() << std::endl;
                    std::cout << "most_likely_state = " << planner_.getMaximumLikelihoodState().transpose() << std::endl;
                    std::cout << "task_action = " << task_action.transpose() << std::endl;
                    std::cout << "state = " << states_.back().transpose() << std::endl;
                }

            }
        
        else
        {
                //planner_.normalizeBelief();
                ROS_INFO("communication round is %d",communication_count);
	            ROS_INFO("n is %d",ncount);
                task_action = planner_.getTaskAction();
                ROS_INFO("local task_action(0)in else is %f",task_action(0));
                ROS_INFO("local task_action(1) in else is %f",task_action(1));
           
                planner_.predictBelief(task_action);
                updateSimulator(task_action);
                if (verbosity > 0)
                {
                    std::cout << "n = " << ncount << std::endl;
                    std::cout << "sensing_action = " << sensing_action << std::endl;
                    std::cout << "observation = " << observation.transpose() << std::endl;
                    std::cout << "most_likely_state = " << planner_.getMaximumLikelihoodState().transpose() << std::endl;
                    std::cout << "task_action = " << task_action.transpose() << std::endl;
                    std::cout << "state = " << states_.back().transpose() << std::endl;
                }
                ncount++;
                communication_count++;
                }
                
        
            planner_.publishParticles();
            model_.publishMap();
            if (has_publisher_)
            {
                publishState();
            }
           
            
           // ncount++;
            //communication_count++;
	        ROS_INFO("!model_.isTerminal(states_.back()) after loop is %d",!model_.isTerminal(states_.back()));
	        ROS_INFO("\n");
            
    }
    run = false;
    int num_sensing_steps = (ncount + 1) / (sensing_interval_ + 1);

    
    if (num_sensing_steps > 0)
    {
        rtt_time_average = rtt_time / num_sensing_steps;
        active_sensing_time_ = active_sensing_time / num_sensing_steps;
        taskaction_time_ = taskaction_time / num_sensing_steps;
        fstream file2;
        file2.open("Task action time average.txt", ios_base::out);
        file2 << taskaction_time_;
        file2.close();

        
    }
    else
    {
        active_sensing_time_ = 0;
        taskaction_time_ = 0;
    }
}



std::vector<Eigen::VectorXd> Simulator::getStates()
{
    return states_;
}

std::vector<unsigned int> Simulator::getSensingActions()
{
    return sensing_actions_;
}

std::vector<Eigen::VectorXd> Simulator::getTaskActions()
{
    return task_actions_;
}

std::vector<Eigen::VectorXd> Simulator::getObservations()
{
    return observations_;
}

double Simulator::getCumulativeReward()
{
   
    return cumulative_reward_;
}

double Simulator::getAverageActiveSensingTime()
{
    return active_sensing_time_;
}



double Simulator::getAvgTaskactionTime()
{
return taskaction_time_;
}

double Simulator::getAvgRttTime()
{
    return rtt_time_average;
}

double Simulator::getRecoveryTime() {
    return time_to_recovery;
}
void Simulator::publishState()
{
    if (has_publisher_)
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "my_frame"));

        visualization_msgs::Marker marker;

        uint32_t shape = visualization_msgs::Marker::CUBE;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::VectorXd state = states_.back();

        model_.fillMarker(state, marker);

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        publisher_.publish(marker);
    }
}
