#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
 #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
//#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


ros::Publisher vis_pub;
ros::Publisher traj_pub;
ros::Publisher point_info;

int point_num = 0;
bool status_flag = false;
bool odom_flag  = false;

geometry_msgs::PoseStamped pose_pub;


class planner {
public:
    //void setStart(double x, double y, double z,double x1,double y1,double z1,double w1)
    void setStart(double x, double y, double z)
    {
        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(x,y,z);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        /*
        start->as<ob::SO3StateSpace::StateType>(1)->x = x1;
        start->as<ob::SO3StateSpace::StateType>(1)->y = y1;
        start->as<ob::SO3StateSpace::StateType>(1)->z = z1;
        start->as<ob::SO3StateSpace::StateType>(1)->w = w1;
        */
        pdef->clearStartStates();
        pdef->addStartState(start);
    }
    //void setGoal(double x, double y, double z,double x1,double y1,double z1,double w1)
    void setGoal(double x, double y, double z)
    {
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(x,y,z);
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        /*
        goal->as<ob::SO3StateSpace::StateType>(1)->x = x1;
        goal->as<ob::SO3StateSpace::StateType>(1)->y = y1;
        goal->as<ob::SO3StateSpace::StateType>(1)->z = z1;
        goal->as<ob::SO3StateSpace::StateType>(1)->w = w1;
        */
        pdef->clearGoal();
        pdef->setGoalState(goal);
        std::cout << "goal set to: " << x << " " << y << " " << z << std::endl;
    }
    void updateMap(boost::shared_ptr<fcl::CollisionGeometry> map)
    {
        tree_obj = map;
    }
    // Constructor
    planner(void)
    {
        //四旋翼的障碍物几何形状
        Quadcopter = boost::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.3, 0.3, 0.05));
        //分辨率参数设置
        fcl::OcTree* tree = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.3)));
        tree_obj = boost::shared_ptr<fcl::CollisionGeometry>(tree);
        //解的状态空间
        space = ob::StateSpacePtr(new ob::SE3StateSpace());
        // create a start state
        ob::ScopedState<ob::SE3StateSpace> start(space);
       // create a goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        // set the bounds for the R^3 part of SE(3)
        // 搜索的三维范围设置
        ob::RealVectorBounds bounds(3);

        bounds.setLow(0,-5);
        bounds.setHigh(0,5);
        bounds.setLow(1,-5);
        bounds.setHigh(1,5);
        bounds.setLow(2,-1.5);
        bounds.setHigh(2,1.5);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // construct an instance of  space information from this state space
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
        start->setXYZ(0,0,0);
        /*
        start->as<ob::SO3StateSpace::StateType>(1)->x = 0;
        start->as<ob::SO3StateSpace::StateType>(1)->y = 0;
        start->as<ob::SO3StateSpace::StateType>(1)->z = 0;
        start->as<ob::SO3StateSpace::StateType>(1)->w = 1;
        */
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // start.random();
        goal->setXYZ(0,0,0);
        /*
        goal->as<ob::SO3StateSpace::StateType>(1)->x = 0;
        goal->as<ob::SO3StateSpace::StateType>(1)->y = 0;
        goal->as<ob::SO3StateSpace::StateType>(1)->z = 0;
        goal->as<ob::SO3StateSpace::StateType>(1)->w = 1;
        */
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // goal.random();
      
        // set state validity checking for this space
        si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1 ));
        // create a problem instance
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);
        // set Optimizattion objective
        pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));
        std::cout << "Initialized: " << std::endl;
    }
    
    // Destructor
    ~planner()
    {
    }
    void replan(void)
    {
        std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
        if(path_smooth->getStateCount () <= 2)
            plan();
        else
        {
            for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
            {
                if(!replan_flag)
                    replan_flag = !isStateValid(path_smooth->getState(idx));
                else
                    break;

            }
            if(replan_flag)
                plan();
            else
                std::cout << "Replanning not required" << std::endl;
        }
        
    }
    void plan(void)
    {

        // create a planner for the defined space
        og::RRTstar* rrt = new og::RRTstar(si);
        //设置rrt的参数range
        rrt->setRange(0.2);
        ob::PlannerPtr plan(rrt);
        // set the problem we are trying to solve for the planner
        plan->setProblemDefinition(pdef);
        // perform setup steps for the planner
        plan->setup();
        // print the settings for this space
        si->printSettings(std::cout);
        std::cout << "problem setting\n";
        // print the problem settings
        pdef->print(std::cout);
        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = plan->solve(1);
        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            std::cout << "Found solution:" << std::endl;
            ob::PathPtr path = pdef->getSolutionPath();
            // og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
            pth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            //pth->printAsMatrix(std::cout);
            // print the path to screen
            // path->print(std::cout);
            
            nav_msgs::Path msg_temp;
            msg_temp.header.stamp = ros::Time::now();
            msg_temp.header.frame_id = "map";

            for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++)
            {
                const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();
                // extract the first component of the state and cast it to what we expect
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
                // extract the second component of the state and cast it to what we expect
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
                geometry_msgs::PoseStamped pose;

                pose.pose.position.x = pos->values[0];
                pose.pose.position.y = pos->values[1];
                pose.pose.position.z = pos->values[2];

                pose.pose.orientation.x = rot->x;
                pose.pose.orientation.y = rot->y;
                pose.pose.orientation.z = rot->z;
                pose.pose.orientation.w = rot->w;

                msg_temp.poses.push_back(pose);

            }
            traj_pub.publish(msg_temp);
            msg = msg_temp;
            //Path smoothing using bspline
            //B样条曲线优化
            og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
            path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            pathBSpline->smoothBSpline(*path_smooth,3);
            // path_smooth.print(std::cout);
            //Publish path as markers
            nav_msgs::Path smooth_msg_temp;
            smooth_msg_temp.header.stamp = ros::Time::now();
            smooth_msg_temp.header.frame_id = "map";
            
            //geometry_msgs::PoseStamped pose_pub;
            for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
            {
                    // cast the abstract state type to the type we expect
                const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

                // extract the first component of the state and cast it to what we expect
                const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // extract the second component of the state and cast it to what we expect
                const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
                
                geometry_msgs::PoseStamped point;
//              pose.header.frame_id = "/world"

                point.pose.position.x = pos->values[0];
                point.pose.position.y = pos->values[1];
                point.pose.position.z = pos->values[2];

                point.pose.orientation.x = rot->x;
                point.pose.orientation.y = rot->y;
                point.pose.orientation.z = rot->z;
                point.pose.orientation.w = rot->w;

                smooth_msg_temp.poses.push_back(point);

                //std::cout << "Published marker: " << idx << std::endl;
            }

            vis_pub.publish(smooth_msg_temp);
            smooth_msg = smooth_msg_temp;
            // ros::Duration(0.1).sleep();
            status_flag = true;//规划完成标志位
            // Clear memory
            pdef->clearSolutionPaths();
            replan_flag = false;
        }
        else
            std::cout << "No solution found" << std::endl;
        point_num = 0;
    }
    nav_msgs::Path smooth_msg;
    nav_msgs::Path msg;
    og::PathGeometric* path_smooth;
    og::PathGeometric* pth;
    
private:

    // construct the state space we are planning in
    ob::StateSpacePtr space;
    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si;
    // create a problem instance
    ob::ProblemDefinitionPtr pdef;
    
    bool replan_flag = false;
    
    
    boost::shared_ptr<fcl::CollisionGeometry> Quadcopter;
    boost::shared_ptr<fcl::CollisionGeometry> tree_obj;
    bool isStateValid(const ob::State *state)
    {
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        fcl::CollisionObject treeObj((tree_obj));
        fcl::CollisionObject aircraftObject(Quadcopter);
        // check validity of state defined by pos & rot
        fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        aircraftObject.setTransform(rotation, translation);
        fcl::CollisionRequest requestType(1,false,1,false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);
        return(!collisionResult.isCollision());
    }

    // Returns a structure representing the optimization objective to use
    // for optimal motion planning. This method returns an objective which
    // attempts to minimize the length in configuration space of computed
    // paths.
    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        return obj;
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
        return obj;
    }

};


void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{
    //std::cout << " got octomap! " << std::endl;
    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    fcl::OcTree* tree = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(tree_oct));
        
    planner_ptr->updateMap(boost::shared_ptr<fcl::CollisionGeometry>(tree));
    
    if(status_flag)
        planner_ptr->replan();

}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
{
    //std::cout << "test" << std::endl;
    planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    //msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    odom_flag = true;
    if(status_flag)
    {
        std::cout << "total num" << planner_ptr->path_smooth->getStateCount() << std::endl;
        if((msg->pose.pose.position.x - planner_ptr->smooth_msg.poses[point_num].pose.position.x)*
        (msg->pose.pose.position.x - planner_ptr->smooth_msg.poses[point_num].pose.position.x) + 
        (msg->pose.pose.position.y - planner_ptr->smooth_msg.poses[point_num].pose.position.y)* 
        (msg->pose.pose.position.y - planner_ptr->smooth_msg.poses[point_num].pose.position.y)
        < 0.09)
        { 
            point_num++;
            std::cout << "the" << point_num << "time" << std::endl;
        }
        if(point_num == planner_ptr->path_smooth->getStateCount())
        {
            std::cout << "reach the target" << std::endl;
            point_num = planner_ptr->path_smooth->getStateCount() - 1;
        }
        
        pose_pub = planner_ptr->smooth_msg.poses[point_num];
        std::cout << "msg publish success" << std::endl;
        pose_pub.header.stamp = ros::Time::now();
        pose_pub.header.frame_id = "map";
        point_info.publish(pose_pub);
    }
}

void setGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg, planner* planner_ptr)
{
    planner_ptr->setGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if(odom_flag)
        planner_ptr->plan();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_planner");
    ros::NodeHandle n;
    planner planner_object;
 
    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap", 1, boost::bind(&octomapCallback, _1, &planner_object));
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/rtabmap/odom", 1, boost::bind(&odomCb, _1, &planner_object));
    
    //ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&startCb, _1, &planner_object));
    ros::Subscriber goal_sub= n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100, boost::bind(&setGoalCallback, _1, &planner_object));


    vis_pub = n.advertise<nav_msgs::Path>( "visualization_marker", 0);
    traj_pub = n.advertise<nav_msgs::Path>("waypoints",1);
    point_info = n.advertise<geometry_msgs::PoseStamped>("next_point",1);
    
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    ros::spin();

    return 0;
}
