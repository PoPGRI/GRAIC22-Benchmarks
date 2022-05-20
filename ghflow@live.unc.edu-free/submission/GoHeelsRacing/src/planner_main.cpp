#include "ros/ros.h"
#include "VoronoiPlanner.h"
#include "GoHeelsRacing/LineSegment.h"
#include "GoHeelsRacing/VoronoiPlannerInput.h"
#include "GoHeelsRacing/VoronoiPlannerOutput.h"
#include <ostream>
#include <signal.h>
#include <functional>

class ManageROS
{
public:
    ManageROS()
    {
        planner_sub = ros_node.subscribe<GoHeelsRacing::VoronoiPlannerInput>("planner_input", 1, &ManageROS::OnVoronoiInput, this);
        planner_pub = ros_node.advertise<GoHeelsRacing::VoronoiPlannerOutput>("planner_output", 1);
    }

    void OnVoronoiInput(const GoHeelsRacing::VoronoiPlannerInput::ConstPtr &msg)
    {
        std::cout << "Input. " << std::flush;
        // Convert VoronoiPlannerInput
        point_type car_location(msg->car_location.x, msg->car_location.y);
        point_type milestone(msg->milestone.x, msg->milestone.y);
        float allowed_obs_dist = msg->allowed_obs_dist;
        std::vector<segment_type>
            obstacles;
        for (auto &obs : msg->obstacles)
        {
            point_type start(obs.start.x, obs.start.y);
            point_type end(obs.end.x, obs.end.y);
            obstacles.push_back(segment_type(start, end));
        }

        const std::vector<point_type> &plan = planner.GetPlan(car_location, milestone, obstacles, allowed_obs_dist);
        std::cout << "Plan. " << std::flush;

        std::vector<segment_type> roadmap;
        planner.GetRoadmapSegments(roadmap);

        Publish(plan, roadmap);

        std::cout << "Published." << std::endl
                  << std::flush;
    }

    void Publish(const std::vector<point_type> &plan, std::vector<segment_type> &roadmap) const
    {
        GoHeelsRacing::VoronoiPlannerOutput output;
        for (auto &point : plan)
        {
            geometry_msgs::Vector3 outPoint;
            outPoint.x = point.x();
            outPoint.y = point.y();
            outPoint.z = 0.0;
            output.plan.push_back(outPoint);
        }
        for (auto &segment : roadmap)
        {
            GoHeelsRacing::LineSegment outSegment;
            outSegment.start.x = segment.low().x();
            outSegment.start.y = segment.low().y();
            outSegment.start.z = 0.0;
            outSegment.end.x = segment.high().x();
            outSegment.end.y = segment.high().y();
            outSegment.end.z = 0.0;
            output.roadmap.push_back(outSegment);
        }
        planner_pub.publish(output);
    }

private:
    VoronoiPlanner planner;

    ros::NodeHandle ros_node;
    ros::Subscriber planner_sub;
    ros::Publisher planner_pub;
};

// namespace
// {
//     std::function<void(int)> shutdown_handler;
//     void signal_handler(int signal) { shutdown_handler(signal); }
// } // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voronoi_planner");
    ManageROS manageROS;

    // Capture Ctr+C to stop the car.
    // signal(SIGINT, signal_handler);
    // shutdown_handler = [](int sig) {
    //     ros::shutdown();
    //     return EXIT_SUCCESS;
    // };

    ros::spin();

    return EXIT_SUCCESS;
}