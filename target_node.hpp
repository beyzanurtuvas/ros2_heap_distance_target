#ifndef TARGET_NODE_HPP
#define TARGET_NODE_HPP

#include "rclcpp/rclcpp.hpp"

struct Target
{
    int id;
    int distance;
};

class TargetNode : public rclcpp::Node
{
public:
    TargetNode();

private:
    Target heap[20];
    int heapSize;
    int nextId;

    rclcpp::Time last_target_time;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop();

    void swap(Target &x, Target &y);
    void heapifyMin(int i);
    void insertMinHeap(Target value);
    Target deleteMin();
};

#endif
