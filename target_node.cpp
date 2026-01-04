#include "target_node/target_node.hpp"
#include <cstdlib>
#include <ctime>

using namespace std::chrono_literals;

TargetNode::TargetNode() : Node("target_node")
{
    srand(time(nullptr));

    heapSize = 0;
    nextId = 5;

    insertMinHeap({1, 15});
    insertMinHeap({2, 5});
    insertMinHeap({3, 20});
    insertMinHeap({4, 8});

    last_target_time = this->now();

    timer_ = this->create_wall_timer(
        1s, std::bind(&TargetNode::controlLoop, this));
}

void TargetNode::swap(Target &x, Target &y)
{
    Target temp = x;
    x = y;
    y = temp;
}

void TargetNode::heapifyMin(int i)
{
    int smallest = i;
    int left = 2 * i + 1;
    int right = 2 * i + 2;

    if (left < heapSize && heap[left].distance < heap[smallest].distance)
        smallest = left;

    if (right < heapSize && heap[right].distance < heap[smallest].distance)
        smallest = right;

    if (smallest != i)
    {
        swap(heap[i], heap[smallest]);
        heapifyMin(smallest);
    }
}

void TargetNode::insertMinHeap(Target value)
{
    int i = heapSize;
    heap[i] = value;
    heapSize++;

    while (i != 0 && heap[(i - 1) / 2].distance > heap[i].distance)
    {
        swap(heap[i], heap[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

Target TargetNode::deleteMin()
{
    Target minValue = heap[0];
    heap[0] = heap[heapSize - 1];
    heapSize--;
    heapifyMin(0);
    return minValue;
}

void TargetNode::controlLoop()
{
    if (heapSize == 0)
        return;

    Target current = heap[0];
    double elapsed = (this->now() - last_target_time).seconds();

    RCLCPP_INFO(this->get_logger(),
        "Aktif hedef -> ID: %d | Mesafe: %d",
        current.id, current.distance);

    if (elapsed >= 10.0)
    {
        deleteMin();
        insertMinHeap({nextId++, rand() % 25 + 1});
        last_target_time = this->now();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetNode>());
    rclcpp::shutdown();
    return 0;
}
