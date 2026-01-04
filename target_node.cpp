#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstdlib>
#include <ctime>

using namespace std::chrono_literals;

struct Target {
    int id;
    int distance;
};

void swap(Target &x, Target &y)
{
    Target temp = x;
    x = y;
    y = temp;
}

void heapifyMin(Target arr[], int N, int i)
{
    int smallest = i;
    int left = 2*i + 1;
    int right = 2*i + 2;

    if (left < N && arr[left].distance < arr[smallest].distance)
        smallest = left;

    if (right < N && arr[right].distance < arr[smallest].distance)
        smallest = right;

    if (smallest != i) {
        swap(arr[i], arr[smallest]);
        heapifyMin(arr, N, smallest);
    }
}

void insertMinHeap(Target arr[], int &N, Target value)
{
    int i = N;
    arr[i] = value;
    N++;

    while (i != 0 && arr[(i-1)/2].distance > arr[i].distance) {
        swap(arr[i], arr[(i-1)/2]);
        i = (i-1)/2;
    }
}

Target deleteMin(Target arr[], int &N)
{
    Target minValue = arr[0];
    arr[0] = arr[N-1];
    N--;
    heapifyMin(arr, N, 0);
    return minValue;
}

class TargetManager : public rclcpp::Node
{
public:
    TargetManager() : Node("target_manager")
    {
        srand(time(nullptr));
        heapSize = 0;
        nextId = 5;

        insertMinHeap(heap, heapSize, {1, 15});
        insertMinHeap(heap, heapSize, {2, 5});
        insertMinHeap(heap, heapSize, {3, 20});
        insertMinHeap(heap, heapSize, {4, 8});

        last_target_time = this->now();

        timer_ = this->create_wall_timer(
            1s, std::bind(&TargetManager::controlLoop, this));
    }

private:
    Target heap[20];
    int heapSize;
    int nextId;

    rclcpp::Time last_target_time;
    rclcpp::TimerBase::SharedPtr timer_;

    void controlLoop()
    {
        if (heapSize == 0)
            return;

        Target current = heap[0];

        auto now = this->now();
        double elapsed = (now - last_target_time).seconds();

        RCLCPP_INFO(this->get_logger(),
            "Aktif hedef -> ID: %d Mesafe: %d",
            current.id, current.distance);

        if (elapsed >= 10.0)
        {
            RCLCPP_WARN(this->get_logger(),
                "10 saniye doldu, hedef degistiriliyor");

            deleteMin(heap, heapSize);
            insertMinHeap(heap, heapSize,
                {nextId++, rand() % 25 + 1});

            last_target_time = now;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetManager>());
    rclcpp::shutdown();
    return 0;
}
