#include <iostream>
#include <cmath>
#include <ctime>
#include <queue>
#include <string>
#include <utility>
#include <functional>
class UserTask
{
public:
    UserTask() {
    }
    UserTask(std::uint16_t deadline, std::uint16_t task_size, std::uint16_t task_id): m_deadline(deadline), m_task_size(task_size), m_task_id(task_id){
    }
    void print_id(){
        std::cout<< "Running task: Task ID: " << m_task_id << ", Task Size: " << m_task_size <<", Task Deadline: " << m_deadline << std::endl;
    }
    std::uint16_t get_deadline(){
        return m_deadline;
    }
    std::uint16_t get_task_size(){
        return m_task_size;
    }
private:
    std::uint16_t m_deadline;
    std::uint16_t m_task_size;
    std::uint16_t m_task_id;
};
template<typename T> void print_queue(T& q){
    while(!q.empty()) {
        auto task = q.top();
        task.print_id();
        q.pop();
    }
}
template <typename T>
void populate_queue(T &user_task_queue){
    srand((int) time(NULL));
    int i = 0;
    while (i++ < 10){
        int ds = (rand() % 2048 + 1);
        std::cout<<"randomly generated data size: " << ds << ", taskID: " << i << std::endl;
        int dl = (rand() % 360 + 1);
        std::cout<<"randomly generated data transfer deadline: " << dl << ", taskID: " << i << std::endl;
        user_task_queue.push(UserTask(ds,dl,i));
    }
}
// lambda is evaluated at compile time using constexpr
constexpr auto cmp_deadline = [](UserTask left, UserTask right) {
    return (left.get_deadline()) > (right.get_deadline()); };
constexpr auto cmp_size = [](UserTask left, UserTask right) {
    return (left.get_task_size()) < (right.get_task_size()); };
// so we can provide values to the aliases
// deadline_priority_queue type arranges the values according to the deadline
using deadline_priority_queue = std::priority_queue<UserTask, std::vector<UserTask>, decltype(cmp_deadline)>;
// size_priority_queue arranges the values according to the size
using size_priority_queue = std::priority_queue<UserTask, std::vector<UserTask>, decltype(cmp_size)>;
int main()
{
    deadline_priority_queue ordered_queue(cmp_deadline);
    populate_queue<deadline_priority_queue>(ordered_queue);
    //print_queue(ordered_queue);
    //std::cout << "======================================" << std::endl;
    //size_priority_queue ordered_queue_2(cmp_size);
    //populate_queue<size_priority_queue>(ordered_queue_2);
    //print_queue(ordered_queue_2);
    while(!ordered_queue.empty()){
        auto task = ordered_queue.top();
    
    std::string stack = "ns3";
    std::string sBandWidthW = "11m";
    std::string sBandWidthWD = "54m";
    std::string sRemainingDTTW, sRemainingDTTWD, sDataTransferTimeW, sDataTransferTimeWD;
    uint16_t mobiles = 2;
    uint16_t deadline = task.get_deadline();
    std::string sDeadline = std::to_string(deadline);
    double dataSize = task.get_task_size();
    double bandWidthW = 11.0;
    double MbtoMBW = bandWidthW/8;
    double bandWidthWD = 54.0;
    double MbtoMBWD = bandWidthWD/8;
    double availBandWidthW = MbtoMBW;
    double availBandWidthWD = MbtoMBWD;
    double finalData, maxDataW, maxDataWD, totalData, remainingData, dataTransferTimeW, dataTransferTimeWD, remainingDTTW, remainingDTTWD, remainingDTT;
    //Resource Allocation Algorithm
    dataTransferTimeW = std::round(dataSize / availBandWidthW);
    dataTransferTimeWD = std::round(dataSize / availBandWidthWD);
    maxDataW = availBandWidthW * deadline;
    maxDataWD = availBandWidthWD * deadline;
    if(maxDataW > dataSize){
        maxDataW = dataSize;
    }
    if(maxDataWD > dataSize){
        maxDataWD = dataSize;
    }
    if(maxDataWD > maxDataW){
        remainingData = dataSize - maxDataWD;
        remainingDTTW = remainingData / availBandWidthW;
        finalData = maxDataWD;
        totalData = remainingData + maxDataWD;
    } else {
        remainingData = dataSize - maxDataW;
        remainingDTTWD = remainingData / availBandWidthWD;
        finalData = maxDataW;
        totalData = remainingData + maxDataW;
    }
    remainingDTT = remainingDTTW + remainingDTTWD;
    if(totalData < dataSize || remainingDTT > deadline){
        std::cout << "This data transfer is not possible" << std::endl;
        ordered_queue.pop();
    } else {
        if(maxDataWD > maxDataW) {
            if(dataTransferTimeWD < deadline) {
                std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi Direct for the time: " << dataTransferTimeWD << "s" << std::endl;
                sDataTransferTimeWD = std::to_string(dataTransferTimeWD);
                ordered_queue.pop();
               // appIfTwo(nodes, sBandWidthWD, sDataTransferTimeWD, dataTransferTimeWD);
            } else {
                std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi Direct for the time: " << deadline << "s" << std::endl;
                sDeadline = std::to_string(deadline);
                ordered_queue.pop();
               // appIfTwo(nodes, sBandWidthWD, sDeadline, deadline);
            }
            if(remainingData == 0){
                std::cout << "No need to use another WCT" << std::endl;
            } else {
                std::cout << "Sending the remaining data " << remainingData << " to node using Wi-Fi for the time: " << remainingDTTW << "s" << std::endl;
                sRemainingDTTW = std::to_string(remainingDTTW);
                ordered_queue.pop();
              //  appIfOne(nodes, sBandWidthW, sRemainingDTTW, remainingDTTW);
            }
            
        } else {
            if(dataTransferTimeW < deadline) {
                std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi for the time: " << dataTransferTimeW << "s" << std::endl;
                sDataTransferTimeW = std::to_string(dataTransferTimeW);
                ordered_queue.pop();
              //  appIfOne(nodes, sBandWidthW, sDataTransferTimeW, dataTransferTimeW);
            } else {
                std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi for the time: " << deadline << "s" << std::endl;
                sDeadline = std::to_string(deadline);
                ordered_queue.pop();
              //  appIfOne(nodes, sBandWidthW, sDeadline, deadline);
            }
            if(remainingData == 0){
                std::cout << "No need to use another WCT" << std::endl;
            } else {
                std::cout << "Sending the remaining data " << remainingData << " to node using Wi-Fi Direct" << remainingDTTWD << "s" << std::endl;
                sRemainingDTTW = std::to_string(remainingDTTWD);
                ordered_queue.pop();
             //   appIfTwo(nodes, sBandWidthW, sRemainingDTTW, remainingDTTWD);
            }
        }
    }
}
    return 0;
}
