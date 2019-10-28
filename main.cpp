#include <string>
#include <cmath>
#include <iostream>
#include <ctime>
#include <queue>
#include <utility>
#include <functional>
#include <thread>
#include <chrono>
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
        user_task_queue.push(UserTask(dl,ds,i));
    std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 2000 + 1));
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
  //  populate_queue<size_priority_queue>(ordered_queue_2);
    //print_queue(ordered_queue_2);
    while(!ordered_queue.empty()){
        auto task = ordered_queue.top();
    
        std::string stack = "ns3";
        std::string sBandWidthW = "11Mbps";
        std::string sBandWidthWD = "54Mbps";
        std::string sRemainingDTTW, sRemainingDTTWD, sDataTransferTimeW, sDataTransferTimeWD;
        uint16_t mobiles = 2;
        uint16_t deadline = task.get_deadline();
        std::cout << "deadline: " << deadline << std::endl;
    
        std::string sDeadline = std::to_string(deadline);
        double dataSize = task.get_task_size();
        std::cout << "dataSize: " << dataSize << std::endl;
        double bandWidthW = 11.0;
        double MbtoMBW = bandWidthW/8;
        double bandWidthWD = 54.0;
        double MbtoMBWD = bandWidthWD/8;
        double dBandWidthW = MbtoMBW;
        double dBandWidthWD = MbtoMBWD;
        double finalData = 0;
        double maxDataW = 0;
        double maxDataWD = 0;
        double totalData = 0;
        double remainingData = 0;
        double dataTransferTimeW = 0;
        double dataTransferTimeWD = 0;
        double remainingDTTW = 0;
        double remainingDTTWD = 0;
        double remainingDTT = 0;
    //Resource Allocation Algorithm
        dataTransferTimeW = std::round(dataSize / dBandWidthW);
        std::cout << "dataTransferTimeW: " << dataTransferTimeW << std::endl;
        dataTransferTimeWD = std::round(dataSize / dBandWidthWD);
        std::cout << "dataTransferTimeWD: " << dataTransferTimeWD << std::endl;
        maxDataW = dBandWidthW * deadline;
        std::cout << "maxDataW: " << maxDataW << std::endl;
        maxDataWD = dBandWidthWD * deadline;
        std::cout << "maxDataWD: " << maxDataWD << std::endl;
    
        if(maxDataW > dataSize){
            maxDataW = dataSize;
        }
        if(maxDataWD > dataSize){
            maxDataWD = dataSize;
        }
        if(maxDataWD > maxDataW){
            remainingData = dataSize - maxDataWD;
            remainingDTTW = remainingData / dBandWidthW;
            finalData = maxDataWD;
            totalData = remainingData + maxDataWD;
        } else {
            remainingData = dataSize - maxDataW;
            remainingDTTWD = remainingData / dBandWidthWD;
            finalData = maxDataW;
            totalData = remainingData + maxDataW;
        }
    
        remainingDTT = remainingDTTW + remainingDTTWD;
    
        std::cout << "remainingData: " << remainingData << std::endl;
        std::cout << "remainingDTTW: " << remainingDTTW << std::endl;
        std::cout << "remainingDTTWD: " << remainingDTTWD << std::endl;
        std::cout << "finalData: " << finalData << std::endl;
        std::cout << "totalData: " << totalData << std::endl;
        std::cout << "remainingDTT: " << remainingDTT << std::endl;
    
        if(totalData < dataSize || remainingDTT > deadline){
            std::cout << "This data transfer is not possible" << std::endl;
            ordered_queue.pop();
        } else {
            if(maxDataWD > maxDataW) {
                if(dataTransferTimeWD < deadline) {
                    std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi Direct for the time: " << dataTransferTimeWD << "s" << std::endl;
                    sDataTransferTimeWD = std::to_string(dataTransferTimeWD);
                    // appIfTwo(nodes, sBandWidthWD, sDataTransferTimeWD, dataTransferTimeWD);
                    ordered_queue.pop();
                } else {
                    std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi Direct for the time: " << deadline << "s" << std::endl;
                    //sDeadline = std::to_string(deadline);
                    // appIfTwo(nodes, sBandWidthWD, sDeadline, deadline);
                    ordered_queue.pop();
                }
    
            if(remainingData == 0){
                std::cout << "No need to use another WCT" << std::endl;
            } else {
                std::cout << "Sending the remaining data " << remainingData << " to node using Wi-Fi for the time: " << remainingDTTW << "s" << std::endl;
                sRemainingDTTW = std::to_string(remainingDTTW);
                //appIfOne(nodes, sBandWidthW, sRemainingDTTW, remainingDTTW);
                ordered_queue.pop();
            }
            
            } else {
                if(dataTransferTimeW < deadline) {
                    std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi for the time: " << dataTransferTimeW << "s" << std::endl;
                    sDataTransferTimeW = std::to_string(dataTransferTimeW);
                    //appIfOne(nodes, sBandWidthW, sDataTransferTimeW, dataTransferTimeW);
                    ordered_queue.pop();
                } else {
                    std::cout << "Sending the maximum data " << finalData << " to node using Wi-Fi for the time: " << deadline << "s" << std::endl;
                    //sDeadline = std::to_string(deadline);
                    //appIfOne(nodes, sBandWidthW, sDeadline, deadline);
                    ordered_queue.pop();
                }
                if(remainingData == 0){
                    std::cout << "No need to use another WCT" << std::endl;
                } else {
                    std::cout << "Sending the remaining data " << remainingData << " to node using Wi-Fi Direct" << remainingDTTWD << "s" << std::endl;
                    sRemainingDTTW = std::to_string(remainingDTTWD);
                    //appIfTwo(nodes, sBandWidthW, sRemainingDTTW, remainingDTTWD);
                    ordered_queue.pop();
                }
            }
        }
    }
    return 0;
}
