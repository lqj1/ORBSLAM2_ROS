//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_IPUBLISHERTHREAD_H
#define ORB_SLAM2_IPUBLISHERTHREAD_H

#include <mutex>


namespace ORB_SLAM2
{

class System;

class IPublisherThread
{
public:
    IPublisherThread();
    virtual ~IPublisherThread();

    virtual void Run() = 0;

    void SetSystem(System *sys);
    System* GetSystem() { return mpSystem; }
    
    void RequestStop();
    bool isStopped();
    void Release();
    void RequestFinish();
    bool isFinished();

protected:
    bool WaitCycleStart();
    
    bool Stop();
    bool CheckFinish();
    void SetFinish(bool value = true);

private:
    std::mutex mMutexStop;
    bool mbStopRequested;
    bool mbStopped;

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    // Mutex for general object mutation
    std::mutex mMutexMut;
    System *mpSystem;
};

}


#endif //ORB_SLAM2_IPUBLISHERTHREAD_H
