#ifndef __SAFE_QUEUE_H__
#define __SAFE_QUEUE_H__

#include <mutex>
#include <queue>
#include <iostream>
#include <condition_variable>

template <typename T>
class SafeQueue
{
private:

    std::queue<T> m_queue; //利用模板函数构造队列

    std::mutex m_mutex; // 访问互斥信号量

    std::condition_variable cond; // 条件变量，通知消费者消费，避免cpu轮询

    
public:

    SafeQueue() {}

    SafeQueue(SafeQueue &&other) {}
    
    ~SafeQueue() {}

    bool empty() // 返回队列是否为空
    {
        std::lock_guard<std::mutex> lock(m_mutex); // 互斥信号变量加锁，防止m_queue被改变

        return m_queue.empty();
    }

    int size()
    {
        std::lock_guard<std::mutex> lock(m_mutex); // 互斥信号变量加锁，防止m_queue被改变

        return m_queue.size();
    }

    // 队列添加元素
    void enqueue(const T &t)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.emplace(t);
        }
        cond.notify_one();  // 唤醒一个等待线程
        // cond.notify_all();  // 唤醒多个等待线程，假如有多个消费者
    }

    // 队列取出元素
    bool dequeue(T &t)
    {
        std::unique_lock<std::mutex> lock(m_mutex); // 队列加锁
        cond.wait(lock, [this] { return !m_queue.empty(); });  // 等待数据，没有数据就一直等
        // if (m_queue.empty())
        //     return false;
        t = std::move(m_queue.front()); // 取出队首元素，返回队首元素值，并进行右值引用

        m_queue.pop(); // 弹出入队的第一个元素

        return true;
    }
};

#endif /* __SAFE_QUEUE_H__ */
