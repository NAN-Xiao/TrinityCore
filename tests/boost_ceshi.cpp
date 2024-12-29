#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <string>
#include <memory>
#include <deque>
#include <mutex>
#include <algorithm>

using boost::asio::ip::tcp;

// 聊天消息结构体，包含用户名和消息内容
// 用于在服务器端各个部分传递和处理来自客户端的消息
struct ChatMessage {
    std::string username;  // 发送消息的客户端用户名
    std::string message;   // 具体的聊天消息内容
};

// 自定义的线程池类的前置声明（假设其定义在别处，按你之前代码框架的要求）
namespace Trinity {
class ThreadPool;
}

// 聊天会话类，代表与一个客户端的连接
class ChatSession : public std::enable_shared_from_this<ChatSession> {
public:
    // 构造函数，接收io_context用于后续的异步I/O操作，以及所有会话的引用
    // 这样可以方便在该会话中与其他会话进行交互（比如广播消息）
    ChatSession(boost::asio::io_context& io_context, std::vector<std::shared_ptr<ChatSession>>& sessions)
        : socket_(io_context), sessions_(sessions) {}

    // 返回与该会话关联的套接字引用，方便进行网络读写等操作
    tcp::socket& socket() {
        return socket_;
    }

    // 启动会话的方法，开始接收客户端发送的用户名
    void start() {
        // 使用asio的async_read函数异步地从套接字读取数据到username_缓冲区
        // 当读取完成（成功或失败）时，会调用handle_read_username方法，并传递错误码作为参数
        boost::asio::async_read(socket_, boost::asio::buffer(username_, 1024),
                                boost::bind(&ChatSession::handle_read_username, shared_from_this(),
                                            boost::asio::placeholders::error));
    }

private:
    // 处理读取用户名操作完成后的回调方法
    void handle_read_username(const boost::system::error_code& error) {
        if (!error) {
            // 调整用户名的长度，去除多余的填充字符（因为async_read可能会填充多余字节）
            username_.resize(strlen(username_));
            // 用户名读取成功后，开始异步接收聊天消息
            boost::asio::async_read(socket_, boost::asio::buffer(buffer_, 1024),
                                    boost::bind(&ChatSession::handle_read_message, shared_from_this(),
                                                boost::asio::placeholders::error));
        } else {
            // 如果读取用户名出现错误，调用remove_session方法移除该会话
            remove_session();
        }
    }

    // 处理读取聊天消息操作完成后的回调方法
    void handle_read_message(const boost::system::error_code& error) {
        if (!error) {
            // 同样调整消息的长度，去除多余填充字节
            buffer_.resize(strlen(buffer_));
            // 构造一个ChatMessage结构体，包含用户名和消息内容
            ChatMessage msg{username_, buffer_};
            // 将该消息广播给其他客户端
            broadcast_message(msg);
            // 继续异步接收下一条消息，形成循环接收消息的机制
            boost::asio::async_read(socket_, boost::asio::buffer(buffer_, 1024),
                                    boost::bind(&ChatSession::handle_read_message, shared_from_this(),
                                                boost::asio::placeholders::error));
        } else {
            // 如果读取消息出现错误，调用remove_session方法移除该会话
            remove_session();
        }
    }

    // 将给定的聊天消息广播给除当前会话之外的其他所有客户端
    void broadcast_message(const ChatMessage& msg) {
        // 使用互斥锁保护会话列表的并发访问，防止在多线程环境下出现数据不一致问题
        std::lock_guard<std::mutex> guard(sessions_mutex_);
        for (auto& session : sessions_) {
            // 排除当前会话自身，不向自己广播消息
            if (session.get()!= this) {
                bool write_in_progress =!write_msgs_.empty();
                // 将消息添加到待发送消息队列中
                write_msgs_.push_back(msg);
                if (!write_in_progress) {
                    // 如果当前会话没有正在进行的发送操作，就开始发送消息
                    do_write();
                }
            }
        }
    }

    // 执行将待发送消息队列中的消息发送给客户端的操作
    void do_write() {
        // 使用asio的async_write函数异步地将消息发送给客户端
        // 发送完成后，会调用下面的lambda表达式作为回调，传递错误码和已发送字节数作为参数
        boost::asio::async_write(socket_, boost::asio::buffer(write_msgs_.front().username + ": " + write_msgs_.front().message + "\n",
                                                              write_msgs_.front().username.size() + write_msgs_.front().message.size() + 2),
                                 [this](const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
                                     if (!error) {
                                         // 如果发送成功，使用互斥锁保护待发送消息队列的并发访问
                                         std::lock_guard<std::mutex> guard(write_msgs_mutex_);
                                         // 将已发送的消息从队列头部移除
                                         write_msgs_.pop_front();
                                         if (!write_msgs_.empty()) {
                                             // 如果队列中还有未发送的消息，继续发送下一条消息
                                             do_write();
                                         }
                                     } else {
                                         // 如果发送出现错误，调用remove_session方法移除该会话
                                         remove_session();
                                     }
                                 });
    }

    // 从会话列表中移除当前会话的方法
    void remove_session() {
        // 使用互斥锁保护会话列表的并发访问
        std::lock_guard<std::mutex> guard(sessions_mutex_);
        // 在会话列表中查找当前会话的迭代器
        auto it = std::find(sessions_.begin(), sessions_.end(), shared_from_this());
        if (it!= sessions_.end()) {
            // 如果找到，从会话列表中删除该会话
            sessions_.erase(it);
        }
    }

    tcp::socket socket_;  // 与客户端通信的套接字对象
    std::vector<std::shared_ptr<ChatSession>>& sessions_;  // 所有聊天会话的引用，用于广播消息等操作
    std::mutex sessions_mutex_;  // 保护会话列表并发访问的互斥锁
    char username_[1024];  // 用于存储客户端发送的用户名的缓冲区
    char buffer_[1024];  // 用于存储客户端发送的聊天消息的缓冲区
    std::deque<ChatMessage> write_msgs_;  // 待发送给客户端的消息队列
    std::mutex write_msgs_mutex_;  // 保护待发送消息队列并发访问的互斥锁
};

// 聊天服务器类
class ChatServer {
public:
    // 构造函数，接收io_context用于网络相关的异步操作，端口号用于监听客户端连接，以及线程数量用于初始化线程池
    ChatServer(boost::asio::io_context& io_context, short port, int numThreads)
        : acceptor_(io_context, tcp::endpoint(tcp::v4(), port)),
          threadPool_(std::make_unique<Trinity::ThreadPool>(numThreads)),
          sessions_() {}

    // 启动服务器的方法，开始接受客户端连接
    void start() {
        start_accept();
    }

private:
    // 开始接受客户端连接的私有方法，这是一个循环操作，不断等待新的客户端连接
    void start_accept() {
        // 创建一个新的聊天会话对象，用于与新连接的客户端进行交互
        auto new_session = std::make_shared<ChatSession>(acceptor_.get_executor().context(), sessions_);
        // 使用acceptor的async_accept方法异步地等待客户端连接
        // 当有客户端连接成功时，会调用下面的lambda表达式作为回调，传递错误码作为参数
        acceptor_.async_accept(new_session->socket(),
                               [this, new_session](const boost::system::error_code& error) {
                                   if (!error) {
                                       // 如果连接成功，将新会话加入会话列表
                                       std::lock_guard<std::mutex> guard(sessions_mutex_);
                                       sessions_.push_back(new_session);
                                       // 通过线程池提交任务，让线程池中的线程去启动该新会话的相关操作（如接收消息等）
                                       threadPool_->PostWork([new_session]() { new_session->start(); });
                                   }
                                   // 无论本次连接是否成功，都继续等待下一个客户端连接
                                   start_accept();
                               });
    }

    tcp::acceptor acceptor_;  // 用于监听客户端连接的TCP接受器对象
    std::unique_ptr<Trinity::ThreadPool> threadPool_;  // 自定义的线程池对象指针，用于管理处理客户端连接的线程
    std::vector<std::shared_ptr<ChatSession>> sessions_;  // 所有聊天会话的列表，用于管理和维护与客户端的连接
    std::mutex sessions_mutex_;  // 保护会话列表并发访问的互斥锁
};

int main(int argc, char* argv[]) {
    try {
        // 检查命令行参数数量是否正确
        if (argc!= 3) {
            std::cerr << "Usage: chat_server <port> <num_threads>\n";
            return 1;
        }

        boost::asio::io_context io_context;
        short port = std::atoi(argv[1]);  // 获取端口号参数并转换为整数类型
        int numThreads = std::atoi(argv[2]);  // 获取线程数量参数并转换为整数类型

        // 创建聊天服务器对象，并传入io_context、端口号和线程数量进行初始化
        ChatServer server(io_context, port, numThreads);
        server.start();  // 启动服务器，开始接受客户端连接

        // 运行io_context的事件循环，开始处理异步任务（如客户端连接、消息收发等）
        io_context.run();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}