#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "ipc_test/ipc_test_consumer.hpp"

class TestIpcConsumer : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // rclcpp 초기화
        rclcpp::init(0, nullptr);
        
        // Consumer 노드 생성
        auto consumer_node = std::make_shared<Consumer>("test_consumer", "test_input");

        // Publisher 노드 생성
        auto pub_node = std::make_shared<rclcpp::Node>("test_publisher");
        auto publisher = pub_node->create_publisher<std_msgs::msg::Float64>("test_input", 10);

        // Executor 설정
        // rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(consumer_node);
        executor.add_node(pub_node);
        thread = std::thread([this]() { this->executor.spin(); });
    }

    void TearDown() override
    {
        // Executor 종료 및 rclcpp 종료
        executor.cancel();
        thread.join();
        rclcpp::shutdown();
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<Consumer> consumer_node;
    std::shared_ptr<rclcpp::Node> pub_node;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
    std::thread thread;
    
    // 수신된 메시지를 저장할 변수
    double received_data = 0.0;

    // 메시지를 수신하는 Callback
    void messageCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        received_data = msg->data;
    }
};

// Consumer가 메시지를 올바르게 수신하는지 테스트
TEST_F(TestIpcConsumer, TestReceiveMessage)
{
    // 수신 콜백 등록
    consumer_node->create_subscription<std_msgs::msg::Float64>("test_input", 10, 
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            this->messageCallback(msg);
        }
    );

    // 테스트할 메시지 생성
    auto message = std::make_shared<std_msgs::msg::Float64>();
    message->data = 42.0;  // 테스트할 데이터 값

    // 메시지 발행
    publisher->publish(*message);

    // 수신 여부를 기다리기 위해 잠시 대기
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // 수신된 데이터가 올바른지 확인
    EXPECT_EQ(received_data, 42.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}