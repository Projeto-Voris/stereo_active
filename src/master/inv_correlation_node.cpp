#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include "stereo_active/srv/move_motor.hpp"
#include <mutex>
#include <vector>
#include <deque>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cstdlib>

/*
tarefas realizadas por inv_correlation_node.py que devem ser realizadas agora por esse cpp:
    callbacks das imagens, subscribers e publishers, trigger/lasers services
    get_images_srv
*/

namespace stereo_active
{
class InvCorrelationNode : public rclcpp::Node{

public:
    explicit InvCorrelationNode(const rclcpp::NodeOptions & options) : Node("inverse_correlation_node", options){
        RCLCPP_INFO(this->get_logger(), "InvCorrelationNode.cpp has been started");
        
        count_= 1;

        // parametros
        this->declare_parameter<int>("num_images",10);
        this->declare_parameter<int>("steps", 20);
        num_images_ = this->get_parameter("num_images").as_int();
        steps_ = this->get_parameter("steps").as_int();

        //publisher
        handshake_images_pub_ = this->create_publisher<std_msgs::msg::Int16>("handshake_images", 10);

        // subscribers
        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>("left/image", 10, std::bind(&InvCorrelationNode::left_image_cb, this, std::placeholders::_1));
        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>("right/image", 10, std::bind(&InvCorrelationNode::right_image_cb, this, std::placeholders::_1));

        // services
        service_request_= false;
        perform_correl_ = false;
        cb_group_srv_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_trigger_client_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_laser_client_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_motor_client_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        srv_ = this->create_service<std_srvs::srv::SetBool>(
            "correlation_process", 
            std::bind(&InvCorrelationNode::get_images_srv, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            cb_group_srv_
        );
        save_im_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "save_images_ssd",
            std::bind(&InvCorrelationNode::save_images_ssd_srv, this, std::placeholders::_1, std::placeholders::_2)
        );

        gpio_client_ = this->create_client<std_srvs::srv::Trigger>(
            "trigger",
            rmw_qos_profile_services_default,
            cb_group_trigger_client_
        );

        laser_client_ = this->create_client<std_srvs::srv::SetBool>(
            "laser",
            rmw_qos_profile_services_default,
            cb_group_laser_client_
        );
        motor_client_ = this->create_client<stereo_active::srv::MoveMotor>(
            "move_motor", 
            rmw_qos_profile_services_default,
            cb_group_motor_client_
        );
    }

private:
    
    //atributos
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;

    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> left_queue_; 
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> right_queue_;

    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> captured_left_images_;
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> captured_right_images_;

    std::mutex mutex_;
    uint8_t count_;
    bool service_request_;
    bool perform_correl_;
    int num_images_;
    int steps_;
    
    // services
    rclcpp::CallbackGroup::SharedPtr cb_group_srv_;
    rclcpp::CallbackGroup::SharedPtr cb_group_trigger_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_laser_client_;
    rclcpp::CallbackGroup::SharedPtr cb_group_motor_client_;
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_im_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gpio_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr laser_client_;
    rclcpp::Client<stereo_active::srv::MoveMotor>::SharedPtr motor_client_;

    //publisher
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr handshake_images_pub_;

    void left_image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
        { // mutex apenas para acesso as filas compartilhadas
        std::lock_guard<std::mutex> lock(mutex_);
        left_queue_.push_back(msg);
        }
        match_images();

    }

    void right_image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
        {
        std::lock_guard<std::mutex> lock(mutex_);
        right_queue_.push_back(msg);
        }
        match_images();
    }

    void match_images(){
        sensor_msgs::msg::Image::ConstSharedPtr right_ptr;
        sensor_msgs::msg::Image::ConstSharedPtr left_ptr;

        while (true){
            { // mutex
                std::lock_guard<std::mutex> lock(mutex_);

                if(!(left_queue_.empty()) && !(right_queue_.empty() )){ // verifica se ha pelo menos uma imagem em cada fila
                    left_ptr = left_queue_.front();
                    right_ptr = right_queue_.front();
                    left_queue_.pop_front();
                    right_queue_.pop_front();
                }
                else{
                    break; // sai do loop
                }
            } // mutex

            this->stereo_images_accumulator(left_ptr, right_ptr);
        }

    }

    void stereo_images_accumulator(const sensor_msgs::msg::Image::ConstSharedPtr left_img_ptr, const sensor_msgs::msg::Image::ConstSharedPtr right_img_ptr){
        
        if(service_request_ == false){
            return;
        }

        captured_left_images_.push_back(left_img_ptr);
        captured_right_images_.push_back(right_img_ptr);

        if (count_ < num_images_){
            count_++;
        }else if (count_ == num_images_){
            RCLCPP_INFO(this->get_logger(),"Captured stereo images: %d/%d", count_, num_images_);
            this->save_images();
            service_request_=false;
        }
    }

    void save_images(){ // salva imagens num arquivo temporario (ja faz parte do pos process)

        std::string base_path = "/dev/shm/stereo_active/";
        std::filesystem::create_directories(base_path + "left");
        std::filesystem::create_directories(base_path + "right");

        for(u_int8_t i=0; i<num_images_; i++){
            try{
                auto cv_ptr_left = cv_bridge::toCvShare(captured_left_images_[i], "mono8"); // preto e branco
                auto cv_ptr_right = cv_bridge::toCvShare(captured_right_images_[i], "mono8");

                char left_filename[256], right_filename[256];
                snprintf(left_filename, sizeof(left_filename), "%sleft/L%02d.png", base_path.c_str(), i + 1);
                snprintf(right_filename, sizeof(right_filename), "%sright/R%02d.png", base_path.c_str(), i + 1);

                cv::imwrite(left_filename, cv_ptr_left->image);
                cv::imwrite(right_filename, cv_ptr_right->image);

            }catch(cv_bridge::Exception& e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s error", e.what());
                return;
            }
        }

        auto num_images = std_msgs::msg::Int16();
        if (perform_correl_) { //identifica a realizacao ou nao do processamento via o sinal enviado do numero de imagens
            num_images.data = num_images_;  // processa
        } else {
            num_images.data = -num_images_; // nao processa
        }
        RCLCPP_INFO(this->get_logger(), "Images saved successfully");
        handshake_images_pub_->publish(num_images);

        captured_left_images_.clear();
        captured_right_images_.clear();

    }

    void save_images_ssd_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        
        const char* home_dir = std::getenv("HOME");
        if (home_dir == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Error on finding HOME directory.");
            response->success = false;
            response->message = "HOME not found";
            return;
        }

        std::string ram_path = "/dev/shm/stereo_active/";
        std::string destiny_path = std::string(home_dir) + "/Pictures/stereo_active_backup";
        RCLCPP_INFO(this->get_logger(), "Saving Images ...");

        try{
            std::filesystem::create_directories(destiny_path);
            std::filesystem::copy(ram_path, destiny_path, std::filesystem::copy_options::recursive | std::filesystem::copy_options::overwrite_existing);
            response->success = true;
            response->message = "Images saved";
            RCLCPP_INFO(this->get_logger(), "Images have been saved");
        }
        catch(const std::filesystem::filesystem_error& e){
            RCLCPP_ERROR(this->get_logger(), "Copy archives error: %s", e.what());
            response->success = false;
            response->message = "Copy archives error";
        }
    }

    void get_images_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        
        count_=1;
        service_request_=true;
        perform_correl_ = request->data;
        captured_left_images_.clear();
        captured_right_images_.clear();

        float angulo_motor = (steps_ / 2048.0f) * 360.0f;

        // call laser service
        auto laser_request = std::make_shared<std_srvs::srv::SetBool::Request>();
        laser_request->data = true;
        auto future_laser = laser_client_->async_send_request(laser_request); // guarda o pedido do laser_request

        if (future_laser.wait_for(std::chrono::seconds(5)) == std::future_status::ready){ // espera por 5sec o status do laser_request

            auto result_laser = future_laser.get();
            if (result_laser->success){ // caso o laser tenha sido ligado, roda o codigo

                RCLCPP_INFO(this->get_logger(), "Laser turned on!");
                auto move_motor_request = std::make_shared<stereo_active::srv::MoveMotor::Request>();

                for(uint8_t i=0; i < num_images_; i++){

                    move_motor_request->angle = angulo_motor;
                    auto future_move_motor = motor_client_->async_send_request(move_motor_request); 
                    if (future_move_motor.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready){ // espera que o motor acabe de se mover

                        auto result_move_motor = future_move_motor.get();
                        if (result_move_motor->success){ 
                            
                            auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
                            auto future_trigger = gpio_client_->async_send_request(trigger_request);
                            if (future_trigger.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready){

                                auto result_trigger = future_trigger.get();

                                if(result_trigger->success){
                                    rclcpp::sleep_for(std::chrono::milliseconds(10)); // espera o time exposition da foto
                                }

                            }else{
                                RCLCPP_ERROR(this->get_logger(), "Trigger service call timed out!");
                                response->success = false;
                                response->message = "Trigger failed";
                                return;
                            }
                        }
                    }else{
                        RCLCPP_ERROR(this->get_logger(), "Move_motor service call failed or timed out!");
                        response->success = false;
                        response->message = "Move_motor failed";
                        return;
                    }
                }
            }else{
                RCLCPP_ERROR(this->get_logger(), "Laser service call failed");
                response->success = false;
                response->message = "Laser failed";
                return;
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Laser service call timed out!");
            response->success = false;
            response->message = "Laser failed";
            return;
        }

        //turn off laser and return motor to initial position
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        laser_request->data = false;
        future_laser = laser_client_->async_send_request(laser_request);

        if (future_laser.wait_for(std::chrono::seconds(5)) == std::future_status::ready){ // espera por 5sec o status do laser_request

           auto result_laser = future_laser.get();
           if (result_laser->success){
                auto move_motor_return_request = std::make_shared<stereo_active::srv::MoveMotor::Request>();
                move_motor_return_request->angle = -(steps_*(num_images_)/2048.0f)*360.0f;
                auto future_move_motor_return = motor_client_->async_send_request(move_motor_return_request);
                if (future_move_motor_return.wait_for(std::chrono::seconds(5)) == std::future_status::ready){ 
                     auto result_move_motor_return = future_move_motor_return.get();
                        if (result_move_motor_return->success){ 
                            RCLCPP_INFO(this->get_logger(), "Varredura completa");
                            response->success = true;
                            response->message = "Varredura completa";
                        }
                }else{
                    RCLCPP_ERROR(this->get_logger(), "Move_motor_return service call timed out!");
                    response->success = false;
                    response->message = "Motor failed";
                    return;
                }
           }else{
                RCLCPP_ERROR(this->get_logger(), "Laser service call failed!");
                response->success = false;
                response->message = "Laser failed";
                return;
           }

        }else{
            RCLCPP_ERROR(this->get_logger(), "Laser service call timed out!");
            response->success = false;
            response->message = "Laser failed";
            return;
        }

    }
};

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_active::InvCorrelationNode)