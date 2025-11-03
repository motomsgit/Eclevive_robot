#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <zed_msgs/msg/object.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using PoseStamped = geometry_msgs::msg::PoseStamped;
auto arm_message = std_msgs::msg::Int32MultiArray();
geometry_msgs::msg::Twist cmdVelMsg;

class zed_human_tracker : public rclcpp::Node {
public:
    zed_human_tracker() : Node("zed_human_tracker") {
        // ZEDで人物認識した内容をsubscribeする
        objectSubscriber_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
            "zed/zed_node/body_trk/skeletons",1,std::bind(&zed_human_tracker::objectCallback, this, std::placeholders::_1));

        arm_pose_Subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "target_arm_pose",1,std::bind(&zed_human_tracker::arm_pose_Callback, this, std::placeholders::_1));

        // TF2 Transform Listener
        recognization_frame_ = this->declare_parameter<std::string>("odom", "zed_camera_link");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        target_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        target_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*target_tf_buffer_);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&zed_human_tracker::on_timer, this));

        // Create a publisher for target_arm_position
        arm_pose_Publisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>("target_arm_pose", 10);

        // Create a publisher for target_arm_position
        current_arm_pose_Publisher_= this->create_publisher<std_msgs::msg::Int32MultiArray>("current_arm_pose", 10);

        // Create publisher for goal
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

                // Create a publisher for cmd_vel
        cmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        //
        zed_goal_status_ =this->create_publisher<std_msgs::msg::Int32>("zed_goal_status", 1);

        // Create action client for NavigateToPose action
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose"); 
        
    }

private:
    
//------------------------------------------------------------------------------------------
    void aim_goal_point(){
        geometry_msgs::msg::TransformStamped tf;
        //tryの中にあるlookupTransformでmapから目標となる人のTF（target_person）を生成するの位置を取得している
        try {
            tf = target_tf_buffer_->lookupTransform(
                "base_link", "target_person",
                tf2::TimePointZero);
           //robot_pose.translation = t.transform.translation;
            RCLCPP_INFO(this->get_logger(), "------------------- find target_person -----------------");
            RCLCPP_INFO(this->get_logger(), "target_person.x =   : %6.3f"  , target_point.x);
            RCLCPP_INFO(this->get_logger(), "target_person.y =   : %6.3f"  , target_point.y);
            RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform map to target_person: %s" , ex.what());
            return;
        }

        //発見した座標に対して差分からcmd_velに変換してpublishする
        
        if(target_point.x > 1.3){ 
            if(target_point.y > 0.3){
                cmdVelMsg.linear.x =0;
                cmdVelMsg.angular.z = tf.transform.translation.y * 1.5 + 0.8;        
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                cmdVelMsg.angular.z = tf.transform.translation.y * 0.3;        
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            else if(target_point.y < -0.3){
                cmdVelMsg.linear.x =0;
                cmdVelMsg.angular.z = tf.transform.translation.y * 1.5 - 0.8;
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                cmdVelMsg.angular.z = tf.transform.translation.y * 0.3;
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(200));  
            }            
            else{
                cmdVelMsg.linear.x =0.7;
                cmdVelMsg.linear.y = tf.transform.translation.y * 0.6;
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                cmdVelMsg.linear.x  = 0.3;
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                cmdVelMsg.linear.x  = 0.1;
                cmdVelPublisher_ ->publish(cmdVelMsg);
                rclcpp::sleep_for(std::chrono::milliseconds(300));
                cmdVelMsg.linear.x =0.0;
                cmdVelMsg.linear.x  = 0.0;
                cmdVelMsg.angular.z = 0; 
                cmdVelPublisher_ ->publish(cmdVelMsg);
            }
        }
        else if(target_point.x > 0.5){
            cmdVelMsg.linear.x  = 0.5; 
            cmdVelMsg.linear.y = tf.transform.translation.y * 0.7;
            cmdVelPublisher_ ->publish(cmdVelMsg);
            rclcpp::sleep_for(std::chrono::milliseconds(300));
            cmdVelMsg.linear.x  = 0.3;
            cmdVelPublisher_ ->publish(cmdVelMsg);
            rclcpp::sleep_for(std::chrono::milliseconds(300));
            cmdVelMsg.linear.x  = 0.1;
            cmdVelPublisher_ ->publish(cmdVelMsg);
            rclcpp::sleep_for(std::chrono::milliseconds(300));
            cmdVelMsg.linear.x =0.0;
            cmdVelMsg.linear.x  = 0.0;
            cmdVelMsg.angular.z = 0; 
            cmdVelPublisher_ ->publish(cmdVelMsg);
        }
        cmdVelMsg.linear.x  = 0.0;
        cmdVelMsg.linear.y  = 0.0;
        cmdVelMsg.angular.z = 0.0; 
        cmdVelPublisher_ ->publish(cmdVelMsg);    
        

        //arm.header.stamp = ros::Time::now();
        //arm.position[0]= theta;
        //arm.position[1]= -0.5;
        //arm.position[2]=  1.0;    
        //joint_pub.publish(arm);
        //ros::Duration(1.0).sleep();
    }    

//---------------------- ZED2iで人物認識した内容の処理を行う ---------------------------------------------------
    void objectCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr objMsg)
    {    
        using namespace std::chrono_literals;
        if (objMsg->objects.size()>0){
            //target_pointを初期化する
            //target_point.x =0.001;
            //target_point.y =0.0001;
            //target_point.z =0.0001;
            //RCLCPP_INFO(this->get_logger(), "I find %ld person", objMsg->objects.size());
            if(zed_goal_msg.data<1){//すでに目標を発見もしくはゴールしていたなら人物認識していることを除外
                zed_goal_msg.data=1;//人物認識通知
            }
            //人物が複数人見つかった場合、より遠くにいる人を目標地点とする
            for(uint i = 0 ; i < objMsg->objects.size()  ; i++ ){
                if(objMsg->objects[i].tracking_state>0){
                    for(int pose_count = 2 ; pose_count <= 7 ; pose_count++ ){//首下の位置を取得する
                        //if (isnan(objMsg->objects[i].skeleton_3d.keypoints[3].kp[2])) {
                        if (std::isnan(objMsg->objects[i].skeleton_3d.keypoints[3].kp[2])) {
                            body_pose[pose_count] =0;
                        }
                        else{
                            body_pose[pose_count] = objMsg->objects[i].skeleton_3d.keypoints[pose_count].kp[2];
                        }    
                    }

                    //RCLCPP_INFO(this->get_logger(), "body_pose[2] =   : %6.3f", body_pose[2]);
                    //RCLCPP_INFO(this->get_logger(), "body_pose[4] =   : %6.3f", body_pose[4]);
                    //RCLCPP_INFO(this->get_logger(), "body_pose[5] =   : %6.3f", body_pose[5]);
                    //RCLCPP_INFO(this->get_logger(), "body_pose[7] =   : %6.3f", body_pose[7]);

                    //肩から上に手を上げた人を見つける
                    if(body_pose[2] < body_pose[3] || body_pose[5] < body_pose[6]){
                        //RCLCPP_INFO(this->get_logger(), "I find you");
                        person_finding = true;//
                        compare_point.x =objMsg->objects[i].position[0];
                        compare_point.y =objMsg->objects[i].position[1];
                        compare_point.z =objMsg->objects[i].position[2];
                        //target_point.x =objMsg->objects[i].position[0];
                        //target_point.y =objMsg->objects[i].position[1];
                        //target_point.z =objMsg->objects[i].position[2];
                        if(compare_point.x > target_point.x){//より遠くの人をターゲットにする
                            target_point.x =objMsg->objects[i].position[0];
                            target_point.y =objMsg->objects[i].position[1];
                            target_point.z =objMsg->objects[i].position[2];
                        }
                        RCLCPP_INFO(this->get_logger(),"---------- get target point --------------");
                        RCLCPP_INFO(this->get_logger(), "target_point.x =   : %6.3f", target_point.x);
                        RCLCPP_INFO(this->get_logger(), "target_point.y =   : %6.3f", target_point.y);
                        RCLCPP_INFO(this->get_logger(), "target_point.z =   : %6.3f", target_point.z);
                        RCLCPP_INFO(this->get_logger(),"------------------------------------------");
                        zed_goal_msg.data=2;//目標発見通知
                        //ターゲットとなった人の角度に合わせてアームの向きを合わせる
                        arm_message.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message.data[1] 
                                            ,arm_message.data[2]
                                            ,arm_message.data[3]
                                            };
                        arm_pose_Publisher_->publish(arm_message);
                        //アームのグリッパーをパクパクさせる
                        rclcpp::sleep_for(600ms);
                        arm_message.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message.data[1] 
                                            ,arm_message.data[2]
                                            ,20
                                            };
                        arm_pose_Publisher_->publish(arm_message);
                        rclcpp::sleep_for(600ms);
                        arm_message.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message.data[1] 
                                            ,arm_message.data[2]
                                            ,-160
                                            };
                        arm_pose_Publisher_->publish(arm_message);
                        rclcpp::sleep_for(600ms);
                        arm_message.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message.data[1] 
                                            ,arm_message.data[2]
                                            ,0
                                            };
                        arm_pose_Publisher_->publish(arm_message);
                        arm_message.data = { int32_t(atan(target_point.y/target_point.x)*80)
                                            ,arm_message.data[1] 
                                            ,arm_message.data[2]
                                            ,-160
                                            };
                        arm_pose_Publisher_->publish(arm_message);

                        current_arm_pose_Publisher_->publish(arm_message);//最後に現状の角度を出力させる
                        rclcpp::sleep_for(500ms);
                        //RCLCPP_INFO(this->get_logger(), "taget_point.x= %5.8f",target_point.x);
                        break;   
                    }
                    else{
                        person_finding = false;
                    }//end else
                }//end if
            }//end for
            if(target_point.x > 0.5){
                //目標となる人のTF（target_person）を生成する
                static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = "zed_camera_link"; // 親フレーム
                transformStamped.child_frame_id  = "target_person"; // 子フレーム
                transformStamped.transform.translation = target_point;
                double offset =0.6;
                transformStamped.transform.translation.x = target_point.x - offset;
                tf2::Quaternion quaternion;
                quaternion.setRPY(0, 0, (atan(target_point.y/target_point.x)/M_PI));
                transformStamped.transform.rotation.x = quaternion.x();
                transformStamped.transform.rotation.y = quaternion.y();
                transformStamped.transform.rotation.z = quaternion.z();
                transformStamped.transform.rotation.w = quaternion.w();
                
                // tfメッセージをbroadcast
                static_tf_broadcaster_->sendTransform(transformStamped);

                aim_goal_point();
            }
            else if(target_point.x <= 0.5 ){
                zed_goal_msg.data=3;//ゴールしたことを通知
                zed_goal_status_->publish(zed_goal_msg);
            }
        }//end if (objMsg->objects.size()>0)
        else{
        //callbackしている時点で何らか認識しているため、ここでは特に何もしない。  
        }
        //目標地点を初期化する
        target_point.x =0.001;
        target_point.y =0.0001;
        target_point.z =0.0001;
    } //end objectCallback
//-------------------現状のアーム角度を取得する -----------------------------------------------
void arm_pose_Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msgin)
    {
        arm_message.data ={  msgin->data[0]
                            ,msgin->data[1]
                            ,msgin->data[2]
                            ,msgin->data[3]
                            };
    }
//------------------1秒ごとにodomとZED2カメラの位置を取得する--------------------------------------
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped t;
        //tryの中にあるlookupTransformでカメラの位置を取得している
        try {
            t = tf_buffer_->lookupTransform(
                "odom", "zed_camera_link",
                tf2::TimePointZero);
           //robot_pose.translation = t.transform.translation;
            robot_pose.transform      = t.transform;  
            robot_pose_yaw  = t.transform.rotation.z; 
            /*
            RCLCPP_INFO(this->get_logger(), "robot_pose.x =   : %6.3f", robot_pose.transform.translation.x);
            RCLCPP_INFO(this->get_logger(), "robot_pose.y =   : %6.3f", robot_pose.transform.translation.y);
            RCLCPP_INFO(this->get_logger(), "robot_pose.z =   : %6.3f", robot_pose.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "robot_rotate.z =   : %6.3f", robot_pose.transform.rotation.z);
            RCLCPP_INFO(this->get_logger(), "robot_rotate.w =   : %6.3f", robot_pose.transform.rotation.w);
            RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");
            */

        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform odom to zed_camera_link: %s" , ex.what());
            return;
        }
        if(pre_zed_goal_msg.data!=zed_goal_msg.data){
            zed_goal_status_->publish(zed_goal_msg);//ゴール認識状態を通知
            pre_zed_goal_msg.data=zed_goal_msg.data;
        }
    }

    //------------action_client_のfeedbackを生成-----------------------------------------------------------
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
    }
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;  
    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr objectSubscriber_; 
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr  arm_pose_Subscriber_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> target_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> target_tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmdVelPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr  arm_pose_Publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr  current_arm_pose_Publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr zed_goal_status_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::string recognization_frame_;
    bool person_finding;                        //人物認識できた際のフラグ
    geometry_msgs::msg::Vector3 compare_point;  //手を上げた人の位置取得用(それぞれの人の位置) 
    geometry_msgs::msg::Vector3 target_point;   //手を上げた人の位置取得用(比較して選定した人の最終目標位置)
    float body_pose[20];             
    std_msgs::msg::Int32 zed_goal_msg;  
    std_msgs::msg::Int32 pre_zed_goal_msg;
    geometry_msgs::msg::TransformStamped robot_pose;  //odom→zed_camera_link間となるロボット位置取得用
    double robot_pose_yaw=0.0001;              //zed2による物体認識から目標位置の角度取得用
    double leave_distance=0.60;                //1歩下がった位置にゴールを指定するため、1歩の距離を指定する

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zed_human_tracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}