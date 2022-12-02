#include "kf_node.h"

KFNode::KFNode(const std::string & node_name, const std::string & node_namespace) : rclcpp::Node(node_name, node_namespace), writer(2, 2048), reader(1, 2048) {

  // Custom code here to initialize BRAM and xkalmanfilterkernel
  // ...
  //Test
  //      timer_ = this->create_wall_timer(500ms, std::bind(&KFNode::timer_callback, this));
  //End test
  
  int result = XKalmanfilterkernel_Initialize(&kf_kernel,"KalmanFilterKernel");

  // Initialize subscribers
  pos_meas_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/sensor/pos_measurement", 
    10, 
    std::bind(&KFNode::pos_meas_callback, this, std::placeholders::_1)
  );
  control_input_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/controller/control_input", 
    10, 
    std::bind(&KFNode::control_input_callback, this, std::placeholders::_1)
  );

  // Initialize publishers
  pos_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/kf/pos_est", 10);

}

KFNode::~KFNode() {
  // Custom code here to close BRAM and xkalmanfilterkernel
  // ...
}

//TEST
    void KFNode::timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "TEST");
    }
//END TEST

void KFNode::pos_meas_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  pos_t pos_meas;
  pos_meas.x = msg->data[0];
  pos_meas.y = msg->data[1];
  pos_meas.z = msg->data[2];
  pos_meas_queue.push(pos_meas);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  // ...
  //TEST
  //RCLCPP_INFO(this->get_logger(), "Received!!!"); 
  //END TEST
  if((pos_meas_queue.empty() == 0) && (control_input_queue.empty() == 0))
  {
XKalmanfilterkernel_Set_q(&kf_kernel, 0.5);
XKalmanfilterkernel_Set_r(&kf_kernel, 0.5);
  	acc_t control_input;
  	uint32_t *bram0_addr = writer.GetPointer();
  	uint32_t *bram1_addr = reader.GetPointer();
  	pos_meas =  pos_meas_queue.front();
  	control_input = control_input_queue.front();
  	pos_meas_queue.pop();
  	control_input_queue.pop();
  	bram0_addr[0] = *(u32 *)(&pos_meas.x);
  	bram0_addr[1] = *(u32 *)(&pos_meas.y);
  	bram0_addr[2] = *(u32 *)(&pos_meas.z);
  	bram0_addr[3] = *(u32 *)(&control_input.ax);
  	bram0_addr[4] = *(u32 *)(&control_input.ay);
  	bram0_addr[5] = *(u32 *)(&control_input.az);
RCLCPP_INFO(this->get_logger(), "Bram data: '%d'", bram0_addr[0]);
RCLCPP_INFO(this->get_logger(), "Bram data: '%d'", bram0_addr[1]);
RCLCPP_INFO(this->get_logger(), "Bram data: '%d'", bram0_addr[2]);
RCLCPP_INFO(this->get_logger(), "Bram data: '%d'", bram0_addr[3]);
RCLCPP_INFO(this->get_logger(), "Bram data: '%d'", bram0_addr[4]);
RCLCPP_INFO(this->get_logger(), "Bram data: '%d'", bram0_addr[5]); 	
XKalmanfilterkernel_Start(&kf_kernel);
  	while(!XKalmanfilterkernel_IsDone(&kf_kernel));
  	pos_meas.x = *(float*)(&bram1_addr[0]);
  	pos_meas.y =  *(float*)(&bram1_addr[1]);
  	pos_meas.z =  *(float*)(&bram1_addr[2]);
  	publish_pos_est(pos_meas);
  }
}

void KFNode::control_input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  acc_t control_input;
  control_input.ax = msg->data[0];
  control_input.ay = msg->data[1];
  control_input.az = msg->data[2];
  control_input_queue.push(control_input);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  // ...
/*
  if((pos_meas_queue.empty() == 0) && (control_input_queue.empty() == 0))
  {
XKalmanfilterkernel_Set_q(&kf_kernel, 0.5);
XKalmanfilterkernel_Set_r(&kf_kernel, 0.5);
  	pos_t pos_meas;
  	uint32_t *bram0_addr = writer.GetPointer();
  	uint32_t *bram1_addr = reader.GetPointer();
  	pos_meas =  pos_meas_queue.front();
  	control_input = control_input_queue.front();
  	pos_meas_queue.pop();
  	control_input_queue.pop();
  	bram0_addr[0] = (u32)pos_meas.x;
  	bram0_addr[1] = (u32)pos_meas.y;
  	bram0_addr[2] = (u32)pos_meas.z;
  	bram0_addr[3] = (u32)control_input.ax;
  	bram0_addr[4] = (u32)control_input.ay;
  	bram0_addr[5] = (u32)control_input.az;
//RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", pos_meas.y);
//RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", pos_meas.z);
//RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", control_input.ax);
//RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", control_input.ay);
//RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", control_input.az);

  	XKalmanfilterkernel_Start(&kf_kernel);
  	while(!XKalmanfilterkernel_IsDone(&kf_kernel));
  	pos_meas.x = bram1_addr[0];
  	pos_meas.y = bram1_addr[1];
  	pos_meas.z = bram1_addr[2];

RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", pos_meas.x);
  	publish_pos_est(pos_meas);
  }*/

}

void KFNode::publish_pos_est(pos_t pos_est) {
  geometry_msgs::msg::PoseStamped pos_est_msg;
  pos_est_msg.header.stamp = this->get_clock()->now();
  pos_est_msg.header.frame_id = "world";
  pos_est_msg.pose.orientation.w = 1.0;
  pos_est_msg.pose.orientation.x = 0.;
  pos_est_msg.pose.orientation.y = 0.;
  pos_est_msg.pose.orientation.z = 0.;
  pos_est_msg.pose.position.x = pos_est.x;
  pos_est_msg.pose.position.y = pos_est.y;
  pos_est_msg.pose.position.z = pos_est.z;
  pos_est_pub_->publish(pos_est_msg);
  //TEST
  RCLCPP_INFO(this->get_logger(), "DONE!!!");
  //END TEST
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
    auto node = std::make_shared<KFNode>();

    rclcpp::spin(node);

	rclcpp::shutdown();
  return 0;
}
