// Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
#include <cstring>
#include <iostream>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <rcl/publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw_fastrtps_shared_cpp/custom_publisher_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("verify_local_transport");
 auto pub=node->create_publisher<sensor_msgs::msg::Imu>("/livox/imu",rclcpp::QoS(1000).reliable());
 auto handle=rcl_publisher_get_rmw_handle(pub->get_publisher_handle().get());
 if(std::strcmp(handle->implementation_identifier,"rmw_fastrtps_cpp"))return 2;
 auto info=static_cast<CustomPublisherInfo*>(handle->data);
 auto qos=info->data_writer_->get_publisher()->get_participant()->get_qos();
 bool shm=false,udp=false;
 std::cout<<"builtin="<<qos.transport().use_builtin_transports<<"\n";
 for(auto & descriptor:qos.transport().user_transports) {
  if(auto d=std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(descriptor)) {
   std::cout<<"SHM segment="<<d->segment_size()<<" queue="<<d->port_queue_capacity()<<"\n";
   shm=shm || d->segment_size()==134217728 && d->port_queue_capacity()==4096;
  }
  if(auto d=std::dynamic_pointer_cast<eprosima::fastdds::rtps::UDPv4TransportDescriptor>(descriptor)) {
   std::cout<<"UDPv4";for(auto &ip:d->interfaceWhiteList)std::cout<<" "<<ip;std::cout<<"\n";
   udp=udp || d->interfaceWhiteList.size()==1 && d->interfaceWhiteList[0]=="127.0.0.1";
  }
 }
 pub.reset();node.reset();rclcpp::shutdown();return shm&&udp&&!qos.transport().use_builtin_transports?0:1;
}
