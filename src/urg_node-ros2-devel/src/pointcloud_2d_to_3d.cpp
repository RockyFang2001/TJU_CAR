#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <chrono>
// ros2 run urg_node pointcloud_2d_to_3d  启动节点
class PointCloudNode : public rclcpp::Node
{
public:
    PointCloudNode() : Node("pointcloud_2d_to_3d")
    {
        // 创建 LaserScan 订阅者
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PointCloudNode::laserScanCallback, this, std::placeholders::_1));

        // 创建 PointCloud2 发布者
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 10);
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        laserScanToPointCloud2(*scan_msg);   
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  void laserScanToPointCloud2(const sensor_msgs::msg::LaserScan& scan_msg)
  {
    auto start = std::chrono::high_resolution_clock::now();
      sensor_msgs::msg::PointCloud2 cloud_msg;
      // 设置PointCloud2消息的头部信息
      cloud_msg.header = scan_msg.header;
      cloud_msg.height = 1;  // 无序点云
      cloud_msg.width = scan_msg.ranges.size();
      cloud_msg.is_dense = false;  // 点云中可能包含无效点
      cloud_msg.fields.resize(3);  // XYZ三个字段

      // 定义点云字段
      cloud_msg.fields[0].name = "x";
      cloud_msg.fields[0].offset = 0;
      cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[0].count = 1;

      cloud_msg.fields[1].name = "y";
      cloud_msg.fields[1].offset = 4;
      cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[1].count = 1;

      cloud_msg.fields[2].name = "z";
      cloud_msg.fields[2].offset = 8;
      cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[2].count = 1;

      cloud_msg.point_step = 12;  // 每个点占用12字节
      cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
      cloud_msg.data.resize(cloud_msg.row_step);

      // 遍历LaserScan数据并填充PointCloud2
      float angle_min = scan_msg.angle_min;
      float angle_max = scan_msg.angle_max;
      float angle_increment = scan_msg.angle_increment;
      float range_min = scan_msg.range_min;
      float range_max = scan_msg.range_max;
      size_t point_index = 0;
      for (size_t i = 0; i < scan_msg.ranges.size(); ++i)
      {

          std::ostringstream range_str;
          float range = (float)scan_msg.ranges[i];
          // printf("%f\n",range);
          if (range < range_min || range > range_max || std::isnan(range))
          // if (std::isnan(range))
          {
              // 跳过无效范围或NaN值
            //   printf("000000000000000\n");
              continue;
          }
          float angle = angle_min + i * angle_increment;

          // 计算三维坐标
          float x = range * cos(angle);
          float y = range * sin(angle);
          float z = 0.0;  // 假设激光雷达在水平平面上
        //   printf("%f,%f\n",x,y);

          if(x<=0) continue;

           // 将点坐标写入PointCloud2
          size_t offset = point_index * cloud_msg.point_step;
          *reinterpret_cast<float*>(&cloud_msg.data[offset + 0]) = x;  // x
          *reinterpret_cast<float*>(&cloud_msg.data[offset + 4]) = y;  // y
          *reinterpret_cast<float*>(&cloud_msg.data[offset + 8]) = z;  // z

          point_index++;
      }
      // 发布PointCloud2消息
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      
      // 输出执行时间（可根据需要选择单位）
      std::cout << "time: " 
                << duration.count() << " us" 
                << " (" << duration.count()/1000.0 << " ms)" 
                << std::endl;
      
      // 发布PointCloud2消息
      cloud_pub_->publish(cloud_msg);
  }

};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}