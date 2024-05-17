/*********************************************************************
 *
 * 软件许可协议 (BSD License)
 *
 *  版权所有 (c) 2008, 2013, Willow Garage, Inc.
 *  保留所有权利.
 *
 *  源代码和二进制形式的再分发和使用, 无论是否修改,
 *  都必须符合以下条件:
 *
 *   * 源代码的再分发必须保留上述版权声明、这个条件列表
 *     和以下免责声明.
 *   * 二进制形式的再分发必须在随分发提供的文档和/或其他
 *     材料中保留上述版权声明、这个条件列表和以下免责声明.
 *   * 未经事先书面许可, 不得使用 Willow Garage, Inc. 的名称
 *     或其贡献者的名称来为派生的产品背书或推广.
 *
 *  该软件由版权所有者和贡献者按“原样”提供, 不提供任何明示或
 *  暗示的保证, 包括但不限于对适销性和特定用途适用性的暗示
 *  保证. 无论在任何情况下, 无论是因合同、严格责任还是侵权行为
 *  (包括疏忽或其他原因), 版权持有人或贡献者均不对因使用本软件
 *  而产生的任何直接、间接、附带、特殊、惩罚性或后果性损害
 *  (包括但不限于采购替代商品或服务; 使用损失、数据丢失或利润
 *  损失; 或业务中断) 负责, 即使已告知可能会发生此类损害.
 *
 * 作者: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_
#define NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_

#include <vector>
#include <list>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{
  /**
   * @class ObservationBuffer
   * @brief 从传感器接收点云数据，将其转换为所需的坐标系并存储
   */
  class ObservationBuffer
  {
  public:
    /**
     * @brief 构造一个观测缓冲区
     * @param topic_name 观测主题的名称，用作错误和警告消息的标识符
     * @param observation_keep_time 定义观测数据保留的时间(秒)，0表示只保留最新的观测数据
     * @param expected_update_rate 该缓冲区预计的更新频率，0表示没有限制
     * @param min_obstacle_height 被视为合法障碍物的最小高度
     * @param max_obstacle_height 被视为合法障碍物的最大高度
     * @param obstacle_max_range 传感器插入障碍物时的最大信任范围
     * @param obstacle_min_range 传感器插入障碍物时的最小信任范围
     * @param raytrace_max_range 传感器清除空间时的最大射线追踪范围
     * @param raytrace_min_range 传感器清除空间时的最小射线追踪范围
     * @param tf2_buffer tf2 缓冲区的引用
     * @param global_frame 要将点云转换到的全局坐标系
     * @param sensor_frame 传感器的原始坐标系，可以留空以从消息中读取
     * @param tf_tolerance 设置新全局坐标系时等待变换可用的时间
     */
    ObservationBuffer(
        const nav2_util::LifecycleNode::WeakPtr &parent,
        std::string topic_name,
        double observation_keep_time,
        double expected_update_rate,
        double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
        double obstacle_min_range,
        double raytrace_max_range, double raytrace_min_range, tf2_ros::Buffer &tf2_buffer,
        std::string global_frame,
        std::string sensor_frame,
        tf2::Duration tf_tolerance);

    /**
     * @brief 析构函数，进行清理
     */
    ~ObservationBuffer();

    /**
     * @brief 将点云转换为全局坐标系并缓冲它
     * <b>注意: 用户有责任确保变换可用... 例如他们应该使用 MessageNotifier</b>
     * @param cloud 要缓冲的点云
     */
    void bufferCloud(const sensor_msgs::msg::PointCloud2 &cloud);

    /**
     * @brief 将所有当前观测的副本推到传入向量的末尾
     * @param observations 要填充的向量
     */
    void getObservations(std::vector<Observation> &observations);

    /**
     * @brief 检查观测缓冲区是否以预期的频率更新
     * @return 如果以预期的频率更新则返回 true，否则返回 false
     */
    bool isCurrent() const;

    /**
     * @brief 锁定观测缓冲区
     */
    inline void lock()
    {
      lock_.lock();
    }

    /**
     * @brief 解锁观测缓冲区
     */
    inline void unlock()
    {
      lock_.unlock();
    }

    /**
     * @brief 重置最后更新的时间戳
     */
    void resetLastUpdated();

  private:
    /**
     * @brief 从缓冲区列表中移除任何陈旧的观测数据
     */
    void purgeStaleObservations();

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};
    tf2_ros::Buffer &tf2_buffer_;
    const rclcpp::Duration observation_keep_time_;
    const rclcpp::Duration expected_update_rate_;
    rclcpp::Time last_updated_;
    std::string global_frame_;
    std::string sensor_frame_;
    std::list<Observation> observation_list_;
    std::string topic_name_;
    double min_obstacle_height_, max_obstacle_height_;
    std::recursive_mutex lock_; ///< @brief 用于安全访问回调数据的锁
    double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
    tf2::Duration tf_tolerance_;
  };
} // namespace nav2_costmap_2d
#endif // NAV2_COSTMAP_2D__OBSERVATION_BUFFER_HPP_
