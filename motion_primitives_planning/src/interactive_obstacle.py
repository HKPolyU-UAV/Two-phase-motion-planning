#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, Marker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback

def make_box( msg ):
    # 创建立方体 Marker
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.scale.x = 10  # 圆柱底面直径
    marker.scale.y = 10  # 圆柱底面直径（需与x相同）
    marker.scale.z = 20  # 圆柱高度（可单独调整）
    marker.color.r = 0.7
    marker.color.g = 0.3
    marker.color.b = 0.3
    marker.color.a = 0.8
    return marker

def make_box_control( msg ):
    # 创建控制交互的控件
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( make_box(msg) )
    msg.controls.append( control )
    return control

def process_feedback(feedback):
    # 反馈处理（当用户操作标记时调用）
    rospy.loginfo(f"Obstacle moved to: {feedback.pose.position}")

if __name__ == "__main__":
    rospy.init_node("interactive_obstacle")
    
    # 创建 Interactive Marker 服务器
    server = InteractiveMarkerServer("obstacle_markers")
    
    # 创建一个 Interactive Marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"  # 设置坐标系（需与 RViz 固定帧一致）
    int_marker.name = "obstacle_1"
    int_marker.description = "Drag to move obstacle"
    int_marker.scale = 100.0  # 标记的缩放比例

    # 设置标记的初始位置
    int_marker.pose.position.x = 1.0
    int_marker.pose.position.y = 2.0
    int_marker.pose.orientation.w = 1.0

    # 添加移动控制（允许沿 XY 平面拖拽）
    control = make_box_control(int_marker)
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.orientation.w = 1.0
    control.orientation.x = 0
    control.orientation.y = 1.0
    control.orientation.z = 0
    int_marker.controls.append(control)

    # 将 Interactive Marker 添加到服务器
    server.insert(int_marker, process_feedback)
    
    # 应用所有更改
    server.applyChanges()
    
    rospy.spin()
