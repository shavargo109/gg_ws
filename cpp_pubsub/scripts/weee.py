#! /usr/bin/env python3
import tf_transformations

# 将欧拉角转换为四元数（roll, pitch, yaw）
roll = 0
pitch = 0
yaw = 3
q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
print(q[3])
# 将四元素转换成欧拉角
# euler = tf_transformations.euler_from_quaternion([x, y, z, w])
