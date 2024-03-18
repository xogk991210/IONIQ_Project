# -*- coding: utf-8 -*-
# 작성자: LABMAS 김태하
# 연락처: +82 010-3374-2647
# 작성일: 2024-3-18

import rclpy  # ROS 2 Python 클라이언트 라이브러리
import math  # 수학 함수 사용을 위한 모듈
import numpy as np  # 다차원 배열 연산을 위한 모듈
import matplotlib.pyplot as plt  # 그래프 관련 모듈
from sensor_msgs.msg import LaserScan  # LaserScan 메시지 타입
from rclpy.qos import qos_profile_sensor_data, QoSProfile  # 토픽 품질 프로파일링 클래스
from sklearn.linear_model import (
    LinearRegression,
    RANSACRegressor,
)  # 클러스터링 라이브러리
from rclpy.node import Node  # ROS 2 노드 클래스


class MinimalSubscriber(Node):
    """
    레이저 스캔 메시지를 구독하여 장애물 회피를 위한 노드 클래스.
    """

    def __init__(self):
        """
        노드 초기화 및 퍼블리셔, 서브스크라이버 생성.
        """
        super().__init__("lidar_filters")  # 노드 이름 설정
        self.subscription = self.create_subscription(  # 서브스크라이버 생성
            LaserScan, "/scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )
        self.ransac = RANSACRegressor(
            LinearRegression(),
            residual_threshold=5.0,  # 잔차 임계값 설정
            random_state=0,
        )

    def transform_lidar_to_xy(self, laser_scan_msg):
        """
        lidar정보를 구독하여 x, y좌표계로 변환하는 함수
        """
        xy_points = []
        angle = laser_scan_msg.angle_min
        for range_value in laser_scan_msg.ranges:
            if (
                range_value < laser_scan_msg.range_min
                or range_value > laser_scan_msg.range_max
            ):
                angle += laser_scan_msg.angle_increment
                continue

            x = range_value * math.cos(angle)
            y = range_value * math.sin(angle)

            xy_points.append((x, y))

            angle += laser_scan_msg.angle_increment

        return xy_points

    def view(self, inlier_mask, outlier_mask, x, y):
        plt.scatter(
            x[inlier_mask], y[inlier_mask], color="blue", marker="o", label="Inliers"
        )
        plt.scatter(
            x[outlier_mask], y[outlier_mask], color="red", marker="x", label="Outliers"
        )
        plt.plot(
            x, self.ransac.predict(x), color="black"
        )  # RANSAC로 추정된 선형 회귀 모델
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend(loc="upper left")
        plt.show()

    def point_filter(self, xy_points):
        xy = np.array(xy_points)
        x = xy[:, 0].reshape(-1, 1)
        y = xy[:, 1]

        self.ransac.fit(x, y)
        inlier_mask = self.ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        self.view(inlier_mask, outlier_mask, x, y)

    def scan_callback(self, sub_msg):
        """ """
        xy_points = self.transform_lidar_to_xy(sub_msg)
        self.point_filter(xy_points)


def main(args=None):
    """
    노드 실행을 위한 메인 함수.

    :param args: 명령행 인자
    """
    rclpy.init(args=args)  # ROS 클라이언트 라이브러리 초기화
    qos = QoSProfile(depth=10)

    minimal_subscriber = MinimalSubscriber()  # 노드 인스턴스 생성
    try:
        rclpy.spin(minimal_subscriber)  # 노드 실행
    except:
        minimal_subscriber.get_logger().info(" 오류 발생 ")  # 예외 처리
    finally:
        minimal_subscriber.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS 클라이언트 라이브러리 종료


if __name__ == "__main__":
    main()
