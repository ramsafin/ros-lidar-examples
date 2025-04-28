#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
from typing import List, Optional, Tuple

import rospy
from sensor_msgs.msg import LaserScan

class ObstacleMovementDetector:
  def __init__(self):
    rospy.init_node('obstacle_movement_detector')

    # Параметры
    self.prev_distance: Optional[float] = None
    self.threshold = 0.02  # Порог в метрах
    self.fov_degrees = 30  # Сектор анализа (±30°)

    # Здесь подпишитесь на измерения LiDAR ...

  def scan_callback(self, msg: LaserScan) -> None:
    # Получение данных из сектора
    frontal_ranges = self.get_frontal_sector(msg.ranges, msg.angle_min, msg.angle_increment)

    if not frontal_ranges:
      rospy.logwarn("Не удалось получить сканы фронального сектора!")
      return

    # Минимальное расстояние
    min_distance = np.min(frontal_ranges)

    # Определение движения
    movement, delta = self.detect_movement(min_distance)

    # Логирование результата
    self.log_movement(movement, min_distance, delta)

  def get_frontal_sector(self, ranges: List[float], angle_min: float, angle_increment: float) -> List[float]:
    """
    Возвращает список расстояний в заданном секторе.
    Исключает значения inf (бесконечности) и nan.
    """
    fov_rad = np.radians(self.fov_degrees)
    frontal_ranges = []

    # Ваш код здесь:
    # 1. Пройти по всем измерениям (ranges)
    # 2. Для каждого измерения вычислить угол
    # 3. Если угол попадает в сектор ±fov_rad и значение валидно - добавить в frontal_ranges

    return frontal_ranges

  def detect_movement(self, current_distance: float) -> Tuple[str, float]:
    """
    Определяет тип движения относительно препятствия.
    Возвращает тип движения и изменение расстояния.
    """
    movement = "stable"
    delta = 0.0

    if self.prev_distance is not None:
      delta = self.prev_distance - current_distance

      # 1. Сравнить delta с threshold
      # 2. Определить movement ("approaching", "receding" или "stable")

    self.prev_distance = current_distance
    return movement, delta

  def log_movement(self, movement: str, distance: float, delta: float) -> None:
    """
    Логирует результат с цветовой подсветкой для терминала.
    Возможно работающий код...
    """
    colors = {
      "approaching": "\033[91m",  # Красный
      "receding": "\033[92m",     # Зелёный
      "stable": "\033[94m"        # Синий
    }
    reset = "\033[0m"

    # Обновляем счётчики
    if movement == "approaching":
      self.approach_count += 1
    elif movement == "receding":
      self.recede_count += 1
    else:
      self.stable_count += 1

    rospy.loginfo(
      f"{colors[movement]}{movement.upper()}{reset} | "
      f"Distance: {distance:.2f}m | "
      f"Change: {delta:.3f}m"
    )

def main() -> None:
  detector = ObstacleMovementDetector()
  rospy.spin()

if __name__ == '__main__':
  main()
