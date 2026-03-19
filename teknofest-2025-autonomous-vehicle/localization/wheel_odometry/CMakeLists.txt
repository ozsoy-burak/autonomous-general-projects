cmake_minimum_required(VERSION 3.0.2)
project(wheel_odom)

# 1. Gerekli Paketleri Bul
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  smart_can_msgs
)

# 2. Paket Dışa Aktarımı (Başka paketler seni kullanmak isterse)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wheel_odom
#  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf smart_can_msgs
#  DEPENDS system_lib
)

# 3. Include (Başlık) Klasörleri
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 4. C++ Kodunu Çalıştırılabilir (Executable) Dosyaya Çevir
add_executable(ackermann_odometry_publisher src/ackermann_odometry_publisher.cpp)

# 5. Kütüphaneleri Linkle
target_link_libraries(ackermann_odometry_publisher
  ${catkin_LIBRARIES}
)
