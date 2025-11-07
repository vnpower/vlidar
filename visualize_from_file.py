import cv2
import numpy as np
import math

# Tỉ lệ chuyển đổi từ mm sang pixel (mm/pixel)
# SCALE_MM_PER_PIXEL = float(input("Nhập tỉ lệ (mm/pixel, ví dụ 10): "))
SCALE_MM_PER_PIXEL = 3

def read_lidar_data(filename):
    """Đọc dữ liệu LIDAR từ file.
    
    Dữ liệu có format:
    "số thứ tự": [quality, distance, angle, time]
    trong đó:
    - số thứ tự: index của điểm (str)
    - quality: chất lượng tín hiệu (int)
    - distance: khoảng cách tính bằng mm (float)
    - angle: góc tính bằng độ (float)
    - time: thời gian (không sử dụng)
    
    Returns:
        List of tuples (distance_mm, angle_deg)
    """
    import json
    data = []
    print(f"\nReading LIDAR data from {filename}")
    
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
            
            if len(content.strip()) == 0:
                raise ValueError("File is empty")
            
            # Đảm bảo định dạng JSON hợp lệ
            content = content.strip()
            if content.endswith(',}'):  # Loại bỏ dấu phẩy trước dấu } cuối nếu có
                content = content[:-2] + '}'
            
            print("Attempting to parse JSON data...")
            try:
                lidar_data = json.loads(content)
                print(f"Successfully parsed JSON with {len(lidar_data)} points")
                
                # In ra vài điểm đầu tiên để debug
                first_few = list(lidar_data.items())[:5]
                print("\nFirst few points:")
                for index, point in first_few:
                    print(f"Point {index}: {point}")
                
                for index, point_data in lidar_data.items():
                    if not isinstance(point_data, list) or len(point_data) < 4:
                        print(f"Warning: Invalid data format for point {index}: {point_data}")
                        continue
                    
                    try:
                        # Kiểm tra giá trị None
                        if any(x is None for x in point_data[:3]):
                            print(f"Warning: Null values in point {index}: {point_data}")
                            continue
                            
                        quality = int(point_data[0])
                        distance_mm = float(point_data[1])
                        angle_deg = float(point_data[2])
                        
                        # Validate data
                        if distance_mm < 0:
                            print(f"Warning: Negative distance for point {index}: {distance_mm}mm")
                            continue
                        
                        if not 0 <= angle_deg <= 360:
                            print(f"Warning: Invalid angle for point {index}: {angle_deg}°")
                            continue
                        
                        data.append((distance_mm, angle_deg))
                    
                    except (ValueError, IndexError) as e:
                        print(f"Error parsing values for point {index}: {e}")
                        continue
            
            except json.JSONDecodeError as e:
                print(f"Error parsing JSON: {e}")
                print("Raw content:")
                print(content[:200] + "..." if len(content) > 200 else content)
                raise
    
    except Exception as e:
        print(f"Error reading file: {e}")
        raise
    
    if not data:
        print("Warning: No valid LIDAR data points were found")
    else:
        print(f"Successfully read {len(data)} valid LIDAR data points")
    
    return data

    if not data:
        print("\nAnalysis of possible issues:")
        print("1. Check if the file is in correct JSON format")
        print("2. Verify that numbers are properly formatted")
        print("3. Confirm that the file encoding is UTF-8")
        print("4. Look for any missing brackets or incorrect commas")
        raise ValueError("No valid LIDAR data found in file")
        
    return data

def main():
    print("Visualizing LIDAR data with:")
    print("- Center at (960, 540) pixels")
    print(f"- Scale: {SCALE_MM_PER_PIXEL}mm/pixel")
    print("- Background: bg1920x1080.png")
    
    # Load background image
    bg_image = cv2.imread("bg1920x1080.png")
    if bg_image is None:
        print("Warning: bg1920x1080.png not found, using black background")
        bg_image = np.zeros((1080, 1920, 3), dtype=np.uint8)

    # Đọc dữ liệu LIDAR
    try:
        lidar_data = read_lidar_data("lidar_output.txt")
    except FileNotFoundError:
        print("Error: lidar_output.txt not found")
        return
    except ValueError as e:
        print(f"Error: {e}")
        return
    
    # Tạo bản sao của background để vẽ
    display_image = bg_image.copy()
    
    # Thống kê để báo cáo phạm vi dữ liệu
    min_dist = float('inf')
    max_dist = 0
    points_drawn = 0
    
    # Vẽ tất cả các điểm
    for distance_mm, angle_deg in lidar_data:
        # Cập nhật thống kê
        min_dist = min(min_dist, distance_mm)
        max_dist = max(max_dist, distance_mm)
        
        # Chuyển từ tọa độ cực (distance_mm, angle_deg) sang tọa độ Cartesian (x_mm, y_mm)
        x_mm = distance_mm * math.sin(math.radians(angle_deg))
        y_mm = distance_mm * math.cos(math.radians(angle_deg))
        
        # Chuyển sang pixel với tỉ lệ SCALE_MM_PER_PIXEL và tâm tại (960,540)
        x_px = int(960 + x_mm / SCALE_MM_PER_PIXEL)
        y_px = int(540 - y_mm / SCALE_MM_PER_PIXEL)  # trừ vì y tăng xuống dưới trong ảnh
        
        # Vẽ điểm nếu nằm trong khung hình
        if 0 <= x_px < 1920 and 0 <= y_px < 1080:
            cv2.circle(display_image, (x_px, y_px), 2, (0, 255, 0), -1)  # Màu xanh lá
            points_drawn += 1
    
    # Vẽ điểm tâm LIDAR
    cv2.circle(display_image, (960, 540), 5, (0, 0, 255), -1)  # Màu đỏ
    
    # Vẽ chú thích thông tin
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    text_color = (255, 255, 255)  # Màu trắng
    info_lines = [
        f"Points: {points_drawn}/{len(lidar_data)} displayed",
        f"Range: {min_dist:.0f}mm to {max_dist:.0f}mm",
        f"Scale: {SCALE_MM_PER_PIXEL}mm/pixel",
        "Green: LIDAR points",
        "Red: LIDAR center (960,540)",
        "Yellow: 0° reference line"
    ]
    
    # Vẽ đường thẳng góc 0 độ (theo trục y âm)
    # Đường thẳng dài 2000mm
    line_length_px = int(2000 / SCALE_MM_PER_PIXEL)  # Chuyển 2000mm sang pixel
    start_point = (960, 540)  # Tâm LIDAR
    end_point = (960, 540 - line_length_px)  # Điểm cuối của đường thẳng
    cv2.line(display_image, start_point, end_point, (0, 255, 255), 2)  # Màu vàng
    
    y = 30
    for line in info_lines:
        cv2.putText(display_image, line, (20, y), font, font_scale, text_color, 1)
        y += 25
    
    # Vẽ tứ giác ABCD
    quad_points = [
        (943, 302),   # A
        (1750, 329),  # B
        (1750, 31),   # C
        (943, 58)     # D
    ]
    
    # Chuyển đổi các điểm từ tọa độ cực sang pixel
    quad_pixels = []
    for dist_mm, angle_deg in quad_points:
        # Chuyển từ tọa độ cực sang Cartesian (mm)
        x_mm = dist_mm * math.sin(math.radians(angle_deg))
        y_mm = dist_mm * math.cos(math.radians(angle_deg))
        
        # Chuyển sang pixel với tỉ lệ SCALE_MM_PER_PIXEL và tâm tại (960,540)
        x_px = int(960 + x_mm / SCALE_MM_PER_PIXEL)
        y_px = int(540 - y_mm / SCALE_MM_PER_PIXEL)
        quad_pixels.append((x_px, y_px))
    
    # Vẽ tứ giác bằng các đoạn thẳng màu xanh dương
    for i in range(4):
        cv2.line(display_image, quad_pixels[i], quad_pixels[(i + 1) % 4], (255, 0, 0), 2)
    
    # Ghi chữ A, B, C, D tại các đỉnh
    labels = ['A', 'B', 'C', 'D']
    for (x, y), label in zip(quad_pixels, labels):
        cv2.putText(display_image, label, (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    
    # Thêm thông tin về tứ giác vào chú thích
    info_lines.append("Blue: Quadrilateral ABCD")
    info_lines.append("A(943mm,302°) B(1750mm,329°)")
    info_lines.append("C(1750mm,31°) D(943mm,58°)")
    
    # Hiển thị ảnh
    cv2.imshow("LIDAR Data Visualization", display_image)
    print("\nVisualization ready:")
    print(f"- {points_drawn}/{len(lidar_data)} points displayed")
    print("- Quadrilateral ABCD added")
    print(f"- Distance range: {min_dist:.0f}mm to {max_dist:.0f}mm")
    print("\nPress any key to close the window")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Tùy chọn: lưu ảnh
    save = input("\nSave visualization? (y/n): ").lower().strip()
    if save.startswith('y'):
        output_file = "lidar_visualization.png"
        cv2.imwrite(output_file, display_image)
        print(f"Saved to {output_file}")

if __name__ == "__main__":
    main()