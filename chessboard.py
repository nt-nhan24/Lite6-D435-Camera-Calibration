import cv2
import numpy as np
import pyrealsense2 as rs

# --- KIỂM TRA LẠI THÔNG SỐ NÀY ---
# Nếu bàn cờ có 12 ô ngang và 9 ô dọc -> Số góc trong là (11, 8)
# Nếu bạn đếm được 12 điểm giao nhau ngang và 9 điểm giao nhau dọc -> Để (12, 9)
CHESSBOARD_SIZE = (8, 11) 

def debug_chessboard():
    # Cấu hình RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        pipeline.start(config)
        print("Đã kết nối RealSense. Đang tìm bàn cờ...")
    except Exception as e:
        print(f"Lỗi kết nối: {e}")
        return

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Chuyển sang mảng numpy
        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 1. Thử tìm góc bàn cờ với cờ hiệu tối ưu hóa
        # CALIB_CB_ADAPTIVE_THRESH: Giúp nhận diện tốt hơn khi ánh sáng không đều
        # CALIB_CB_NORMALIZE_IMAGE: Giúp tăng độ tương phản
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            # Nếu tìm thấy, vẽ các điểm màu lên
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            
            cv2.putText(img, "STATUS: SUCCESS", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(img, "STATUS: SEARCHING...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Hiển thị hướng dẫn
        cv2.putText(img, f"Target Size: {CHESSBOARD_SIZE}", (20, 450), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow("Debug Chessboard RealSense", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    debug_chessboard()