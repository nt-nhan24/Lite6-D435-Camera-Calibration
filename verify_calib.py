import cv2
import numpy as np
import pyrealsense2 as rs
from xarm.wrapper import XArmAPI
from scipy.spatial.transform import Rotation as R

# --- THÔNG SỐ TỪ KẾT QUẢ CALIB CỦA BẠN ---
T_CAM_EE = np.array([
    [-0.00909662, -0.9998831 ,  0.01228976,  0.06846232],
    [ 0.99993235, -0.0091848 , -0.00713714, -0.03100006],
    [ 0.00724918,  0.01222401,  0.99989901,  0.00846006],
    [ 0.        ,  0.        ,  0.        ,  1.        ]
])

CHESSBOARD_SIZE = (8, 11) # Cập nhật theo thực tế tấm bàn cờ của bạn
SQUARE_SIZE = 0.023       # 23mm
ROBOT_IP = '192.168.1.165'

class CalibVerifier:
    def __init__(self):
        self.arm = XArmAPI(ROBOT_IP)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.mtx = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist = np.array(intr.coeffs)

        self.objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE
        
        self.recorded_base_positions = []

    def get_target_in_base(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: return None
        
        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        
        if ret:
            # 1. Tính T_target_cam (PnP)
            _, rvec, tvec = cv2.solvePnP(self.objp, corners, self.mtx, self.dist)
            rmat_target_cam, _ = cv2.Rodrigues(rvec)
            T_target_cam = np.eye(4)
            T_target_cam[:3, :3] = rmat_target_cam
            T_target_cam[:3, 3] = tvec.flatten()

            # 2. Lấy T_ee_base từ Robot
            code, pos = self.arm.get_position(is_radian=True)
            if code == 0:
                r_rob = R.from_euler('xyz', pos[3:], degrees=False).as_matrix()
                T_ee_base = np.eye(4)
                T_ee_base[:3, :3] = r_rob
                T_ee_base[:3, 3] = np.array(pos[:3]) / 1000.0

                # 3. Tính T_target_base = T_ee_base * T_cam_ee * T_target_cam
                T_target_base = T_ee_base @ T_CAM_EE @ T_target_cam
                
                # Tọa độ XYZ của tâm bàn cờ so với gốc robot
                pos_base = T_target_base[:3, 3]
                return pos_base, img
        return None, img

    def run(self):
        print("Đang kiểm chứng... Di chuyển robot đến các vị trí khác nhau.")
        print("Nhấn 'Enter' để lấy 1 mẫu tọa độ. Nhấn 'Q' để xem báo cáo sai số.")
        
        while True:
            pos_base, img = self.get_target_in_base()
            cv2.imshow("Verification", img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 13 and pos_base is not None: # Phím Enter
                self.recorded_base_positions.append(pos_base)
                print(f"Mẫu {len(self.recorded_base_positions)}: XYZ={pos_base}")
                
            elif key == ord('q'):
                break
        
        if len(self.recorded_base_positions) > 1:
            data = np.array(self.recorded_base_positions)
            mean_pos = np.mean(data, axis=0)
            std_dev = np.std(data, axis=0)
            max_error = np.max(np.linalg.norm(data - mean_pos, axis=1))

            print("\n" + "="*40)
            print("BÁO CÁO ĐỘ CHÍNH XÁC")
            print("="*40)
            print(f"Vị trí trung bình bàn cờ: {mean_pos}")
            print(f"Độ lệch chuẩn (X, Y, Z) mm: {std_dev * 1000}")
            print(f"Sai số khoảng cách lớn nhất (Max Error): {max_error * 1000:.2f} mm")
            
            if max_error < 0.005: # Dưới 5mm
                print("=> Kết quả: Rất tốt (Tuyệt vời cho gắp nhả chính xác)")
            elif max_error < 0.015: # Dưới 15mm
                print("=> Kết quả: Chấp nhận được (Cần tối ưu thêm nếu gắp linh kiện nhỏ)")
            else:
                print("=> Kết quả: Sai số lớn (Nên calib lại và xoay robot nhiều góc hơn)")
            print("="*40)

        self.arm.disconnect()
        self.pipeline.stop()

if __name__ == "__main__":
    verifier = CalibVerifier()
    verifier.run()