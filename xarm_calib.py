import cv2
import numpy as np
import pyrealsense2 as rs
from xarm.wrapper import XArmAPI
from scipy.spatial.transform import Rotation as R
import time

# --- CẤU HÌNH ---
CHESSBOARD_SIZE = (8, 11)  
SQUARE_SIZE = 0.023        # 23mm
ROBOT_IP = '192.168.1.165'

class ChessboardHandEye:
    def __init__(self, ip):
        # 1. Kết nối Robot
        print(f"Đang kết nối Robot tại {ip}...")
        self.arm = XArmAPI(ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        
        # 2. Kết nối Camera RealSense (Sửa lại thứ tự chuẩn)
        print("Đang khởi tạo RealSense D435i...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Thử cấu hình 640x480 để đảm bảo băng thông USB
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        try:
            self.profile = self.pipeline.start(self.config)
            print("Camera đã sẵn sàng!")
        except Exception as e:
            print(f"Lỗi khởi động camera: {e}")
            print("Thử lại với cấu hình mặc định...")
            self.profile = self.pipeline.start()

        # Lấy thông số nội tại Camera
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.mtx = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist = np.array(intr.coeffs)

        # Tạo tọa độ 3D chuẩn cho bàn cờ
        self.objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
        self.objp *= SQUARE_SIZE

    def get_chessboard_pose(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: return None
        
        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Sử dụng thêm các Flag để tăng khả năng nhận diện khi ánh sáng yếu hoặc lóa
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, flags)
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Giải PnP tìm Pose
            _, rvec, tvec = cv2.solvePnP(self.objp, corners2, self.mtx, self.dist)
            rmat, _ = cv2.Rodrigues(rvec)
            
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            cv2.putText(img, "READY TO SAVE (Press 'S')", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            return rmat, tvec, img
        else:
            cv2.putText(img, "SEARCHING CHESSBOARD...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return None, None, img

    def get_robot_pose(self):
        code, pos = self.arm.get_position(is_radian=True)
        if code == 0:
            # pos: [x, y, z, r, p, y] (mm, radian)
            r = R.from_euler('xyz', pos[3:], degrees=False)
            return r.as_matrix(), np.array(pos[:3]) / 1000.0 # Trả về mét
        return None, None

    def run(self):
        R_gripper2base, t_gripper2base = [], []
        R_target2cam, t_target2cam = [], []
        
        print("\nHƯỚNG DẪN:")
        print("1. Di chuyển robot sao cho camera thấy bàn cờ.")
        print("2. Nhấn 'S' để lưu mẫu (Cần ít nhất 15 mẫu ở các góc khác nhau).")
        print("3. Nhấn 'Q' để kết thúc và tính toán ma trận.")

        while True:
            rmat_cam, tvec_cam, img = self.get_chessboard_pose()
            cv2.imshow("Hand-Eye Calibration - Lite 6", img)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                if rmat_cam is not None:
                    rmat_rob, tvec_rob = self.get_robot_pose()
                    if rmat_rob is not None:
                        R_gripper2base.append(rmat_rob)
                        t_gripper2base.append(tvec_rob)
                        R_target2cam.append(rmat_cam)
                        t_target2cam.append(tvec_cam)
                        print(f"-> Đã lưu mẫu {len(R_gripper2base)} thành công!")
                    else:
                        print("Lỗi: Không lấy được tọa độ Robot.")
                else:
                    print("Lỗi: Camera chưa nhận diện được bàn cờ.")
            
            elif key == ord('q'):
                break

        # Tính toán kết quả cuối cùng
        if len(R_gripper2base) >= 10:
            print("\nĐang tính toán ma trận Hand-Eye...")
            try:
                R_cam2ee, t_cam2ee = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=cv2.CALIB_HAND_EYE_TSAI
                )
                
                print("\n" + "="*30)
                print("KẾT QUẢ CALIBRATION (Camera to Flange)")
                print("="*30)
                print(f"Translation (X, Y, Z) in m:\n{t_cam2ee.flatten()}")
                
                r_euler = R.from_matrix(R_cam2ee).as_euler('xyz', degrees=True)
                print(f"Rotation (Roll, Pitch, Yaw) in degrees:\n{r_euler}")
                
                # Tạo ma trận 4x4 để dùng sau này
                T_cam_ee = np.eye(4)
                T_cam_ee[:3, :3] = R_cam2ee
                T_cam_ee[:3, 3] = t_cam2ee.flatten()
                print("\nTransformation Matrix 4x4:\n", T_cam_ee)
                print("="*30)
                
            except Exception as e:
                print(f"Lỗi khi tính toán: {e}")
        else:
            print(f"Chỉ có {len(R_gripper2base)} mẫu. Cần tối thiểu 10-15 mẫu để có độ chính xác.")

        self.arm.disconnect()
        self.pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrator = ChessboardHandEye(ROBOT_IP)
    calibrator.run()