---

# Lite6 & RealSense Eye-in-Hand Calibration Suite

Repository này cung cấp bộ công cụ Python hoàn chỉnh để thực hiện hiệu chuẩn **Eye-in-Hand** cho robot **UFactory Lite 6** và camera **Intel RealSense D435i**. Quy trình bao gồm từ bước kiểm tra nhận diện hình ảnh, tính toán ma trận chuyển đổi đến kiểm chứng sai số thực tế.

## 🛠 Yêu cầu hệ thống

### 1. Phần cứng
- Robot UFactory Lite 6.
- Camera Intel RealSense D435i (Kết nối USB 3.0).
- Tấm hiệu chuẩn bàn cờ (Chessboard pattern).

### 2. Phần mềm
Cài đặt các thư viện cần thiết:
```bash
pip install xarm-python-sdk pyrealsense2 opencv-contrib-python scipy
```

## 📂 Danh sách các Script

1.  **`chessboard.py`**: Công cụ debug giúp kiểm tra khả năng nhận diện bàn cờ của OpenCV từ luồng dữ liệu camera. Sử dụng để đảm bảo cấu hình số lượng góc (`CHESSBOARD_SIZE`) chính xác trước khi calib.
2.  **`xarm_calib.py`**: Script chính thực hiện thu thập dữ liệu tọa độ robot và camera. Sử dụng thuật toán Tsai-Lenz để tính toán ma trận chuyển đổi $T_{cam}^{flange}$.
3.  **`verify_calib.py`**: Chương trình kiểm chứng độ chính xác sau hiệu chuẩn. Script này đo lường sự sai lệch vị trí vật thể trong hệ tọa độ Base khi robot di chuyển ở nhiều tư thế khác nhau.

## 🚀 Quy trình thực hiện

### Bước 1: Kiểm tra nhận diện hình ảnh (`chessboard.py`)
- Cấu hình `CHESSBOARD_SIZE` khớp với số góc trong của tấm bàn cờ (ví dụ: `(8, 11)`).
- Chạy script: `python chessboard.py`.
- Đảm bảo trạng thái hiện **STATUS: SUCCESS** trước khi tiếp tục.

### Bước 2: Hiệu chuẩn Hand-Eye (`xarm_calib.py`)
- Thiết lập `ROBOT_IP` và kích thước ô vuông `SQUARE_SIZE` (đơn vị mét).
- Chạy script: `python xarm_calib.py`.
- Di chuyển robot đến các vị trí quan sát khác nhau (thay đổi linh hoạt góc Roll, Pitch, Yaw) và nhấn **'S'** để lưu mẫu.
- Sau khi thu thập đủ mẫu (đề xuất trên 15 mẫu, hệ thống đã thử nghiệm thành công với 26 mẫu), nhấn **'Q'** để tính toán kết quả.
- Kết quả ma trận 4x4, vector dịch chuyển (Translation) và góc xoay (Rotation) sẽ được in ra màn hình.

### Bước 3: Kiểm chứng sai số (`verify_calib.py`)
- Sao chép ma trận kết quả vào biến `T_CAM_EE` trong script `verify_calib.py`.
- Chạy script: `python verify_calib.py`.
- Di chuyển robot và nhấn **Enter** tại các vị trí khác nhau để lấy mẫu tọa độ bàn cờ trong hệ Base.
- Nhấn **'Q'** để nhận báo cáo sai số (Max Error). Sai số dưới 5mm được đánh giá là rất tốt cho các tác vụ gắp nhả chính xác.

## ⚠️ Lưu ý kỹ thuật
- **Độ ổn định:** Bàn cờ phải được cố định tuyệt đối trên mặt phẳng trong suốt quá trình hiệu chuẩn và kiểm chứng.
- **Tọa độ:** Script tự động xử lý việc chuyển đổi từ mm (robot) sang mét (camera/calibration) để đảm bảo tính đồng nhất.

---
*Dự án được thực hiện bởi Nhân - Sinh viên ngành Robot và Trí tuệ nhân tạo, HUTECH.*
