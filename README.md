---

# Lite6 & RealSense Eye-in-Hand Calibration

Repository này cung cấp các công cụ Python để thực hiện hiệu chuẩn **Eye-in-Hand** (camera gắn trên khâu cuối) cho robot **UFactory Lite 6** sử dụng camera **Intel RealSense D435i**. Quy trình này xác định ma trận biến đổi tọa độ từ tâm camera về mặt bích (flange) của robot.

## 🛠 Yêu cầu hệ thống

### 1. Phần cứng
- Robot UFactory Lite 6.
- Camera Intel RealSense D435i (kết nối qua cổng USB 3.0).
- Tấm hiệu chuẩn Chessboard (Bàn cờ trắng đen).

### 2. Phần mềm
Cài đặt các thư viện cần thiết:
```bash
pip install xarm-python-sdk pyrealsense2 opencv-contrib-python scipy
```

## 📂 Cấu trúc mã nguồn

* **`chessboard.py`**: Công cụ chẩn đoán hình ảnh. Sử dụng để kiểm tra xem OpenCV có nhận diện được bàn cờ và đếm đúng số lượng góc (corners) hay không trước khi tiến hành hiệu chuẩn.
* **`xarm_calib.py`**: Script chính thực hiện quy trình hiệu chuẩn. Nó thu thập dữ liệu tọa độ từ robot và camera, sau đó tính toán ma trận Hand-Eye bằng phương pháp Tsai-Lenz.

## 🚀 Quy trình thực hiện

### Bước 1: Kiểm tra nhận diện (`chessboard.py`)
1.  Đảm bảo biến `CHESSBOARD_SIZE` trong code khớp với số góc giao nhau bên trong tấm bàn cờ của bạn.
    * *Ví dụ: Bàn cờ 11x8 ô vuông tương ứng với (10, 7) góc trong.*
2.  Chạy script để kiểm tra luồng video:
    ```bash
    python chessboard.py
    ```
3.  Nếu hiện **STATUS: SUCCESS**, bạn đã sẵn sàng sang bước tiếp theo.

### Bước 2: Hiệu chuẩn Hand-Eye (`xarm_calib.py`)
1.  Cập nhật `ROBOT_IP` và `SQUARE_SIZE` (kích thước thật của một cạnh ô vuông đơn vị mét).
2.  Chạy chương trình:
    ```bash
    python xarm_calib.py
    ```
3.  **Lấy mẫu (Data Collection):**
    - Di chuyển robot đến ít nhất 15 vị trí khác nhau sao cho camera luôn thấy rõ bàn cờ.
    - **Quan trọng:** Phải thay đổi góc xoay (Roll, Pitch, Yaw) của cổ tay robot ở mỗi vị trí để thuật toán đạt độ chính xác cao nhất.
    - Nhấn phím **'S'** tại mỗi vị trí để lưu mẫu.
4.  **Tính toán:**
    - Nhấn phím **'Q'** sau khi đã đủ mẫu để máy tính giải ma trận.

## 📊 Kết quả đầu ra
Sau khi hoàn tất, chương trình sẽ in ra ma trận biến đổi 4x4 ($T_{cam}^{flange}$):
- **Translation (m):** Độ lệch X, Y, Z giữa tâm camera và mặt bích.
- **Rotation:** Góc xoay Roll, Pitch, Yaw định hướng camera so với mặt bích.

## ⚠️ Lưu ý kỹ thuật
- **Độ phẳng:** Tấm bàn cờ phải tuyệt đối phẳng. Bất kỳ sự cong vênh nào cũng sẽ làm sai lệch kết quả calib.
- **Ánh sáng:** Tránh để ánh sáng đèn phản chiếu trực tiếp lên bề mặt bàn cờ (gây lóa).
- **Tọa độ:** Robot sử dụng đơn vị mm, nhưng kết quả calib được chuyển đổi sang mét (m) để đồng bộ với các thư viện Computer Vision.

---
