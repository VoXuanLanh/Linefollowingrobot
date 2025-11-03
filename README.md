Hệ thống xe dò line được thiết kế nhằm mục tiêu tạo ra một robot tự hành có khả năng bám theo vạch đen trên nền trắng và vượt qua các thử thách như cua gấp, ngã ba, ngã tư, đoạn đứt line và vòng tròn theo yêu cầu của đường đua. Xe được điều khiển hoàn toàn tự động, không cần can thiệp của người dùng trong quá trình vận hành.
Cấu trúc hệ thống gồm các khối chức năng chính:
- Khối điều khiển trung tâm: Sử dụng vi điều khiển ESP32 làm bộ xử lý chính, đọc dữ liệu từ cảm biến, tính toán điều khiển (PID) và phát tín hiệu PWM điều khiển động cơ.
- Khối truyền động: Gồm 2 động cơ DC gắn bánh xe chủ động và 1 bánh đa hướng giúp xe di chuyển linh hoạt và ổn định. Driver TB6612FNG điều khiển tốc độ và chiều quay động cơ.
- Khối cảm biến: Sử dụng cảm biến dò line 5 kênh dựa trên hồng ngoại TCRT5000, phát hiện vị trí vạch line và gửi tín hiệu số về vi điều khiển để xác định hướng đi.
- Dùng nguồn pin Li-ion 3.7–4.2V cung cấp năng lượng cho cả hệ thống, đảm bảo xe hoạt động độc lập, cơ động.
- Khối cơ khí: Khung xe được thiết kế nhỏ gọn, nhẹ, đáp ứng giới hạn kích thước ≤ 20×20×20 cm và khối lượng ≤ 1.5 kg, dễ bảo trì và lắp ráp.
