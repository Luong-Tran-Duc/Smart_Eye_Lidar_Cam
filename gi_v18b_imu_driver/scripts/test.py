import sys

def check_frame(data):
    if len(data) != 80:
        return False, "Frame too short"
    if data[0] != 0x55 or data[1] != 0xAA or data[2] != 0x4C:
        return False, "Header mismatch"
    checksum = sum(data[3:79]) & 0xFF
    if checksum != data[79]:
        return False, f"Checksum FAIL: expected {checksum:#02x}, got {data[79]:#02x}"
    return True, "OK"

def analyze_file(filename):
    with open(filename, 'rb') as f:
        buf = f.read()

    print(f"📄 Tổng cộng {len(buf)} byte trong file `{filename}`")

    valid = 0
    corrupt = 0
    i = 0
    while i < len(buf) - 80:
        if buf[i] == 0x55 and buf[i+1] == 0xAA and buf[i+2] == 0x4C:
            frame = buf[i:i+80]
            ok, msg = check_frame(frame)
            if ok:
                print(f"✅ Gói hợp lệ tại offset {i}")
                valid += 1
                i += 80
            else:
                print(f"❌ Gói lỗi tại offset {i}: {msg}")
                corrupt += 1
                i += 1  # trượt để tìm gói tiếp
        else:
            i += 1

    print("\n📊 TỔNG KẾT:")
    print(f"✅ Gói hợp lệ: {valid}")
    print(f"❌ Gói lỗi: {corrupt}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_imu_dump.py imu_dump.bin")
    else:
        analyze_file(sys.argv[1])
