import time
from src.data.serial_reading_simple2 import HWT9053Reader

# === LANGKAH 1: Tentukan Port Sensor ===
# Sesuaikan dengan port USB/Serial yang terbaca di komputer (contoh: /dev/ttyUSB0, /dev/ttyUSB1, dll)
# Anda bisa mengeceknya di terminal dengan perintah: ls /dev/ttyUSB*
PORT = "/dev/ttyUSB2" 
BAUDRATE = 9600

def main():
    print("Mulai inisialisasi sensor WitMotion...")
    
    # === LANGKAH 2: Buat Objek Pembaca Sensor ===
    # Kita menggunakan kelas HWT9053Reader yang sudah ada untuk menangani komunikasi dasar (Modbus)
    reader = HWT9053Reader(port=PORT, baudrate=BAUDRATE)
    
    # === LANGKAH 3: Coba Hubungkan ke Sensor ===
    is_connected = reader.connect()
    
    if not is_connected:
        print(f"GAGAL: Tidak bisa terhubung ke sensor di port {PORT}.")
        print("Pastikan kabel USB sudah terpasang dan port-nya benar.")
        return

    print("BERHASIL: Sensor terhubung! Mulai membaca data mentah (raw data)...")
    print("-" * 50)
    
    try:
        # === LANGKAH 4: Loop Terus Menerus untuk Membaca Data ===
        while True:
            # Membaca seluruh data mentah dari sensor
            raw_data = reader.read_all()
            
            # Ambil beberapa data penting saja untuk ditampilkan (Akselerasi & Orientasi)
            # Anda juga bisa print(raw_data) langsung untuk melihat semuanya
            accel_x = raw_data.get('accelerationX', 0.0)
            accel_y = raw_data.get('accelerationY', 0.0)
            accel_z = raw_data.get('accelerationZ', 0.0)
            
            yaw = raw_data.get('yaw', 0.0)
            pitch = raw_data.get('pitch', 0.0)
            roll = raw_data.get('roll', 0.0)
            
            # Tampilkan ke layar dengan format yang rapi
            print(f"Accel(X,Y,Z): [{accel_x:5.2f}, {accel_y:5.2f}, {accel_z:5.2f}] | "
                  f"Sudut(R,P,Y): [{roll:6.2f}, {pitch:6.2f}, {yaw:6.2f}]")
            
            # Beri jeda sedikit agar tulisan di layar tidak berjalan terlalu cepat
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        # Menangkap sinyal saat kita menekan Ctrl+C untuk berhenti
        print("\nProgram dihentikan oleh user.")
        
    finally:
        # === LANGKAH 5: Putus Koneksi ===
        # Selalu pastikan menutup koneksi port serial ketika program selesai
        reader.disconnect()
        print("Koneksi sensor diputus.")

if __name__ == "__main__":
    main()
