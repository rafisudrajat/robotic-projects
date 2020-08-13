#Algoritma kode robot
1.  Robot diletakkan di sebelah kiri titik tengah start area (0-0.5 M) 
2.  Setting setpoint line X  sebagai garis lurus yang sejajar dengan garis yg memotong titik tengah start area [start center] dan tiang. setting setpoint line Y sebagai garis yg tegak lurus setpoint X. (dilakukan sebelum di awal lomba)
pake data dari IMU (bisa data dari MPU/CM).
3.  Deteksi tiang pake kamera, dan buat garis imajiner [trajectory line] yang memotong robot dan tiang (harus ubah koordinat kamera ke koordinat IMU robot) 
4.  Buat garis imajiner dari orientasi IMU sekarang [orientation line] dan jalanin gait robot, ambil data dari IMU sambil jalanin ros::timer. 
Jika |sudut antara orientation line dan setpoint line X| [namain psi] !=0 selama waktu tertentu [tol_time], jalanin turn gain sampai orientation line [-psi]-> nilai psi paling akhir. 
5.  Ulangi langkah 4 sampai |sudut antara trajectory line dan setpoint line X| >= 90 derajat [theta]. Jika terpenuhi maka check nilai psi, jika psi !=0 maka putar pake turn gain sampe jadi ~0.check lagi apakah |sudut antara trajectory line dan setpoint line X| >= 90 derajat.
6.  majuin robot dikit terus matikan gait robot, nyalakan turn gain sampai orientation line sejajar dengan setpoint Y. Ulangi langkah 3-5 tapi setpoint yang dipake jadi setpoint Y.
7.  majuin robot dikit terus matikan gait robot, nyalakan turn gain sampai orientation line sejajar dengan setpoint X.
8.  Ubah trajectory line jadi garis yang memotong robot dan titik tengah start area
9.  Ulangi langkah 4 dan 5

STATE:

initial: [pake constructor]
-menentukan setpoint_X dan setpoint_Y

planning: 
-Deteksi objek yang akan dijadikan trajectory path (bisa tiang dan sesuatu objek yang ada di start center)
-Menentukan/memilih setpoint manakah yang akan dijadikan acuan

walking:
-Robot berjalan dengan timer tol_time
-jika tol_time habis maka cek nilai psi, jika psi!=0, maka turn gain dinyalakan hingga psi baru = -psi lama [robot tetep jalan]
-check nilai theta secara berkala apakah sudah >= 90 derajat. jika terpenuhi, maka check nilai psi, jika psi!=0, maka hentikan jalan robot
dan nyalakan turn gain sampai psi mendekti 0. kemudian check lagi apakah nilai theta >= 90 derajat

mini_translation:
-majuin robot dikit (pake timer)