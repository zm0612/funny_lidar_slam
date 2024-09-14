import numpy as np
import matplotlib.pyplot as plt


def load_txt_data(data_path):
    try:
        return np.loadtxt(data_path)
    except FileNotFoundError as err:
        print('this is a OSError: ' + str(err))


fuse_path_data = load_txt_data('./fuse_path.txt')
gt_path_data = load_txt_data('./gt_path.txt')
gps_measure_data = load_txt_data('./gps_measure.txt')
imu_bias_data = load_txt_data('./imu_bias.txt')

plt.figure(0)
plt.plot(fuse_path_data[:, 1], fuse_path_data[:, 2], color='g', linestyle='-', label='fuse_path')
plt.plot(gt_path_data[:, 1], gt_path_data[:, 2], color='r', linestyle='--', label='gt_path')
plt.scatter(gps_measure_data[:, 1], gps_measure_data[:, 2], color='b', label='gps_measure', s=3)
plt.title("path")
plt.legend()

f, ax = plt.subplots(2, 3)
ax[0, 0].axhline(imu_bias_data[0, 0], color='r', label='acc_bias_x_gt')
ax[0, 0].plot(imu_bias_data[1:-1, 0], color='g', label='acc_bias_x')
ax[0, 0].set_title('acc bias x')

ax[0, 1].axhline(imu_bias_data[0, 1], color='r', label='acc_bias_y_gt')
ax[0, 1].plot(imu_bias_data[1:-1, 1], color='g', label='acc_bias_y')
ax[0, 1].set_title('acc bias y')

ax[0, 2].axhline(imu_bias_data[0, 2], color='r', label='acc_bias_z_gt')
ax[0, 2].plot(imu_bias_data[1:-1, 2], color='g', label='acc_bias_z')
ax[0, 2].set_title('acc bias z')

ax[1, 0].axhline(imu_bias_data[0, 3], color='r', label='gyro_bias_x_gt')
ax[1, 0].plot(imu_bias_data[1:-1, 3], color='g', label='gyro_bias_x')
ax[1, 0].set_title('gyro bias x')

ax[1, 1].axhline(imu_bias_data[0, 4], color='r', label='gyro_bias_y_gt')
ax[1, 1].plot(imu_bias_data[1:-1, 4], color='g', label='gyro_bias_y')
ax[1, 1].set_title('gyro bias y')

ax[1, 2].axhline(imu_bias_data[0, 5], color='r', label='gyro_bias_z_gt')
ax[1, 2].plot(imu_bias_data[1:-1, 5], color='g', label='gyro_bias_z')
ax[1, 2].set_title('gyro bias z')
plt.show()
