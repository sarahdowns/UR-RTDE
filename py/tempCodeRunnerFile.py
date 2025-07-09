_c.getAsyncOperationProgressEx().isAsyncOperationRunning():
#     time.sleep(0.1)

#     tcp_pose = rtde_r.getActualTCPPose()
#     tcp_force = rtde_r.getActualTCPForce()
#     timestamp = time.time()

#     log_writer.writerow([
#         f"{timestamp:.4f}",
#         f"{tcp_pose[0]:.4f}", f"{tcp_pose[1]:.4f}", f"{tcp_pose[2]:.4f}",
#         f"{tcp_pose[3]:.4f}", f"{tcp_pose[4]:.4f}", f"{tcp_pose[5]:.4f}",
#         f"{tcp_force[0]:.4f}", f"{tcp_force[1]:.4f}", f"{tcp_force[2]:.4f}"
#     ])