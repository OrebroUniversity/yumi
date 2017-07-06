from ftplib import FTP

left_arm_ftp = FTP('192.168.125.201')
left_arm_ftp.login('admin')
left_arm_ftp.retrlines('LIST')
left_arm_ftp.quit()