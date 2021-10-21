import paramiko
import time

try:
	while 1:
		file = open("./SlaveGPS.txt", 'a')
		sshSlave = paramiko.SSHClient()
		sshSlave.set_missing_host_key_policy(paramiko.AutoAddPolicy)
		sshSlave.connect("192.168.1.1", port=2249, username="root", password="innonet160905")
		stdin, stdout, stderr = sshSlave.exec_command("cat /tmp/gps/lat /tmp/gps/lng")
		lines = stdout.readlines()
		SlaveGPSData = ''.join(lines).split('\n')
		Slavelat = SlaveGPSData[0]
		Slavelng = SlaveGPSData[1]
		StringData = Slavelat + " " + Slavelng
		file.write(StringData+"\n")
		file.close()
		time.sleep(2)

except Exception as err:
	print(err)
	file.close()
