import paramiko
import time

try:
	while 1:
		file = open("./MasterGPS.txt", 'w')

		sshSlave = paramiko.SSHClient()
		sshSlave.set_missing_host_key_policy(paramiko.AutoAddPolicy)
		sshSlave.connect("192.168.11.20", port=2249, username="root", password="innonet160905")
		stdin, stdout, stderr = sshSlave.exec_command("cat /tmp/gps/lat /tmp/gps/lng")
		lines = stdout.readlines()
		GPSData = ''.join(lines).split('\n')
		lat = GPSData[0]
		lng = GPSData[1]
		StringData = lat + " " + lng
		file.write(StringData)
		file.close()
		time.sleep(60)

except Exception as err:
	print(err)
	file.close()
