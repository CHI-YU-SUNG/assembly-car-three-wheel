import serial
import socket
from time import sleep

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
host='192.168.1.138'
port=12346
s.bind((host,port))

s.listen(5)
ser = serial.Serial('/dev/serial0', 115200, timeout=None,parity=serial.PARITY_NONE)

while True:
    try:
       	c, addr=s.accept()
	print 'connected by ',addr
	while True:
		try:
			#print'From client: ',c.recv(1024)
        		#c.send('i am app inventor\n')
			rec=c.recv(11)	
		#	if rec[i]=='/':
		#		print('hello')
		#			break
			print(rec)
			ser.write(rec)
			#ser.write('-1000,-1000\r\n')
			#sleep(1)
		except Exception:
			c.close()
			break
    except KeyboardInterrupt:

        break

print 'Close Server!'
s.close()
