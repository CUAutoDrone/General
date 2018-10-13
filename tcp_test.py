import socket

HOST = '192.168.2.3'
PORT = 9995

with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((HOST,PORT))
	s.listen()
	conn,addr = s.accept()
	with conn:
		print('Connected by',addr)
		while True:
			conn.sendall(b"3")
		