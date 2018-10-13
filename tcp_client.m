t = tcpclient('192.168.2.2',9989)
while(true)
data = read(t);
disp(data)
end