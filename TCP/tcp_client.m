t = tcpclient('192.168.2.3',9989);
while(true)
data = read(t);
str = native2unicode(data, 'UTF-8');
disp(str)
end