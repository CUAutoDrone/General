%t = tcpclient('192.168.137.175',9995);
t = tcpip('192.168.137.175',9995);
fopen(t);
fig = figure
hold on
times = [0];
axs = [0];
p = plot(times,axs)
p.XDataSource = 'times';
p.YDataSource = 'axs';
tic
while(true)
    data = fread(t,8);
    flushinput(t);
    str = native2unicode(data, 'UTF-8');
    ax = str2double(strtrim(str));
    times = [times toc];
    axs = [axs ax];
    if length(axs)>100
        times = times(2:end);
        axs = axs(2:end);
    end
    refreshdata(fig)
    drawnow
end