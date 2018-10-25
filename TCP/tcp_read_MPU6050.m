t = tcpclient('192.168.2.2',9995);
fig = figure
hold on
times = [0];
axs = [0];
p = plot(times,axs)
p.XDataSource = 'times';
p.YDataSource = 'axs';
tic
while(true)
    data = read(t,8);
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