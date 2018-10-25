figure
hold on
view(30,30)

%line1
line1_p1 = [1;1;0];
line1_p2 = [-1;-1;0];

%line1
line2_p1 = [-1;1;0];
line2_p2 = [1;-1;0];


%
set(findall(gca, 'Type', 'Line'),'LineWidth',5);

hold off
