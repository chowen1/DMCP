b=zeros(12,6);
for i = 1:6
a1 = tabulate(proposedBeam(:,i,1));
a2 = tabulate(proposedBeam(:,i,2));
for j = 1:12
b(13-j,i) = (a1(j,3) + a2(j,3))/2*0.01;
end
end

xvalues = {'CAV1','CAV2','CAV3','CAV4','CAV5','CAV6'};
yvalues = {'11/6','5/3','3/2','4/3','7/6','1','5/6','2/3','1/2','1/3','1/6','0'};
h = heatmap(xvalues,yvalues,b);

h.Title = 'Power heatmap';
h.XLabel = 'CAV';
h.YLabel = 'AOD(Π)';


