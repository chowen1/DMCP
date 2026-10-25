b=zeros(5,6);
for i = 1:6
a1 = tabulate(proposedPow(:,i,1));
a2 = tabulate(proposedPow(:,i,2));
for j = 1:5
b(6-j,i) = (a1(j,3) + a2(j,3))/2*0.01;
end
end

xvalues = {'CAV1','CAV2','CAV3','CAV4','CAV5','CAV6'};
yvalues = {'10','8','6','4','2'};
h = heatmap(xvalues,yvalues,b);

h.Title = 'Power heatmap';
h.XLabel = 'CAV';
h.YLabel = 'Power(mW)';


