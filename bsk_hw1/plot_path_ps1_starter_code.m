% program to plot actual pose history and motion inferred from encoder data
% make sure this file is in the same directory as the data files "pose.csv"
% and "encoders.csv"
load pose.csv
figure(1)
clf
xpose = pose(:,1);
ypose = pose(:,2);
plot(xpose,ypose,'r')
axis equal
title('path followed by Harlie: actual (red) and encoder est (blue)')
hold on %prepare this figure to receive a second plot, overlayed on the first

load encoders.csv
enc_left = encoders(:,1);
enc_rt = encoders(:,2);
npts = size(encoders,1);  %find out how many points there are
samps = 1:npts; %create a vector of sample numbers
kr = 1;
pos = [0,0];
positions = zeros(npts-1,2);

kr = 0.0008;
kl = 0.0008;

track = 0.5;
angle = pi/4+.2;

for i=1:npts-1
    dsr = kr * (enc_rt(i+1) - enc_rt(i));
    dsl = kl * (enc_left(i+1) - enc_left(i));
    dsv = 0.5 * dsr + 0.5 * dsl;
    angle = angle + ((dsl - dsr) / track);
    pos = [(pos(1) + dsv * sin(angle)),(pos(2) + dsv * cos(angle))];
    positions(i,1:2) = [pose(1,1)+pos(1),pose(1,2)+pos(2)];
end
plot(positions(:,1),positions(:,2),'b')
%figure(2)
%plot(samps, enc_left,'b',samps,enc_rt,'r')
%xlabel('sample number')
%ylabel('cumulative encoder counts')
%title('encoder counts left (blue) and right (red)')
