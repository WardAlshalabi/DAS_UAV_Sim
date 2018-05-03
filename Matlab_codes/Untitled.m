close all
clear all
clc
rosshutdown
rosinit('10.40.5.178');
global data;
global goal;
goal= [0,0,0];
sub = rossubscriber('/communication',@Callback);
pause(1)
number_of_points=12;
radius=10;
y1=450;
y2=270;
y3=90;
circles=[90, y1, radius;    300, y1, radius;    510, y1, radius;    685, y1, radius;
         90, y2, radius;    270, y2, radius;    480, y2, radius;    685, y2, radius;
         120, y3, radius;   330, y3, radius;    510, y3, radius;    685, y3, radius];
    
global counters
counters=zeros(1,number_of_points);
while(1)
RGB=imread('arena2.png');
for i=1:number_of_points
RGB = insertShape(RGB,'FilledCircle',circles(i,:),'color',[counters(i),255-counters(i),0]);
end
imshow(RGB);
for i=1:number_of_points
    if(counters(i)<255)
    counters(i)=counters(i)+1;
    end
end
%write ROS part here

end

function Callback(~, message)
    %exampleHelperROSChatterCallback - ROS subscriber callback function to display data
    %   from the message.
    %   exampleHelperROSChatterCallback(~,MESSAGE) returns no arguments- it simply displays
    %   message content.
    %   
    %   See also ROSPublishAndSubscribeExample
    
    %   Copyright 2014 The MathWorks, Inc.
    global counters;
    global data;
    global goal;
    data=message.Data;
    if(goal(data(1)+1)~=0)
    counters(goal(data(1)+1))=0;
    end
    goal(data(1)+1)=data(2)-2;
    
end