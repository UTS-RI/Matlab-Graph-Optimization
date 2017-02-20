function [ Feature ] = Fun_generateFeatures

Feature=struct;
Feature.num=50;


%radius=10;
%Feature.position=(rand(3,Feature.num)-0.5*ones(3,Feature.num))*2*radius;

Feature.position=[];
 for i=-2:1:2
     for j=-2:1:2
         for k=-2:1:2
         Feature.position=[Feature.position [i;j;k]];
         end
     end
 end


Feature.num = size(Feature.position ,2 );

end

