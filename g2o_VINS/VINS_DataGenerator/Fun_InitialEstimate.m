function [ InitialEstimate ] = Fun_InitialEstimate( RealVehicle, Settings )

NumOfFrames=Settings.NumOfFrames;

InitialEstimate=[];
BS=Settings.BS;

for i=0:1:NumOfFrames-1
    
    orientation= RealVehicle.orientation( 1:3,  i*3*BS+1    :  i*3*BS+3  );
    velocity_L=RealVehicle.localvelocity( :, i*BS+1  );
    position=RealVehicle.position( :, i*BS+1  );
    
    accbias=RealVehicle.acc_bias_history(1:3, i*BS+1 );
    gyrobias=RealVehicle.gyro_bias_history(1:3, i*BS+1 );
    
    index=[i ];
    Index=repmat(index, 3, 1 );
    
    tempA= [orientation  position velocity_L  Index  accbias  gyrobias];
      
    InitialEstimate= [InitialEstimate; tempA   ]; 
end




end

