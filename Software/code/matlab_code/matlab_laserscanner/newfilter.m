function LaserMask = newfilter(vidobj,imgSize,aMega)

fprintf(aMega,'r2r');
frameOn=getsnapshot(vidobj);
pause(.01);
fprintf(aMega,'r3r');

[LaserMask,~] = thres(frameOn); 
end