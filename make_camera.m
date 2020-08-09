%This is the make camera function
function camera = make_camera( x, y, max_range, color )


% Default values of the camera struct
camera.x = x;
camera.y = y;
camera.size = 1;
camera.max_range = 20; 
camera.color = color;
camera.trail = false;

s = camera.size;
fig_coords = [-s s 2*s s -s; -s -s 0 s s];
w1 = [-0.8*s -0.1*s -0.1*s -0.8*s; -s -s -1.25*s -1.25*s];
w2 = [-0.8*s -0.1*s -0.1*s -0.8*s; s s 1.25*s 1.25*s];
w3 = [0.1*s 0.8*s 0.8*s 0.1*s; -s -s -1.25*s -1.25*s];
w4 = [0.1*s 0.8*s 0.8*s 0.1*s; s s 1.25*s 1.25*s];
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

camera.fig_coords = R*fig_coords;
camera.w1 = R*w1;
camera.w2 = R*w2;
camera.w3 = R*w3;
camera.w4 = R*w4;

camera.h = zeros(6,1);
camera.h(1) = patch(camera.fig_coords(1,:)+x, camera.fig_coords(2,:)+y, camera.color); 
camera.h(2) = patch(camera.w1(1,:)+camera.x, camera.w1(2,:)+camera.y, 'k');  
camera.h(3) = patch(camera.w2(1,:)+camera.x, camera.w2(2,:)+camera.y, 'k');  
camera.h(4) = patch(camera.w3(1,:)+camera.x, camera.w3(2,:)+camera.y, 'k');  
camera.h(5) = patch(camera.w4(1,:)+camera.x, camera.w4(2,:)+camera.y, 'k');  
camera.h(6) = plot(x,y,'k*');
