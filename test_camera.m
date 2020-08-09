function [ camera, bearing ] = test_camera( camera, robot )

if robot.bearing > camera.max_range
    bearing = [];
    camera.color = [0.5 0.5 0.5];
else
    camera.color = 'b';
    bearing = pdist([camera, robot], Euclidean);
end
