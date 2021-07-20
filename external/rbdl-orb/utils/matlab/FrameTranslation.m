function [F] = FrameTranslation (translation, euler_angles)

% Here we store the result
Result = zeros (4,4);
Result(4,4) = 1;

% Set the translation part:
Result (1:3, 4) = -translation;

% Calculate the rotations
Rotation = eye (3,3);

% Z Rotation
RotZ = eye (3,3);

if (euler_angles(1) != 0.) 
	s = sin (euler_angles(1));
	c = cos (euler_angles(1));
	RotZ = [c, s, 0;
					-s, c, 0;
					0, 0, 1];
end 

Rotation = RotZ;

Result (1:3, 1:3) = Rotation;

F = Result;

end;
