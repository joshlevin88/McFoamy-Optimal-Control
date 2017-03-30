% function [M] = ManeuverTrajectory(t,x,y,z,pitch,roll,yaw,scale_factor,step,varargin)
function [M] = ManeuverTrajectory(t,x,y,z,pitch,roll,yaw,scale_factor,step)

%   x,y,z               center trajectory (vector)    [m]
%   pitch,roll,yaw      euler's angles                [rad]
%   scale_factor        normalization factor          [scalar] (related to body aircraft dimension)
%   step                attitude sampling factor      [scalar] (the points number between two body models)

%   OPTIONAL INPUT: 
%   View               sets the camera view. Use Matlab's "view" as argument to reuse the current view.

%   NOTE:
%   Reference System:
%                       X body- The axial force along the X body  axis is
%                       positive along forward; the momentum around X body
%                       is positive roll clockwise as viewed from behind;
%                       Y body- The side force along the Y body axis is
%                       positive along the right wing; the moment around Y
%                       body is positive in pitch up;
%                       Z body- The normal force along the Z body axis is
%                       positive down; the moment around Z body is positive
%                       roll clockwise as viewed from above.

if (length(x)~=length(y))||(length(x)~=length(z))||(length(y)~=length(z))
    disp('  Error:');
    disp('      Uncorrect Dimension of the center trajectory Vectors. Please Check the size');
    M = 0;
    return;
end

if ((length(pitch)~=length(roll))||(length(pitch)~=length(yaw))||(length(roll)~=length(yaw)))
    disp('  Error:');
    disp('      Uncorrect Dimension of the euler''s angle Vectors. Please Check the size');
    M = 0;
    return;
end

if length(pitch)~=length(x)
    disp('  Error:');
    disp('      Size mismatch between euler''s angle vectors and center trajectory vectors');
    M=0;
    return
end

% if step size is defined larger than the total distance
if step >= length(x)
    disp('  Error:');
    disp('      Attitude samplig factor out of range. Reduce step');
    M = 0;
    return
end

% if step size is defined less than one
if step < 1
    step = 1;
end

% if view input argument is provided
% if nargin == 10   
%     theView = cell2mat(varargin(1));
% end
% 
% if nargin < 10
    theView = [82.50 2];
% end

mov = nargout;
%cur_dir = pwd; % current directly

% Load the YAK54 geometry
load YAK54_MatlabMdl
V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));

correction = max(abs(V(:,1)));
V = V./(scale_factor*correction);
ii = length(x);
resto = mod(ii,step);

frame = 0;

for i = 1:step:(ii-resto)
    
    if mov || (i == 1)
      %clf;
      %plot3(x,y,z);
      surface('XData',[x(:) x(:)],'YData',[y(:) y(:)],'ZData',[z(:) z(:)],'CData',[t(:) t(:)],'FaceColor','none','EdgeColor','flat','LineWidth',1.5,'Marker','none');
      set(gca,'zdir','reverse')
      set(gca,'ydir','reverse')
      grid on;
      xlabel('X'),ylabel('Y'),zlabel('Z');
      hold on;
      %light;
    end

    theta = pitch(i);
    phi = roll(i);
    psi = yaw(i);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
     Tbe = [cos(psi)*cos(theta), sin(psi)*cos(theta), -sin(theta);
         cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) ...
         sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
         cos(theta)*sin(phi);
         cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
         sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi) ...
         cos(theta)*cos(phi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Vnew = V*Tbe;
    rif = [x(i) y(i) z(i)];
    X0 = repmat(rif,size(Vnew,1),1);
    Vnew = Vnew + X0;
    p = patch('faces', F1, 'vertices' ,Vnew);    
    set(p, 'facec', [1 1 1]);
    p = patch('faces', F2, 'vertices' ,Vnew);    
    set(p, 'facec', [1 1 1]);
    p = patch('faces', F3, 'vertices' ,Vnew);    
    set(p, 'facec', [1 1 1]);
    p = patch('faces', F4, 'vertices' ,Vnew);    
    p = patch('faces', F5, 'vertices' ,Vnew);    
    p = patch('faces', F6, 'vertices' ,Vnew);    
    p = patch('faces', F7, 'vertices' ,Vnew);    
    p = patch('faces', F8, 'vertices' ,Vnew);    
    set(p, 'facec', [1 1 1]);    
    %set(p, 'EdgeColor','none');

    if mov || (i == 1)
         view(theView);
          axis equal;
    end

    if mov
        if i == 1
            ax = axis;
        else
            axis(ax);
        end

        lighting phong
        frame = frame + 1;
        M(frame) = getframe;
    end

end

hold on;
%plot3(x,y,z);
%lighting phong;
grid on;
view(theView);

daspect([1 1 1]) ;
set(gca,'FontSize',20);
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 30);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 30);
zlabel('$z$ [m]', 'Interpreter', 'latex', 'FontSize', 30);
saveas(gcf,'Maneuver_Traj.png');