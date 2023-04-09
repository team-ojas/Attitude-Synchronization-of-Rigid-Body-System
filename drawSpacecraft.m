function drawSpacecraft(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);       % inertial altitude   
    u        = uu(4);       % body frame velocity i component
    v        = uu(5);       % body frame velocity j component
    w        = uu(6);       % body frame velocity k component
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);      % roll rate
    q        = uu(11);      % pitch rate     
    r        = uu(12);      % yaw rate    
    t        = uu(13);      % time

    % define persistent variables 
    persistent spacecraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0 
        figure(1), clf
        [Vertices, Faces, facecolors] = defineSpacecraftBody;
        spacecraft_handle = drawSpacecraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Spacecraft')
        grid on
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-10,10,-10,10,-10,10]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawSpacecraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           spacecraft_handle);
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [
      0, 1, 0;
      1, 0, 0;
      0, 0, -1
      ];
  V = V*R;
  
  if isempty(handle)
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function NED=rotate(NED,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;
  % rotate vertices
  NED = R*NED;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function NED = translate(NED,pn,pe,pd)
  NED = NED + repmat([pn;pe;pd],1,size(NED,2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define spacecraft vertices and faces
function [V,F,colors] = defineSpacecraftBody()
    % Define the vertices (physical location of vertices)
    V = [...
          1     0    0;... % point 1
        0.25   0.25 -0.25;... % point 2
        0.25  -0.25 -0.25;... % point 3
        0.25  -0.25  0.25;... % point 4
        0.25   0.25  0.25;... % point 5
         -3     0    0;... % point 6
          0   1.5    0;... % point 7
       -0.5   1.5    0;... % point 8
       -0.5  -1.5    0;... % point 9
          0  -1.5    0;... % point 10
       -2.5   0.5    0;... % point 11
         -3   0.5    0;... % point 12
         -3  -0.5    0;... % point 13
       -2.5  -0.5    0;... % point 14
       -2.5     0    0;... % point 15
         -3     0   -1;... % point 16
    ];

    % define faces as a list of vertices numbered above
    F = [...
        1,  2,  3;...  % nose_t
        1,  3,  4;...  % nose_l
        1,  4,  5;...  % nose_b 
        1,  5,  2;...  % nose_r
        2,  3,  6;...  % centre_body_t
        3,  4,  6;...  % centre_body_l
        4,  5,  6;...  % centre_body_b
        5,  2,  6;...  % centre_body_r
        6, 15, 16;...  % vertical_tail
        7,  8,  9;...  % wing
%         7,  9, 10;...  % wing
       11, 12, 13;...  % horizontal_tail 
%        11, 13, 14;...  % horizontal_tail
        ];

    % define colors for each face    
    myred = [1, 0, 0];
    mygreen = [0, 1, 0];
    myblue = [0, 0, 1];
    myyellow = [1, 1, 0];
    %mycyan = [0, 1, 1];

    colors = [...
        myred;...       % nose
        myred;...       % nose
        myred;...       % nose
        myred;...       % nose
        mygreen;...     % centre_body
        mygreen;...     % centre_body
        mygreen;...     % centre_body
        mygreen;...     % centre_body
        myblue;...      % vertical_tail
        myyellow...     % wing
%         myyellow...     % wing
        myyellow...     % horizontal_tail
%         myyellow...     % horizontal_tail
        ];
end

  