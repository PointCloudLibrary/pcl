% Copyright (c) 2014-, Open Perception, Inc.
% All rights reserved.
%
%  Redistribution and use in source and binary forms, with or without
%  modification, are permitted provided that the following conditions
%  are met:
%
%   * Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%   * Redistributions in binary form must reproduce the above
%     copyright notice, this list of conditions and the following
%     disclaimer in the documentation and/or other materials provided
%     with the distribution.
%   * Neither the name of the copyright holder(s) nor the names of its
%     contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
%  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  POSSIBILITY OF SUCH DAMAGE.
% Author: Marco Paladini <marco.paladini@ocado.com>

% sample octave script to load camera poses from file and plot them
% example usage: run 'pcl_kinfu_app -save_pose camera.csv' to save
% camera poses in a 'camera.csv' file
% run octave and cd into the directory where this script resides
% and call plot_camera_poses('<path-to-camera.csv-file>')

function plot_camera_poses(filename)
poses=load(filename);
%% show data on a 2D graph
h=figure();
plot(poses,'*-');
legend('x','y','z','qw','qx','qy','qz');

%% show data as 3D axis
h=figure();
for n=1:size(poses,1)
  t=poses(n,1:3);
  q=poses(n,4:7);
  r=q2rot(q);
  coord(h,r,t);
end
octave_axis_equal(h);

%% prevent Octave from quitting if called from the command line
input('Press enter to continue'); 
end

function coord(h,r,t)
figure(h);
hold on;
c={'r','g','b'};
p=0.1*[1 0 0;0 1 0;0 0 1];
for n=1:3 
  a=r*p(n,:)';
  plot3([t(1),t(1)+a(1)], [t(2),t(2)+a(2)], [t(3),t(3)+a(3)], 'color', c{n});
end

function R=q2rot(q)
% conversion code from http://en.wikipedia.org/wiki/Rotation_matrix%Quaternion	
Nq = q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2;
if Nq>0; s=2/Nq; else s=0; end
X = q(2)*s; Y = q(3)*s; Z = q(4)*s;
wX = q(1)*X; wY = q(1)*Y; wZ = q(1)*Z;
xX = q(2)*X; xY = q(2)*Y; xZ = q(2)*Z;
yY = q(3)*Y; yZ = q(3)*Z; zZ = q(4)*Z;
R=[ 1.0-(yY+zZ)       xY-wZ        xZ+wY  ;
      xY+wZ   1.0-(xX+zZ)       yZ-wX  ;
      xZ-wY        yZ+wX   1.0-(xX+yY) ];
end

function octave_axis_equal(h)
% workaround for axis auto not working in 3d
% tanks http://octave.1599824.n4.nabble.com/axis-equal-help-tp1636701p1636702.html
figure(h);
xl = get (gca, "xlim");
yl = get (gca, "ylim");
zl = get (gca, "zlim");
span = max ([diff(xl), diff(yl), diff(zl)]);
xlim (mean (xl) + span*[-0.5, 0.5])
ylim (mean (yl) + span*[-0.5, 0.5])
zlim (mean (zl) + span*[-0.5, 0.5])
end

