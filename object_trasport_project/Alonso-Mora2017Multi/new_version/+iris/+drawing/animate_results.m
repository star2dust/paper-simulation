function animate_results(results, record, folder_name)
import iris.thirdParty.polytopes.*;

if nargin < 2
  record = false;
else
  if nargin < 3
    folder_name = ['videos/', datestr(now,'yyyy-mm-dd_HH.MM.SS')];
  end
end

dim = length(results.start);
C = results.e_history{1}.C;
d = results.e_history{1}.d;
A_bounds = results.p_history{1}.A(end-2*dim+1:end,:);
b_bounds = results.p_history{1}.b(end-2*dim+1:end);
bound_pts = lcon2vert(A_bounds, b_bounds);
lb = min(bound_pts)';
ub = max(bound_pts)';

if record
  frame = 1;
  system(sprintf('mkdir -p %s', folder_name));
  w = VideoWriter([folder_name, '/', 'animation']);
  w.FrameRate = 5;
  w.open();
  save([folder_name, '/', 'results'], 'results');

  delay = 1/w.FrameRate;
  delay1 = 1;
  delay2 = 2;
  gif_fname = [folder_name, '/', 'animation.gif'];
  loops = 65525;

  draw([], [], C, d, results.obstacles, lb, ub, results);
  h = gcf;
  movegui(h);
  w.writeVideo(getframe(h));
  print(gcf, sprintf('%s/%d_a', folder_name, 0), '-dpdf');
  img = getframe();
  [M, c_map]= rgb2ind(img.cdata,256);
  imwrite(M,c_map,[gif_fname],'gif','LoopCount',loops,'DelayTime',delay1);
end

figure(2);
clf;
for j = 1:length(results.p_history)
  A = results.p_history{j}.A;
  b = results.p_history{j}.b;
  draw(A, b, C, d, results.obstacles, lb, ub, results);
  if record
    h = gcf;
    movegui(h);
    w.writeVideo(getframe(h));
    print(gcf, sprintf('%s/%d_a', folder_name, j), '-dpdf');
    img = getframe();
    [M, c_map]= rgb2ind(img.cdata,256);
    imwrite(M,c_map,[gif_fname],'gif','WriteMode','append','DelayTime',delay)
  end
%   pause(0.1);
  C = results.e_history{j+1}.C;
  d = results.e_history{j+1}.d;
  draw(A, b, C, d, results.obstacles, lb, ub, results);
  if record
    h = gcf;
    movegui(h);
    w.writeVideo(getframe(h));
    print(gcf, sprintf('%s/%d_b', folder_name, j), '-dpdf', '-painters');
    img = getframe();
    [M, c_map]= rgb2ind(img.cdata,256);
    if j==length(results.p_history)
      imwrite(M,c_map,[gif_fname],'gif','WriteMode','append','DelayTime',delay2);
    else
      imwrite(M,c_map,[gif_fname],'gif','WriteMode','append','DelayTime',delay);
    end
  end
%   pause(0.25);
end

if record
  w.close();
end
end

function draw(A, b, C, d, obstacles, lb, ub, results)
  import iris.drawing.*;
  dim = length(results.start);
  if dim == 2
      draw_2d(A,b,C,d,obstacles,lb,ub);
      plot(results.start(1), results.start(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
  elseif dim == 3
      draw_3d(A,b,C,d,obstacles,lb,ub);
      plot3(results.start(1), results.start(2), results.start(3), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 15);
  else
    draw_Nd(A,b,C,d,obstacles,lb,ub);
  end
end