% Initialize space:
nrows = 400; ncols = 600;
obstacle = false(nrows, ncols);
[x, y] = meshgrid (1:ncols, 1:nrows);

% Generate obstacles
% a:
obstacle (300:end, 100:250) = true;
% b:
obstacle (150:200, 400:500) = true;
% c:
t = ((x - 200).^2 + (y - 50).^2) < 50^2;
obstacle(t) = true;
% d:
t = ((x - 400).^2 + (y - 300).^2) < 100^2;
obstacle(t) = true;

% Artificial potential, repulsive part:
d0 = 2; nu = 800;

d = bwdist(obstacle);
d2 = (d/100) + 1;

repulsive = nu*((1./d2 - 1/d0).^2);
repulsive (d2 > d0) = 0;

% Artificial potential, attractive part:
goal = [590, 50];
start = [50, 350];

xi = 1/700;
attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );

% Final potential:
f = attractive + repulsive;

% Final route:
route = double(ceil(GradientBasedPlanner (f, start, goal, 1000));

function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
  % @brief Plans a path through a 2D C-space based on the gradient of potential f
  %        from a start configuration to a destination configuration
  %
  % @param start_coords - Start coordinates
  % @param end_coords - End coordinates denote the coordinates of the start and end
  % @param max_its - An upper bound on the number of iterations
  %
  % @return route - n-by-2 array for the coordinates of the robot as it moves along the route.

  % Max step size:
  max_delta = 1.0;

  % Vicinity of end state:
  max_dist = 2.0;

  % Extract gradient components:
  [gx, gy] = gradient (-f);

  size(gx)
  size(gy)

  % Initialize route:
  route = zeros(max_its, 2);

  % Initialize iteration:
  current_coords = start_coords;
  for i = 1:max_its
    % Append to route:
    route(i,:) = current_coords;

    % Termination criteria:
    if norm(end_coords - current_coords) < max_dist
      route = route(1:i,:);
      break;
    end

    % Update current coords:
    current_ind = sub2ind(size(f), round(current_coords(1,2)), round(current_coords(1,1)));

    current_delta = [gx(current_ind), gy(current_ind)];
    current_delta = max_delta / norm(current_delta) * current_delta;

    current_coords = current_coords + current_delta;
  end
end
