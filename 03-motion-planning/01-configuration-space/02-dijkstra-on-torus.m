function route = DijkstraTorus (input_map, start_coords, dest_coords, drawMap)
  % @brief Run Dijkstra's algorithm on a torus grid
  %
  %
  % @param input_map - A logical array where
  %   1. the freespace cells are false or 0
  %   2. the obstacles are true or 1
  % @param start_coords - Coordinates of the start cell
  % @param dest_coords - Coordinates of the end cell
  %
  % @return route - An array containing the linear indices of the cells along the
  % shortest route from start to dest or an empty array if there is no route
  %

  % set up color map for display
  % 1 - white - clear cell
  % 2 - black - obstacle
  % 3 - red = visited
  % 4 - blue  - on list
  % 5 - green - start
  % 6 - yellow - destination
  cmap = [...
    1 1 1; ...
    0 0 0; ...
    1 0 0; ...
    0 0 1; ...
    0 1 0; ...
    1 1 0;...
  ];
  colormap(cmap);

  % grid dimensions:
  [nrows, ncols] = size(input_map);

  % adjacency:
  adjacency = [...
    -1, +0;...
    +1, +0;...
    +0, -1;...
    +0, +1;...
  ];
  nadjs = size(adjacency, 1);

  % map - a table that keeps track of the state of each grid cell
  map = zeros(nrows,ncols);
  map(~input_map) = 1;  % Mark free cells
  map(input_map)  = 2;  % Mark obstacle cells

  % Generate linear indices of start and dest nodes
  start_node = sub2ind(size(map), start_coords(1), start_coords(2));
  dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));
  map(start_node) = 5;
  map(dest_node)  = 6;

  % Initialize distance array
  distances = Inf(nrows,ncols);

  % For each grid cell this array holds the index of its parent
  parent = zeros(nrows,ncols);

  % Initialize frontier:
  distances(start_node) = 0;

  % Main Loop
  while true
      % Draw current map
      map(start_node) = 5;
      map(dest_node) = 6;

      if drawMap
          image(1.5, 1.5, map);
          grid on;
          axis image;
          drawnow;
      end

      % Find the node with the minimum distance
      [min_dist, current] = min(distances(:));

      if ((current == dest_node) || isinf(min_dist))
          break
      end

      % Update map
      map(current) = 3;         % mark current node as visited
      distances(current) = Inf; % remove this node from further consideration

      % Compute row, column coordinates of current node
      [i, j] = ind2sub(size(distances), current);

      % Visit each neighbor of the current node and update the map, distances
      % and parent tables appropriately.
      for k = 1:nadjs
        next_row = mod(i - 1 + adjacency(k,1) + nrows, nrows) + 1;
        next_col = mod(j - 1 + adjacency(k,2) + ncols, ncols)  + 1;
        next_ind = sub2ind(size(map), next_row, next_col);

        if (...
          (map(next_row, next_col) ~= 3) && ...
          (map(next_row, next_col) ~= 5) && ...
          distances(next_ind) > (min_dist + 1) ...
        )
          parent(next_row, next_col) = current;
          distances(next_ind) = min_dist + 1;
          map(next_ind) = 4;
        end
      end
  end

  if (isinf(distances(dest_node)))
      route = [];
  else
      route = dest_node;

      while (parent(route(1)) ~= 0)
          route = [parent(route(1)), route];
      end
  end
end
