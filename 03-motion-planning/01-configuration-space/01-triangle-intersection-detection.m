function flag = triangle_intersection(P1, P2)
    % @brief whether the two triangles intersect with each other
    % @param P1 - 3-by-2 array, vertices of the first triangle
    % @param P2 - 3-by-2 array, vertices of the second triangle
    % @return true if the triangles overlap and false otherwise

    % Initialize flag:
    flag = false;

    % First enumerate all 9 edge combinations:
    if (...
        is_edges_intersect(P1([1,2],:), P2([1,2],:)) || ...
        is_edges_intersect(P1([1,2],:), P2([2,3],:)) || ...
        is_edges_intersect(P1([1,2],:), P2([1,3],:)) || ...
        is_edges_intersect(P1([2,3],:), P2([1,2],:)) || ...
        is_edges_intersect(P1([2,3],:), P2([2,3],:)) || ...
        is_edges_intersect(P1([2,3],:), P2([1,3],:)) || ...
        is_edges_intersect(P1([1,3],:), P2([1,2],:)) || ...
        is_edges_intersect(P1([1,3],:), P2([2,3],:)) || ...
        is_edges_intersect(P1([1,3],:), P2([1,3],:)) ...
    )
        flag = true;
    else
    % If no edge intersection, test whether two triangles are nested:
        N = size(P1, 1);
        for n = 1:N
            flag = is_point_inside_triangle(P1(n,:), P2);
            if flag
                break
            end
        end

        if ~flag
            N = size(P2, 1);
            for n = 1:N
                flag = is_point_inside_triangle(P2(n,:), P1);
                if flag
                    break
                end
            end
        end
    end
end

function value = cross2d(u, v)
    % @brief cross product of 2d vectors
    % @param u - vector u
    % @param v - vector v
    % @return z component of the two vector's cross product
    value = u(1,1)*v(1,2) - u(1,2)*v(1,1);
end

function flag = is_edges_intersect(E1, E2)
    % @brief whether two edges intersect
    %
    % http://thirdpartyninjas.com/blog/2008/10/07/line-segment-intersection/
    %
    % @param E1 - 2-by-2 array, endpoints of first edge
    % @param E2 - 2-by-2 array, endpoints of second edge
    % @return true if two edges intersect false otherwise

    x1 = E1(2,:) - E1(1,:);
    x2 = E2(2,:) - E2(1,:);
    x3 = E1(1,:) - E2(1,:);

    d = cross2d(x1,x2);
    u = cross2d(x2,x3);
    v = cross2d(x1,x3);

    if (d < 0)
        u = -u;
        v = -v;
        d = -d;
    end

    flag = (u >= 0) && (u <= d) && (v >= 0) &&(v <= d);
end

function flag = is_point_inside_triangle(p, T)
    % @brief whether point p lies inside triangle T
    %
    % http://thirdpartyninjas.com/blog/2008/10/07/line-segment-intersection/
    %
    % @param p - 1-by-2 array, coordinates of point p
    % @param T - 3-by-2 array, vertices of triangle T
    % @return true if p lies inside and false otherwise

    x1 = T(3,:)-T(1,:);
    x2 = T(2,:)-T(1,:);
    x3 = p - T(1,:);

    u = cross2d(x3,x1);
    v = cross2d(x2,x3);
    d = cross2d(x2,x1);

    if (d < 0)
        u = -u;
        v = -v;
        d = -d;
    end

    flag = (u >= 0) && (v >= 0) && ((u+v) <= d);
end
