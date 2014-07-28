% Written by Yann Roth during spring semester 2014
function ret_d = build_dist3D(N, R)
% Build the vector containing all the difference of state wanted between
% the agents for a polygonal formation on a plane parallel to YZ with the 
% leader (the first agent) in the center of the formation in 3-dimension.
position = zeros(3, N);
for i=2:N
    position(2, i)=cos((2*pi/(N-1))*(i-1))*R;
    position(3, i)=sin((2*pi/(N-1))*(i-1))*R;
    position(1, i)=0;
end

[full_d{1:N}] = deal(zeros(3,N));
d = zeros(3, N);
for i=1:N
    for j=1:N
        d(:, j) = position(:, i) - position(:, j);
    end
    full_d{i} = d;
end

ret_d = full_d;