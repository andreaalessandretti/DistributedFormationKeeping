% Written by Yann Roth during spring semester 2014
function ret_d = build_dist(N, R)
% Build the vector containing all the difference of state wanted between
% the agents for a polygonal formation with the leader (the first agent) in
% the center of the formation in 2-dimension.
%
% di = full_d{i} 
% di(:,j) = pi-pj

position = zeros(2, N);
for i=2:N
    position(1, i)=cos((2*pi/(N-1))*(i-1))*R;
    position(2, i)=sin((2*pi/(N-1))*(i-1))*R;
end

[full_d{1:4}] = deal(zeros(2,4));
d = zeros(2, N);
for i=1:N
    for j=1:N
        d(:, j) = position(:, i) - position(:, j);
    end
    full_d{i} = d;
end

ret_d = full_d;