%% Load data

cd ~/go/src/github.com/PrincetonUniversity/ellipswarm/cmd/swarm/data

filename = 'box100_0.h5';

p = h5read(filename, '/particles');
groups = h5read(filename, '/groups');

L = h5readatt(filename, '/config', 'DomainSize');

n = size(p.Dir, 1);
m = size(p.Dir, 2);

%% Plot number of groups

plot(groups)
title(sprintf('median = %d', median(groups)))
xlabel('time (steps)')
ylabel('number of groups')

%% Show simulation

for k=1:m
    plot([-1; p.Pos.X(p.Group(:,k)==0,k)], [-1; p.Pos.Y(p.Group(:,k)==0,k)], '.k')
    hold on
    mg = double(max(p.Group(:,k)));
    for g=1:mg
        plot(p.Pos.X(p.Group(:,k)==g,k), p.Pos.Y(p.Group(:,k)==g,k), '.', 'Color', hsv2rgb([double(g/mg) 1 0.7]), 'MarkerSize', 12)
    end
    hold off
    s = 's';
    if groups(k) == 1, s = ''; end
    title(sprintf('%d group%s', groups(k), s))
    axis([0 L 0 L])
    %drawnow
    pause(0.05)
end

%% Compute group size distribution

dist = [];
for k=1:m
    count = accumarray(p.Group(:,k)+1, 1);
    dist = [dist; count(count > 0)];
end
fprintf('%f ± %f\n', mean(dist), std(dist))
