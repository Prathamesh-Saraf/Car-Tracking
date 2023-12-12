% N - 1, E - 2, S - 3, W - 4

function intersectionMap = get_map()
    intersectionKeys = {'[-500 -200]', '[-350 -200]', '[-350 -800]', '[-750 -800]', '[-950 -800]', ...
                    '[-950 -400]', '[-950 200]', '[-750 200]', '[-550 200]', '[-550 -400]', '[-750 -400]'};
    intersectionValues = {
        [1, 2, 3],
        [1, 3, 4],
        [1, 3, 4],
        [1, 2, 3, 4],
        [1, 2, 3],
        [1, 2, 3],
        [1, 2, 3],
        [1, 2, 3, 4],
        [1, 3, 4],
        [1, 3, 4],
        [1, 2, 3, 4]
    };
    intersectionMap = containers.Map(intersectionKeys, intersectionValues);

end