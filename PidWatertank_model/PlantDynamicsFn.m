function dHdt = PlantDynamicsFn(H, u)
    a = 2;
    b = 5;
    A = 20;

    dHdt = double(b/A*u - a/A*sqrt(H));
end