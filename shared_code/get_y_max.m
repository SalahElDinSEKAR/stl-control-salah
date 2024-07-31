function [t1, ymean, t2, ymax ] = get_y_max(R)

if ~isa(R,'BreachRequirement')
    %B = R;
    %Ry = BreachRequirement({'alw (y[t]>0)'}); 
    %og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
    %Ry.AddPostProcess(og);
    %Ry.Eval(B);
    Yvals = R.BrSet.GetExprValues('sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
else
    Yvals = R.BrSet.GetExprValues('sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
end

t = R.GetTime();
ymax = 0*t - inf;
Y = [];
for isig = 1:numel(Yvals)
    Yi  = Yvals{isig};
    Y(isig,:) = Yi; 
    ymax= max([Yi ; ymax]);    
end

[t1, ymean] = RobustEv(t, mean(Y), [0 inf]);
[t2, ymax ] = RobustEv(t, ymax,    [0 inf]);    

if t1(end)== t1(end-1)
    t1 = t1(1:end-1);
    ymean = ymean(1:end-1);
end

if t2(end)== t2(end-1)
    t2 = t2(1:end-1);
    ymax = ymax(1:end-1);
end
