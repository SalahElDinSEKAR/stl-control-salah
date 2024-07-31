function X = downsample_and_format_data(pose_csv)
    % Read the data from the CSV file
    data = readtable(pose_csv);

    % Extract columns assuming the CSV file has columns: 'time' and 'data0' to 'data8'
    time = data.time;
    values0 = data.data0; % H
    values1 = data.data1; % Hp
    values2 = data.data2; % Hpp
    values3 = data.data3; % ref
    values4 = data.data4; % refp
    values5 = data.data5; % refpp
    values6 = data.data6; % V
    values7 = data.data7; % Vp
    values8 = data.data8;

    % Define the downsampling factor
    downsampling_factor = 10;

    % Initialize downsampled arrays
    downsampled_time = [];
    downsampled_values0 = [];
    downsampled_values1 = [];
    downsampled_values2 = [];
    downsampled_values3 = [];
    downsampled_values4 = [];
    downsampled_values5 = [];
    downsampled_values6 = [];
    downsampled_values7 = [];
    downsampled_values8 = [];

    % Loop through the data with an appropriate offset
    for i = 1:downsampling_factor:1010
        downsampled_time = [downsampled_time; time(i)];
        downsampled_values0 = [downsampled_values0; values0(i)];
        downsampled_values3 = [downsampled_values3; values3(i)];
        downsampled_values6 = [downsampled_values6; values6(i)];
        
        if i > downsampling_factor
            % Ensure that Hp of current downsampled point is H of previous downsampled point
            downsampled_values1 = [downsampled_values1; values0(i - downsampling_factor)];
            % Ensure that refp of current downsampled point is ref of previous downsampled point
            downsampled_values4 = [downsampled_values4; values3(i - downsampling_factor)];
            % Ensure that Vp of current downsampled point is V of previous downsampled point
            downsampled_values7 = [downsampled_values7; values6(i - downsampling_factor)];
            
            if i > 2 * downsampling_factor
                % Ensure that Hpp of current downsampled point is Hp of previous downsampled point
                downsampled_values2 = [downsampled_values2; values0(i - 2 * downsampling_factor)];
                % Ensure that refpp of current downsampled point is refp of previous downsampled point
                downsampled_values5 = [downsampled_values5; values3(i - 2 * downsampling_factor)];
            else
                downsampled_values2 = [downsampled_values2; values0(1)]; % Use first value of H
                downsampled_values5 = [downsampled_values5; values3(1)]; % Use first value of ref
            end
        else
            downsampled_values1 = [downsampled_values1; values0(1)]; % Use first value of H
            downsampled_values2 = [downsampled_values2; values0(1)]; % Use first value of H
            downsampled_values4 = [downsampled_values4; values3(1)]; % Use first value of ref
            downsampled_values5 = [downsampled_values5; values3(1)]; % Use first value of ref
            downsampled_values7 = [downsampled_values7; values6(1)]; % Use first value of V
        end
        
        downsampled_values8 = [downsampled_values8; values8(i)];
    end

    % Combine downsampled data into a table
    downsampled_data = table(downsampled_time, downsampled_values0, downsampled_values1, downsampled_values2, downsampled_values3, downsampled_values4, downsampled_values5, downsampled_values6, downsampled_values7, downsampled_values8, ...
        'VariableNames', {'time', 'Value0', 'Value1', 'Value2', 'Value3', 'Value4', 'Value5', 'Value6', 'Value7', 'Value8'});

    % Convert table to array
    X = table2array(downsampled_data);

    % Transpose the array and remove the first row (time)
    X = X';
    X(1, :) = [];
end
