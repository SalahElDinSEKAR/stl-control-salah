% Define the paths to the ROS 2 setup and workspace setup files
ros2SetupFile = '/opt/ros/humble/setup.bash';
workspaceSetupFile = '~/Documents/stage/ros2-matlab/install/setup.bash';
launchFile = 'Nodes_launch.py'; % Your launch file within the workspace
packageName = 'testing'; % Your package name

% Construct the command to source the setup files and launch the nodes
command = sprintf('source %s && source %s && ros2 launch %s %s', ros2SetupFile, workspaceSetupFile, packageName, launchFile);

% Use the system command to execute the constructed command
[status, cmdout] = system(command);

% Check if the command was executed successfully
if status == 0
    disp('ROS 2 nodes launched successfully.');
else
    disp('Failed to launch ROS 2 nodes.');
    % Display the command output for debugging
    disp(cmdout);
end

%%
% Define the paths to the ROS 2 setup and workspace setup files
ros2SetupFile = '/opt/ros/humble/setup.bash';
workspaceSetupFile = '/path/to/your/ros2_workspace/install/setup.bash';
launchFile = 'Nodes_launch.py'; % Your launch file within the workspace
packageName = 'testing'; % Your package name

% Get the current LD_LIBRARY_PATH
currentLDLibraryPath = getenv('LD_LIBRARY_PATH');

% Define the path to the system libstdc++.so.6 (you may need to adjust this path)
systemLibPath = '/usr/lib/x86_64-linux-gnu';

% Prepend the system library path to LD_LIBRARY_PATH
newLDLibraryPath = [systemLibPath ':' currentLDLibraryPath];

% Set the modified LD_LIBRARY_PATH
setenv('LD_LIBRARY_PATH', newLDLibraryPath);

% Construct the command to source the setup files and launch the nodes
command = sprintf('source %s && source %s && ros2 launch %s %s &', ros2SetupFile, workspaceSetupFile, packageName, launchFile);

% Launch the ROS 2 nodes in the background
system(command);

% Pause for 10 seconds
pause(10);

% Terminate the ROS 2 nodes after 10 seconds
terminate_command = sprintf('pkill -f "ros2 launch %s %s"', packageName, launchFile);
system(terminate_command);

% Check if the nodes were launched and terminated successfully
disp('ROS 2 nodes launched and terminated successfully.');

% Restore the original LD_LIBRARY_PATH
setenv('LD_LIBRARY_PATH', currentLDLibraryPath);
%%
% Define the paths to the ROS 2 setup and workspace setup files
ros2SetupFile = '/opt/ros/humble/setup.bash';
workspaceSetupFile = '~/Documents/stage/ros2-matlab/install/setup.bash';
launchFile = 'Nodes_launch.py'; % Your launch file within the workspace
packageName = 'testing'; % Your package name

% Construct the command to source the setup files and launch the nodes
command = sprintf('source %s && source %s && ros2 launch %s %s', ros2SetupFile, workspaceSetupFile, packageName, launchFile);

% Launch the command in the background using & at the end
system([command, ' &']);

% Pause for 10 seconds
pause(10);

% Find the process ID (PID) of the running ROS 2 launch command
[~, pid] = system('pgrep -f "ros2 launch"');

% Convert PID to number
pid = str2double(pid);

% Kill the process
if ~isnan(pid)
    system(sprintf('kill %d', pid));
    disp('ROS 2 nodes stopped.');
else
    disp('Failed to find the ROS 2 nodes process.');
end
